/*

https://github.com/osmcode/osm-data-anomaly-detection

Copyright (C) 2019-2021  Jochen Topf <jochen@topf.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <getopt.h>
#include <iostream>
#include <string>
#include <system_error>
#include <vector>

#include <osmium/handler.hpp>
#include <osmium/index/id_set.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/io/any_output.hpp>
#include <osmium/osm/location.hpp>
#include <osmium/osm/timestamp.hpp>
#include <osmium/util/memory.hpp>
#include <osmium/util/memory_mapping.hpp>
#include <osmium/util/progress_bar.hpp>
#include <osmium/util/verbose_output.hpp>
#include <osmium/visitor.hpp>

#include <gdalcpp.hpp>

#include "utils.hpp"

static const char* const program_name = "odad-find-colocated-nodes";

struct options_type {
    osmium::Timestamp before_time{osmium::end_of_time()};
    bool verbose = true;
};

struct stats_type {
    uint64_t locations_with_colocated_nodes = 0;
    uint64_t colocated_nodes = 0;
    uint64_t ways_referencing_colocated_nodes = 0;
    uint64_t relations_referencing_colocated_nodes = 0;
};

// must be a power of 2
// must change build_filename() function if you change this
constexpr const unsigned int num_buckets = 1U << 8U;

std::string build_filename(const std::string& dirname, unsigned int n) {
    static const char* lookup_hex = "0123456789abcdef";

    std::string filename = dirname;
    filename += "/locations_";
    filename += lookup_hex[(n >> 4U) & 0xfU];
    filename += lookup_hex[n & 0xfU];
    filename += ".dat";

    return filename;
}

class Bucket {

    // maximum size of bucket before it gets flushed
    constexpr static const int max_bucket_size = 512 * 1024;

    std::vector<osmium::Location> m_data;

    std::string m_filename;

    int m_fd;

public:

    Bucket(const std::string& dirname, unsigned int n) :
        m_filename(build_filename(dirname, n)),
        m_fd(::open(m_filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666)) { // NOLINT(hicpp-signed-bitwise)
        if (m_fd < 0) {
            throw std::system_error{errno, std::system_category(), std::string{"Can't open file '"} + m_filename + "'"};
        }
        m_data.reserve(max_bucket_size);
    }

    Bucket(const Bucket&) = delete;
    Bucket& operator=(const Bucket&) = delete;

    Bucket(Bucket&&) = default;
    Bucket& operator=(Bucket&&) = default;

    ~Bucket() {
        try {
            flush();
        } catch (...) {
            // ignore exceptions
        }
        ::close(m_fd);
    }

    void set(const osmium::Location& location) {
        m_data.push_back(location);
        if (m_data.size() == max_bucket_size) {
            flush();
        }
    }

    void flush() {
        if (m_data.empty()) {
            return;
        }

        const auto bytes = m_data.size() * sizeof(osmium::Location);
        const auto length = ::write(m_fd, m_data.data(), bytes);
        if (length != long(bytes)) { // NOLINT(google-runtime-int)
            throw std::system_error{errno, std::system_category(), std::string{"can't write to file '"} + m_filename + "'"};
        }

        m_data.clear();
    }

}; // class Bucket

void extract_locations(const osmium::io::File& input_file, const std::string& directory, const options_type& options) {
    std::vector<Bucket> buckets;
    buckets.reserve(num_buckets);
    for (unsigned int i = 0; i < num_buckets; ++i) {
        buckets.emplace_back(directory, i);
    }

    osmium::io::Reader reader{input_file, osmium::osm_entity_bits::node};
    osmium::ProgressBar progress_bar{reader.file_size(), display_progress()};
    while (osmium::memory::Buffer buffer = reader.read()) {
        progress_bar.update(reader.offset());
        for (const auto& node : buffer.select<osmium::Node>()) {
            if (node.timestamp() < options.before_time) {
                const auto bucket_num = static_cast<uint32_t>(node.location().x()) & (num_buckets - 1);
                buckets[bucket_num].set(node.location());
            }
        }
    }
    progress_bar.done();
    reader.close();

    for (auto& bucket : buckets) {
        bucket.flush();
    }
}

std::vector<osmium::Location> find_locations(const std::string& directory) {
    std::vector<osmium::Location> locations;

    for (unsigned int i = 0; i < num_buckets; ++i) {
        const auto filename = build_filename(directory, i);
        const int fd = ::open(filename.c_str(), O_RDONLY | O_CLOEXEC); // NOLINT(hicpp-signed-bitwise)
        if (fd < 0) {
            throw std::system_error{errno, std::system_category(), std::string{"Can't open file '"} + filename + "'"};
        }
        const auto file_size = osmium::util::file_size(fd);

        if (file_size > 0) {
            osmium::util::TypedMemoryMapping<osmium::Location> m_mapping{file_size / sizeof(osmium::Location), osmium::util::MemoryMapping::mapping_mode::write_private, fd };

            std::sort(m_mapping.begin(), m_mapping.end());

            auto it = m_mapping.begin();
            while ((it = std::adjacent_find(it, m_mapping.end())) != m_mapping.end()) {
                locations.push_back(*it);
                ++it;
                ++it;
            }
        }

        ::close(fd);
        ::unlink(filename.c_str());
    }

    std::sort(locations.begin(), locations.end());
    const auto last = std::unique(locations.begin(), locations.end());
    locations.erase(last, locations.end());

    return locations;
}

class CheckHandler : public HandlerWithDB {

    stats_type m_stats;
    gdalcpp::Layer m_layer_colocated_nodes;
    osmium::io::Writer& m_writer;
    const std::vector<osmium::Location>& m_locations;
    osmium::index::IdSetSmall<osmium::unsigned_object_id_type> m_node_ids;
    bool m_nodes_done = false;

public:

    CheckHandler(const std::string& output_dirname, osmium::io::Writer& writer, const std::vector<osmium::Location>& locations) :
        HandlerWithDB(output_dirname + "/geoms-colocated-nodes.db"),
        m_layer_colocated_nodes(m_dataset, "colocated_nodes", wkbPoint, {"SPATIAL_INDEX=NO"}),
        m_writer(writer),
        m_locations(locations) {
        m_layer_colocated_nodes.add_field("node_id", OFTReal, 12);
        m_layer_colocated_nodes.add_field("timestamp", OFTString, 20);
        m_stats.locations_with_colocated_nodes = locations.size();
    }

    void node(const osmium::Node& node) {
        const auto r = std::equal_range(m_locations.begin(),
                                        m_locations.end(), node.location());

        if (r.first != r.second) {
            m_node_ids.set(node.positive_id());
            ++m_stats.colocated_nodes;
            m_writer(node);
            gdalcpp::Feature feature{m_layer_colocated_nodes, m_factory.create_point(node.location())};
            feature.set_field("node_id", static_cast<double>(node.id()));
            const auto ts = node.timestamp().to_iso();
            feature.set_field("timestamp", ts.c_str());
            feature.add_to_layer();
        }
    }

    void way(const osmium::Way& way) {
        if (!m_nodes_done) {
            m_nodes_done = true;
            m_node_ids.sort_unique();
        }

        for (const auto& node_ref : way.nodes()) {
            if (m_node_ids.get_binary_search(node_ref.positive_ref())) {
                ++m_stats.ways_referencing_colocated_nodes;
                m_writer(way);
                break;
            }
        }
    }

    void relation(const osmium::Relation& relation) {
        for (const auto& member : relation.members()) {
            if (member.type() == osmium::item_type::node && m_node_ids.get_binary_search(member.positive_ref())) {
                ++m_stats.relations_referencing_colocated_nodes;
                m_writer(relation);
                break;
            }
        }
    }

    const stats_type& stats() const noexcept {
        return m_stats;
    }

}; // class CheckHandler

static void print_help() {
    std::cout << program_name << " [OPTIONS] OSM-FILE OUTPUT-DIR\n\n"
              << "Find nodes having the exact same location.\n"
              << "\nOptions:\n"
              << "  -a, --min-age=DAYS      Only include objects at least DAYS days old\n"
              << "  -b, --before=TIMESTAMP  Only include objects changed last before\n"
              << "                          this time (format: yyyy-mm-ddThh:mm:ssZ)\n"
              << "  -h, --help              This help message\n"
              << "  -q, --quiet             Work quietly\n"
              ;
}

static options_type parse_command_line(int argc, char* argv[]) {
    static struct option long_options[] = {
        {"age",     required_argument, nullptr, 'a'},
        {"before",  required_argument, nullptr, 'b'},
        {"help",          no_argument, nullptr, 'h'},
        {"quiet",         no_argument, nullptr, 'q'},
        {nullptr, 0, nullptr, 0}
    };

    options_type options;

    while (true) {
        const int c = getopt_long(argc, argv, "a:b:hq", long_options, nullptr);
        if (c == -1) {
            break;
        }

        switch (c) {
            case 'a':
                if (options.before_time != osmium::end_of_time()) {
                    std::cerr << "You can not use both -a,--age and -b,--before together\n";
                    std::exit(2);
                }
                options.before_time = build_timestamp(optarg);
                break;
            case 'b':
                if (options.before_time != osmium::end_of_time()) {
                    std::cerr << "You can not use both -a,--age and -b,--before together\n";
                    std::exit(2);
                }
                options.before_time = osmium::Timestamp{optarg};
                break;
            case 'h':
                print_help();
                std::exit(0);
            case 'q':
                options.verbose = false;
                break;
            default:
                std::exit(2);
        }
    }

    const int remaining_args = argc - optind;
    if (remaining_args != 2) {
        std::cerr << "Usage: " << program_name << " [OPTIONS] OSM-FILE OUTPUT-DIR\n"
                  << "Call '" << program_name << " --help' for usage information.\n";
        std::exit(2);
    }

    return options;
}

int main(int argc, char* argv[]) try {
    const auto options = parse_command_line(argc, argv);

    osmium::util::VerboseOutput vout{options.verbose};
    vout << "Starting " << program_name << "...\n";

    const std::string input_filename{argv[optind]};
    const std::string output_dirname{argv[optind + 1]};

    vout << "Command line options:\n";
    vout << "  Reading from file '" << input_filename << "'\n";
    vout << "  Writing to directory '" << output_dirname << "'\n";
    if (options.before_time == osmium::end_of_time()) {
        vout << "  Get all objects independent of change timestamp (change with --age, -a or --before, -b)\n";
    } else {
        vout << "  Get only objects last changed before: " << options.before_time << " (change with --age, -a or --before, -b)\n";
    }
    const osmium::io::File input_file{input_filename};
    const osmium::io::File output_file{output_dirname + "/colocated-nodes.osm.pbf"};

    osmium::io::Header header;
    header.set("generator", program_name);
    osmium::io::Writer writer{output_file, header, osmium::io::overwrite::allow};

    vout << "Extracting all locations...\n";
    extract_locations(input_file, output_dirname, options);

    vout << "Finding locations with multiple nodes...\n";
    const auto locations = find_locations(output_dirname);
    vout << "Found " << locations.size() << " locations with multiple nodes.\n";

    vout << "Copying colocated nodes and the ways/relations referencing them...\n";
    osmium::io::Reader reader{input_file, osmium::osm_entity_bits::nwr};

    LastTimestampHandler last_timestamp_handler;
    CheckHandler handler{output_dirname, writer, locations};

    osmium::ProgressBar progress_bar{reader.file_size(), display_progress()};
    while (osmium::memory::Buffer buffer = reader.read()) {
        progress_bar.update(reader.offset());
        osmium::apply(buffer, last_timestamp_handler, handler);
    }
    progress_bar.done();

    reader.close();
    writer.close();

    vout << "Writing out stats...\n";
    const auto last_time{last_timestamp_handler.get_timestamp()};
    write_stats(output_dirname + "/stats-colocated-nodes.db", last_time, [&](std::function<void(const char*, uint64_t)>& add){
        add("locations_with_colocated_nodes", handler.stats().locations_with_colocated_nodes);
        add("colocated_nodes", handler.stats().colocated_nodes);
        add("ways_referencing_colocated_nodes", handler.stats().ways_referencing_colocated_nodes);
        add("relations_referencing_colocated_nodes", handler.stats().relations_referencing_colocated_nodes);
    });

    osmium::MemoryUsage memory_usage;
    if (memory_usage.peak() != 0) {
        vout << "Peak memory usage: " << memory_usage.peak() << " MBytes\n";
    }

    vout << "Done with " << program_name << ".\n";

    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    std::exit(1);
}

