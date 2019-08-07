/*

https://github.com/osmcode/osm-data-anomaly-detection

Copyright (C) 2019  Jochen Topf <jochen@topf.org>

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

#include <cstdlib>
#include <ctime>
#include <functional>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <string>

#include <osmium/index/id_set.hpp>
#include <osmium/index/nwr_array.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/io/any_output.hpp>
#include <osmium/osm/timestamp.hpp>
#include <osmium/tags/taglist.hpp>
#include <osmium/tags/tags_filter.hpp>
#include <osmium/util/memory.hpp>
#include <osmium/util/progress_bar.hpp>
#include <osmium/util/verbose_output.hpp>

#include <gdalcpp.hpp>

#include "utils.hpp"

static const char* program_name = "odad-find-orphans";

struct options_type {
    osmium::Timestamp before_time{osmium::end_of_time()};
    bool verbose = true;
    bool untagged = true;
    bool tagged = true;
};

struct stats_type {
    uint64_t orphan_nodes = 0;
    uint64_t orphan_ways = 0;
    uint64_t orphan_relations = 0;
};

using id_set_type = osmium::index::IdSetDense<osmium::unsigned_object_id_type>;

static osmium::nwr_array<id_set_type> create_index_of_referenced_objects(const osmium::io::File& input_file, osmium::ProgressBar& progress_bar) {
    osmium::nwr_array<id_set_type> index;

    osmium::io::Reader reader{input_file, osmium::osm_entity_bits::way | osmium::osm_entity_bits::relation};

    while (osmium::memory::Buffer buffer = reader.read()) {
        progress_bar.update(reader.offset());

        for (const auto& object : buffer.select<osmium::OSMObject>()) {
            if (object.type() == osmium::item_type::way) {
                for (const auto& node_ref : static_cast<const osmium::Way&>(object).nodes()) {
                    index(osmium::item_type::node).set(node_ref.positive_ref());
                }
            } else if (object.type() == osmium::item_type::relation) {
                for (const auto& member : static_cast<const osmium::Relation&>(object).members()) {
                    index(member.type()).set(member.positive_ref());
                }
            }
        }
    }

    reader.close();

    return index;
}

class CheckHandler : public HandlerWithDB {

    options_type m_options;
    stats_type m_stats;

    gdalcpp::Layer m_layer_orphan_nodes;
    gdalcpp::Layer m_layer_orphan_ways;

    osmium::TagsFilter m_filter{false};

    osmium::nwr_array<id_set_type>& m_index;
    osmium::nwr_array<std::unique_ptr<osmium::io::Writer>> m_writers;

public:

    CheckHandler(const std::string& output_dirname, const options_type& options, osmium::nwr_array<id_set_type>& index) :
        HandlerWithDB(output_dirname + "/geoms-orphans.db"),
        m_options(options),
        m_layer_orphan_nodes(m_dataset, "orphan_nodes", wkbPoint, {"SPATIAL_INDEX=NO"}),
        m_layer_orphan_ways(m_dataset, "orphan_ways", wkbLineString, {"SPATIAL_INDEX=NO"}),
        m_index(index) {
        m_layer_orphan_nodes.add_field("node_id", OFTReal, 12);
        m_layer_orphan_nodes.add_field("timestamp", OFTString, 20);

        m_layer_orphan_ways.add_field("way_id", OFTInteger, 10);
        m_layer_orphan_ways.add_field("timestamp", OFTString, 20);

        m_filter.add_rule(true, "created_by");
        m_filter.add_rule(true, "source");

        osmium::io::Header header;
        header.set("generator", program_name);
        m_writers(osmium::item_type::node).reset(new osmium::io::Writer{output_dirname + "/n-orphans.osm.pbf", header, osmium::io::overwrite::allow});
        m_writers(osmium::item_type::way).reset(new osmium::io::Writer{output_dirname + "/w-orphans.osm.pbf", header, osmium::io::overwrite::allow});
        m_writers(osmium::item_type::relation).reset(new osmium::io::Writer{output_dirname + "/r-orphans.osm.pbf", header, osmium::io::overwrite::allow});
    }

    void node(const osmium::Node& node) {
        if (node.timestamp() >= m_options.before_time) {
            return;
        }

        if (m_index(osmium::item_type::node).get(node.positive_id())) {
            return;
        }

        if ((m_options.untagged && node.tags().empty()) ||
                (m_options.tagged && !node.tags().empty() && osmium::tags::match_all_of(node.tags(), std::cref(m_filter)))) {
            (*m_writers(osmium::item_type::node))(node);
            ++m_stats.orphan_nodes;
            gdalcpp::Feature feature{m_layer_orphan_nodes, m_factory.create_point(node)};
            feature.set_field("node_id", static_cast<double>(node.id()));
            const auto ts = node.timestamp().to_iso();
            feature.set_field("timestamp", ts.c_str());
            feature.add_to_layer();
        }
    }

    void way(const osmium::Way& way) {
        if (way.timestamp() >= m_options.before_time) {
            return;
        }

        if (m_index(osmium::item_type::way).get(way.positive_id())) {
            return;
        }

        if ((m_options.untagged && way.tags().empty()) ||
                (m_options.tagged && !way.tags().empty() && osmium::tags::match_all_of(way.tags(), std::cref(m_filter)))) {
            (*m_writers(osmium::item_type::way))(way);
            ++m_stats.orphan_ways;
            try {
                gdalcpp::Feature feature{m_layer_orphan_ways, m_factory.create_linestring(way)};
                feature.set_field("way_id", static_cast<double>(way.id()));
                const auto ts = way.timestamp().to_iso();
                feature.set_field("timestamp", ts.c_str());
                feature.add_to_layer();
            } catch (const osmium::geometry_error&) {
                // ignore geometry errors
            }
        }
    }

    void relation(const osmium::Relation& relation) {
        if (relation.timestamp() >= m_options.before_time) {
            return;
        }

        if (m_index(osmium::item_type::relation).get(relation.positive_id())) {
            return;
        }

        if ((m_options.untagged && relation.tags().empty()) ||
                (m_options.tagged && !relation.tags().empty() && osmium::tags::match_all_of(relation.tags(), std::cref(m_filter)))) {
            (*m_writers(osmium::item_type::relation))(relation);
            ++m_stats.orphan_relations;
        }
    }

    void close() {
        m_writers(osmium::item_type::node)->close();
        m_writers(osmium::item_type::way)->close();
        m_writers(osmium::item_type::relation)->close();
    }

    const stats_type stats() const noexcept {
        return m_stats;
    }

}; // class CheckHandler

static void print_help() {
    std::cout << program_name << " [OPTIONS] OSM-FILE OUTPUT-DIR\n\n"
              << "Find objects that are unreferenced and untagged (or minimally tagged).\n"
              << "\nOptions:\n"
              << "  -a, --min-age=DAYS      Only include objects at least DAYS days old\n"
              << "  -b, --before=TIMESTAMP  Only include objects changed last before\n"
              << "                          this time (format: yyyy-mm-ddThh:mm:ssZ)\n"
              << "  -h, --help              This help message\n"
              << "  -q, --quiet             Work quietly\n"
              << "  -u, --untagged-only     Untagged objects only\n"
              << "  -U, --no-untagged       No untagged objects\n"
              ;
}

static options_type parse_command_line(int argc, char* argv[]) {
    static struct option long_options[] = {
        {"age",     required_argument, nullptr, 'a'},
        {"before",  required_argument, nullptr, 'b'},
        {"help",          no_argument, nullptr, 'h'},
        {"quiet",         no_argument, nullptr, 'q'},
        {"untagged-only", no_argument, nullptr, 'u'},
        {"no-untagged",   no_argument, nullptr, 'U'},
        {nullptr, 0, nullptr, 0}
    };

    options_type options;

    while (true) {
        const int c = getopt_long(argc, argv, "a:b:hquU", long_options, nullptr);
        if (c == -1) {
            break;
        }

        switch (c) {
            case 'a':
                if (options.before_time != osmium::end_of_time()) {
                    std::cerr << "You can not use both -a,--age and -b,--before together\n";
                    std::exit(2);
                }
                options.before_time = osmium::Timestamp{std::time(nullptr) - std::atoi(optarg) * 60 * 60 * 24};
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
            case 'u':
                options.tagged = false;
                break;
            case 'U':
                options.untagged = false;
                break;
            default:
                std::exit(2);
        }
    }

    if (!options.tagged && !options.untagged) {
        std::cerr << "Can not use -u,--untagged-only and -U,--no-untagged together.\n";
        std::exit(2);
    }

    const int remaining_args = argc - optind;
    if (remaining_args != 2) {
        std::cerr << "Usage: " << program_name << " [OPTIONS] OSM-FILE OUTPUT-DIR\n"
                  << "Call '" << program_name << " --help' for usage information.\n";
        std::exit(2);
    }

    return options;
}

int main(int argc, char* argv[]) {
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
    vout << "  Finding untagged objects: " << (options.untagged ? "yes" : "no") << " (change with --untagged, -u)\n";
    vout << "  Finding tagged objects: " << (options.tagged ? "yes" : "no") << " (change with --no-untagged, -U)\n";

    const osmium::io::File input_file{input_filename};

    const auto file_size = osmium::util::file_size(input_filename);
    osmium::ProgressBar progress_bar{file_size * 2, display_progress()};

    vout << "First pass: Creating index of referenced objects...\n";
    auto index = create_index_of_referenced_objects(input_file, progress_bar);
    progress_bar.file_done(file_size);

    progress_bar.remove();
    vout << "Second pass: Writing out non-referenced and untagged objects...\n";

    LastTimestampHandler last_timestamp_handler;
    CheckHandler handler{output_dirname, options, index};

    osmium::io::Reader reader{input_file, osmium::osm_entity_bits::nwr};

    while (osmium::memory::Buffer buffer = reader.read()) {
        progress_bar.update(reader.offset());
        osmium::apply(buffer, last_timestamp_handler, handler);
    }
    progress_bar.done();

    handler.close();
    reader.close();

    vout << "Writing out stats...\n";
    const auto last_time{last_timestamp_handler.get_timestamp()};
    write_stats(output_dirname + "/stats-orphans.db", last_time, [&](std::function<void(const char*, uint64_t)>& add){
        add("orphan_nodes", handler.stats().orphan_nodes);
        add("orphan_ways", handler.stats().orphan_ways);
        add("orphan_relations", handler.stats().orphan_relations);
    });

    osmium::MemoryUsage memory_usage;
    if (memory_usage.peak() != 0) {
        vout << "Peak memory usage: " << memory_usage.peak() << " MBytes\n";
    }

    vout << "Done with " << program_name << ".\n";

    return 0;
}

