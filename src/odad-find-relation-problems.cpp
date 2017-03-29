/*

https://github.com/osmcode/osm-data-anomaly-detection

Copyright (C) 2017  Jochen Topf <jochen@topf.org>

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
#include <cstring>
#include <ctime>
#include <getopt.h>
#include <iostream>
#include <string>

#include <osmium/io/any_input.hpp>
#include <osmium/io/any_output.hpp>
#include <osmium/io/file.hpp>
#include <osmium/osm/entity_bits.hpp>
#include <osmium/osm/object.hpp>
#include <osmium/util/memory.hpp>
#include <osmium/util/progress_bar.hpp>
#include <osmium/util/verbose_output.hpp>
#include <osmium/visitor.hpp>

#include "utils.hpp"

static const char* program_name = "odad-find-relation-problems";
static const size_t min_members_of_large_relations = 1000;

struct options_type {
    osmium::Timestamp before_time{osmium::end_of_time()};
    bool verbose = true;
};

struct stats_type {
    uint64_t relation_members = 0;
    uint64_t no_members = 0;
    uint64_t large_relations = 0;
    uint64_t multipolygon_node_member = 0;
    uint64_t multipolygon_relation_member = 0;
    uint64_t multipolygon_unknown_role = 0;
    uint64_t multipolygon_empty_role = 0;
    uint64_t multipolygon_area_tag = 0;
    uint64_t multipolygon_boundary_tag = 0;
};

class CheckHandler : public osmium::handler::Handler {

    options_type m_options;
    stats_type m_stats;

    osmium::io::Writer m_writer_no_member;
    osmium::io::Writer m_writer_large_relations;
    osmium::io::Writer m_writer_multipolygon_non_way_member;
    osmium::io::Writer m_writer_multipolygon_unknown_role;
    osmium::io::Writer m_writer_multipolygon_empty_role;
    osmium::io::Writer m_writer_multipolygon_area_tag;
    osmium::io::Writer m_writer_multipolygon_boundary_tag;

public:

    CheckHandler(const std::string& directory, const options_type& options, const osmium::io::Header& header) :
        m_options(options),
        m_writer_no_member(directory + "/relation-no-member.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_large_relations(directory + "/large-relations.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_non_way_member(directory + "/relation-multipolygon-non-way-member.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_unknown_role(directory + "/relation-multipolygon-unknown-role.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_empty_role(directory + "/relation-multipolygon-empty-role.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_area_tag(directory + "/relation-multipolygon-area-tag.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_boundary_tag(directory + "/relation-multipolygon-boundary-tag.osm.pbf", header, osmium::io::overwrite::allow) {
    }

    void multipolygon_relation(const osmium::Relation& relation) {
        for (const auto& member : relation.members()) {
            if (member.type() != osmium::item_type::way) {
                if (member.type() == osmium::item_type::node) {
                    ++m_stats.multipolygon_node_member;
                } else {
                    ++m_stats.multipolygon_relation_member;
                }
                m_writer_multipolygon_non_way_member(relation);
            }
            if (std::strcmp(member.role(), "inner") &&
                std::strcmp(member.role(), "outer") &&
                std::strcmp(member.role(), "")) {
                ++m_stats.multipolygon_unknown_role;
                m_writer_multipolygon_unknown_role(relation);
            }
            if (member.role()[0] == '\0') {
                ++m_stats.multipolygon_empty_role;
                m_writer_multipolygon_empty_role(relation);
            }
        }

        const char* area = relation.tags().get_value_by_key("area");
        if (area) {
            ++m_stats.multipolygon_area_tag;
            m_writer_multipolygon_area_tag(relation);
        }

        const char* boundary = relation.tags().get_value_by_key("boundary");
        if (boundary) {
            ++m_stats.multipolygon_boundary_tag;
            m_writer_multipolygon_boundary_tag(relation);
        }
    }

    void relation(const osmium::Relation& relation) {
        if (relation.timestamp() >= m_options.before_time) {
            return;
        }

        if (relation.members().empty()) {
            ++m_stats.no_members;
            m_writer_no_member(relation);
        }

        m_stats.relation_members += relation.members().size();

        if (relation.members().size() >= min_members_of_large_relations) {
            ++m_stats.large_relations;
            m_writer_large_relations(relation);
        }

        const char* type = relation.tags().get_value_by_key("type");
        if (type && !std::strcmp(type, "multipolygon")) {
            multipolygon_relation(relation);
        }
    }

    void close() {
        m_writer_no_member.close();
        m_writer_large_relations.close();
        m_writer_multipolygon_non_way_member.close();
        m_writer_multipolygon_unknown_role.close();
        m_writer_multipolygon_empty_role.close();
        m_writer_multipolygon_area_tag.close();
        m_writer_multipolygon_boundary_tag.close();
    }

    const stats_type stats() const noexcept {
        return m_stats;
    }

}; // class CheckHandler

static void print_help() {
    std::cout << program_name << " [OPTIONS] OSM-FILE OUTPUT-DIR\n\n"
              << "Find relations with problems.\n"
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
        {"age",     required_argument, 0, 'a'},
        {"before",  required_argument, 0, 'b'},
        {"help",          no_argument, 0, 'h'},
        {"quiet",         no_argument, 0, 'q'},
        {0, 0, 0, 0}
    };

    options_type options;

    while (true) {
        const int c = getopt_long(argc, argv, "a:b:hq", long_options, 0);
        if (c == -1) {
            break;
        }

        switch (c) {
            case 'a':
                if (options.before_time != osmium::end_of_time()) {
                    std::cerr << "You can not use both -a,--age and -b,--before together\n";
                    std::exit(2);
                }
                options.before_time = osmium::Timestamp{std::time(0) - std::atoi(optarg) * 60 * 60 * 24};
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

    osmium::io::Reader reader{input_filename, osmium::osm_entity_bits::relation};

    osmium::io::Header header;
    header.set("generator", program_name);

    LastTimestampHandler last_timestamp_handler;
    CheckHandler handler{output_dirname, options, header};

    vout << "Reading relations and checking for problems...\n";
    osmium::ProgressBar progress_bar{reader.file_size(), display_progress()};
    while (osmium::memory::Buffer buffer = reader.read()) {
        progress_bar.update(reader.offset());
        osmium::apply(buffer, last_timestamp_handler, handler);
    }
    progress_bar.done();

    handler.close();
    reader.close();

    vout << "Writing out stats...\n";
    const auto last_time{last_timestamp_handler.get_timestamp()};
    write_stats(output_dirname + "/stats-relation-problems.db", last_time, [&](std::function<void(const char*, uint64_t)>& add){
        add("relation_members", handler.stats().relation_members);
        add("no_members", handler.stats().no_members);
        add("large_relations", handler.stats().large_relations);
        add("multipolygon_node_member", handler.stats().multipolygon_node_member);
        add("multipolygon_relation_member", handler.stats().multipolygon_relation_member);
        add("multipolygon_unknown_role", handler.stats().multipolygon_unknown_role);
        add("multipolygon_empty_role", handler.stats().multipolygon_empty_role);
        add("multipolygon_area_tag", handler.stats().multipolygon_area_tag);
        add("multipolygon_boundary_tag", handler.stats().multipolygon_boundary_tag);
    });

    osmium::MemoryUsage memory_usage;
    if (memory_usage.peak()) {
        vout << "Peak memory usage: " << memory_usage.peak() << " MBytes\n";
    }

    vout << "Done with " << program_name << ".\n";

    return 0;
}

