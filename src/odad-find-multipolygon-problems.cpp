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

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>

#include <osmium/index/id_set.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/io/file.hpp>
#include <osmium/osm/entity_bits.hpp>
#include <osmium/osm/object.hpp>
#include <osmium/relations/manager_util.hpp>
#include <osmium/relations/relations_manager.hpp>
#include <osmium/tags/taglist.hpp>
#include <osmium/tags/tags_filter.hpp>
#include <osmium/util/file.hpp>
#include <osmium/util/memory.hpp>
#include <osmium/util/progress_bar.hpp>
#include <osmium/util/verbose_output.hpp>
#include <osmium/visitor.hpp>

#include "outputs.hpp"
#include "utils.hpp"

static const char* program_name = "odad-find-multipolygon-problems";

struct options_type {
    bool verbose = true;
};

struct stats_type {
    uint64_t multipolygon_relations = 0;
    uint64_t multipolygon_relations_without_tags = 0;
    uint64_t multipolygon_relation_members = 0;
    uint64_t multipolygon_relation_way_members = 0;
    uint64_t multipolygon_relation_members_with_same_tags = 0;
    uint64_t multipolygon_relation_members_with_conflicting_tags = 0;
};

struct MPFilter : public osmium::TagsFilter {

    MPFilter() : osmium::TagsFilter(true) {
        add_rule(false, "type");
        add_rule(false, "created_by");
        add_rule(false, "source");
        add_rule(false, "note");
    }

}; // struct MPFilter

struct KeyCompare {
    bool operator()(const osmium::Tag& lhs, const osmium::Tag& rhs) const noexcept {
        return !strcmp(lhs.key(), rhs.key());
    }
}; // struct KeyCompare

class CheckMPManager : public osmium::relations::RelationsManager<CheckMPManager, true, true, true> {

    Outputs& m_outputs;
    options_type m_options;
    stats_type m_stats;
    MPFilter m_filter;
    KeyCompare m_key_compare;

    bool compare_tags(const osmium::TagList& rtags, const osmium::TagList& wtags) const noexcept {
        const auto d = std::count_if(wtags.cbegin(), wtags.cend(), std::cref(m_filter));
        if (d > 0) {
            using iterator = boost::filter_iterator<MPFilter, osmium::TagList::const_iterator>;
            iterator rfi_begin{std::cref(m_filter), rtags.cbegin(), rtags.cend()};
            iterator rfi_end{std::cref(m_filter), rtags.cend(), rtags.cend()};
            iterator wfi_begin{std::cref(m_filter), wtags.cbegin(), wtags.cend()};
            iterator wfi_end{std::cref(m_filter), wtags.cend(), wtags.cend()};

            if (std::equal(wfi_begin, wfi_end, rfi_begin) && d == std::distance(rfi_begin, rfi_end)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Check if a way and a relation have identical keys.
     */
    bool conflicting_tags(const osmium::TagList& rtags, const osmium::TagList& wtags) const noexcept {
        const auto d = std::count_if(wtags.cbegin(), wtags.cend(), std::cref(m_filter));
        if (d > 0) {
            using iterator = boost::filter_iterator<MPFilter, osmium::TagList::const_iterator>;
            iterator rfi_begin{std::cref(m_filter), rtags.cbegin(), rtags.cend()};
            iterator rfi_end{std::cref(m_filter), rtags.cend(), rtags.cend()};
            iterator wfi_begin{std::cref(m_filter), wtags.cbegin(), wtags.cend()};
            iterator wfi_end{std::cref(m_filter), wtags.cend(), wtags.cend()};

            if (std::equal(wfi_begin, wfi_end, rfi_begin, m_key_compare) && d == std::distance(rfi_begin, rfi_end)) {
                return true;
            }
        }
        return false;
    }

public:

    CheckMPManager(Outputs& outputs, const options_type& options) :
        m_outputs(outputs),
        m_options(options),
        m_stats(),
        m_filter() {
    }

    const stats_type& stats() const noexcept {
        return m_stats;
    }

    bool new_relation(const osmium::Relation& relation) noexcept {
        if (relation.tags().has_tag("type", "multipolygon")) {
            ++m_stats.multipolygon_relations;
            return true;
        }
        return false;
    }

    bool new_member(const osmium::Relation& /*relation*/, const osmium::RelationMember& member, std::size_t /*n*/) noexcept {
        ++m_stats.multipolygon_relation_members;
        if (member.type() == osmium::item_type::way) {
            ++m_stats.multipolygon_relation_way_members;
            return true;
        }
        return false;
    }

    void complete_relation(const osmium::Relation& relation) {
        if (osmium::tags::match_none_of(relation.tags(), m_filter)) {
            ++m_stats.multipolygon_relations_without_tags;
            return;
        }

        std::vector<osmium::unsigned_object_id_type> marks;
        bool same_tags = false;
        bool same_keys = false;

        for (const auto& member : relation.members()) {
            if (member.type() == osmium::item_type::way) {
                const auto* way = this->get_member_way(member.ref());
                if (compare_tags(relation.tags(), way->tags())) {
                    ++m_stats.multipolygon_relation_members_with_same_tags;
                    marks.push_back(way->positive_id());
                    same_tags = true;
                } else if (way->is_closed() && strcmp(member.role(), "inner") && conflicting_tags(relation.tags(), way->tags())) {
                    // Check only outer rings. Otherwise inner rings of a forest multipolygon relation
                    // are flagged as errors because the inner ring is a meadow.
                    ++m_stats.multipolygon_relation_members_with_conflicting_tags;
                    marks.push_back(way->positive_id());
                    same_keys = true;
                }
            }
        }

        if (!marks.empty() && same_tags) {
            m_outputs["multipolygon_relations_with_same_tags"].add(relation, 1, marks);
        }
        if (!marks.empty() && same_keys) {
            m_outputs["multipolygon_relations_with_same_keys"].add(relation, 1, marks);
        }
    }

}; // CheckMPManager

static void print_help() {
    std::cout << program_name << " [OPTIONS] OSM-FILE OUTPUT-DIR\n\n"
              << "Find multipolygons with problems.\n"
              << "\nOptions:\n"
              << "  -h, --help              This help message\n"
              << "  -q, --quiet             Work quietly\n"
              ;
}

static options_type parse_command_line(int argc, char* argv[]) {
    static struct option long_options[] = {
        {"help",  no_argument, 0, 'h'},
        {"quiet", no_argument, 0, 'q'},
        {0, 0, 0, 0}
    };

    options_type options;

    while (true) {
        const int c = getopt_long(argc, argv, "hq", long_options, 0);
        if (c == -1) {
            break;
        }

        switch (c) {
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

static void write_data_files(const std::string& input_filename, Outputs& outputs) {
    osmium::io::Reader reader{input_filename};
    osmium::ProgressBar progress_bar{reader.file_size(), display_progress()};

    while (osmium::memory::Buffer buffer = reader.read()) {
        progress_bar.update(reader.offset());
        for (const auto& object : buffer.select<osmium::OSMObject>()) {
            outputs.for_all([&](Output& output) {
                output.write_to_all(object);
            });
        }
    }

    progress_bar.done();
    reader.close();

    outputs.for_all([](Output& output) {
        output.close_writer_all();
    });
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

    osmium::io::Header header;
    header.set("generator", program_name);

    Outputs outputs{output_dirname, "geoms-multipolygon-problems", header};
    outputs.add_output("multipolygon_relations_with_same_tags", false, true);
    outputs.add_output("multipolygon_relations_with_same_keys", false, true);

    LastTimestampHandler last_timestamp_handler;

    vout << "Reading relations and checking for problems...\n";
    const auto file_size = osmium::util::file_size(input_filename);
    osmium::ProgressBar progress_bar{file_size * 2, display_progress()};

    CheckMPManager manager{outputs, options};

    osmium::io::File file{input_filename};
    osmium::relations::read_relations(file, manager);

    vout << "Reading ways and checking for problems...\n";
    osmium::io::Reader reader{file, osmium::osm_entity_bits::way};
    if (file.format() == osmium::io::file_format::pbf && !has_locations_on_ways(reader.header())) {
        std::cerr << "Input file must have locations on ways.\n";
        return 2;
    }

    while (osmium::memory::Buffer buffer = reader.read()) {
        progress_bar.update(reader.offset());
        osmium::apply(buffer, last_timestamp_handler, manager.handler());
    }
    progress_bar.file_done(file_size);
    progress_bar.done();
    reader.close();

    outputs.for_all([&](Output& output){
        output.close_writer_rel(); // XXX
        output.prepare();
    });

    vout << "Writing out data files...\n";
    write_data_files(input_filename, outputs);

    vout << "Writing out stats...\n";
    const auto last_time{last_timestamp_handler.get_timestamp()};
    write_stats(output_dirname + "/stats-multipolygon-problems.db", last_time, [&](std::function<void(const char*, uint64_t)>& add_stat){
        add_stat("multipolygon_relations",                       manager.stats().multipolygon_relations);
        add_stat("multipolygon_relations_without_tags",          manager.stats().multipolygon_relations_without_tags);
        add_stat("multipolygon_relation_members",                manager.stats().multipolygon_relation_members);
        add_stat("multipolygon_relation_way_members",            manager.stats().multipolygon_relation_way_members);
        add_stat("multipolygon_relation_members_with_same_tags", manager.stats().multipolygon_relation_members_with_same_tags);
        add_stat("multipolygon_relation_members_with_same_keys", manager.stats().multipolygon_relation_members_with_conflicting_tags);
        outputs.for_all([&](Output& output){
            add_stat(output.name(), output.counter());
        });
    });

    osmium::MemoryUsage memory_usage;
    if (memory_usage.peak()) {
        vout << "Peak memory usage: " << memory_usage.peak() << " MBytes\n";
    }

    vout << "Done with " << program_name << ".\n";

    return 0;
}

