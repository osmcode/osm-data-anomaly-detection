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

#include <osmium/geom/ogr.hpp>
#include <osmium/index/id_set.hpp>
#include <osmium/index/nwr_array.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/io/any_output.hpp>
#include <osmium/io/file.hpp>
#include <osmium/osm/entity_bits.hpp>
#include <osmium/osm/object.hpp>
#include <osmium/util/memory.hpp>
#include <osmium/util/progress_bar.hpp>
#include <osmium/util/verbose_output.hpp>
#include <osmium/visitor.hpp>
#include <osmium/tags/tags_filter.hpp>

#include <gdalcpp.hpp>

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
    uint64_t no_tag = 0;
    uint64_t only_type_tag = 0;
    uint64_t no_type_tag = 0;
    uint64_t large = 0;
    uint64_t multipolygon_node_member = 0;
    uint64_t multipolygon_relation_member = 0;
    uint64_t multipolygon_unknown_role = 0;
    uint64_t multipolygon_empty_role = 0;
    uint64_t multipolygon_area_tag = 0;
    uint64_t multipolygon_boundary_administrative_tag = 0;
    uint64_t multipolygon_old_style = 0;
    uint64_t multipolygon_single_way = 0;
    uint64_t multipolygon_duplicate_way = 0;
    uint64_t boundary_empty_role = 0;
    uint64_t boundary_duplicate_way = 0;
    uint64_t boundary_area_tag = 0;
    uint64_t boundary_no_boundary_tag = 0;
};

using id_map_type = std::map<osmium::unsigned_object_id_type, osmium::unsigned_object_id_type>;
using id_maps = osmium::nwr_array<id_map_type>;
using id_set_type = osmium::index::IdSetSmall<osmium::unsigned_object_id_type>;

struct indexes_type {
    id_maps no_tag;
    id_set_type no_tag_rels;

    id_maps only_type_tag;
    id_set_type only_type_tag_rels;

    id_maps no_type_tag;
    id_set_type no_type_tag_rels;
};

struct MPFilter : public osmium::TagsFilter {

    MPFilter() : osmium::TagsFilter(true) {
        add_rule(false, "type");
        add_rule(false, "created_by");
        add_rule(false, "source");
        add_rule(false, "note");
    }

}; // struct MPFilter

class CheckHandler : public osmium::handler::Handler {

    options_type m_options;
    indexes_type& m_indexes;
    stats_type m_stats;
    MPFilter m_mp_filter;

    osmium::io::Writer m_writer_no_member;
    osmium::io::Writer m_writer_no_tag;
    osmium::io::Writer m_writer_only_type_tag;
    osmium::io::Writer m_writer_no_type_tag;
    osmium::io::Writer m_writer_large;
    osmium::io::Writer m_writer_multipolygon_non_way_member;
    osmium::io::Writer m_writer_multipolygon_unknown_role;
    osmium::io::Writer m_writer_multipolygon_empty_role;
    osmium::io::Writer m_writer_multipolygon_area_tag;
    osmium::io::Writer m_writer_multipolygon_boundary_administrative_tag;
    osmium::io::Writer m_writer_multipolygon_old_style;
    osmium::io::Writer m_writer_multipolygon_single_way;
    osmium::io::Writer m_writer_multipolygon_duplicate_way;
    osmium::io::Writer m_writer_boundary_empty_role;
    osmium::io::Writer m_writer_boundary_duplicate_way;
    osmium::io::Writer m_writer_boundary_area_tag;
    osmium::io::Writer m_writer_boundary_no_boundary_tag;

    static bool find_duplicate_ways(const osmium::Relation& relation) {
        std::vector<osmium::object_id_type> way_ids;
        way_ids.reserve(relation.members().size());
        for (const auto& member : relation.members()) {
            if (member.type() == osmium::item_type::way) {
                way_ids.push_back(member.ref());
            }
        }
        std::sort(way_ids.begin(), way_ids.end());
        const auto it = std::adjacent_find(way_ids.begin(), way_ids.end());
        return it != way_ids.end();
    }

    static void add_members_to_index(const osmium::Relation& relation, id_maps& maps) {
        for (const auto& member : relation.members()) {
            maps(member.type()).insert(std::make_pair(member.positive_ref(), relation.positive_id()));
        }
    }

    void multipolygon_relation(const osmium::Relation& relation) {
        const auto non_way_member = m_stats.multipolygon_node_member + m_stats.multipolygon_relation_member;
        const auto unknown_role = m_stats.multipolygon_unknown_role;
        const auto empty_role = m_stats.multipolygon_empty_role;

        for (const auto& member : relation.members()) {
            switch (member.type()) {
                case osmium::item_type::node:
                    ++m_stats.multipolygon_node_member;
                    break;
                case osmium::item_type::way:
                    if (member.role()[0] == '\0') {
                        ++m_stats.multipolygon_empty_role;
                    } else if (std::strcmp(member.role(), "inner") &&
                               std::strcmp(member.role(), "outer")) {
                        ++m_stats.multipolygon_unknown_role;
                    }
                    break;
                case osmium::item_type::relation:
                    ++m_stats.multipolygon_relation_member;
                    break;
                default:
                    break;
            }
        }

        if (non_way_member != m_stats.multipolygon_node_member + m_stats.multipolygon_relation_member) {
            m_writer_multipolygon_non_way_member(relation);
        }

        if (unknown_role != m_stats.multipolygon_unknown_role) {
            m_writer_multipolygon_unknown_role(relation);
        }

        if (empty_role != m_stats.multipolygon_empty_role) {
            m_writer_multipolygon_empty_role(relation);
        }

        if (relation.members().size() == 1 && relation.members().cbegin()->type() == osmium::item_type::way) {
            ++m_stats.multipolygon_single_way;
            m_writer_multipolygon_single_way(relation);
        }

        if (find_duplicate_ways(relation)) {
            ++m_stats.multipolygon_duplicate_way;
            m_writer_multipolygon_duplicate_way(relation);
        }

        if (relation.tags().size() == 1 || std::none_of(relation.tags().cbegin(), relation.tags().cend(), std::cref(m_mp_filter))) {
            ++m_stats.multipolygon_old_style;
            m_writer_multipolygon_old_style(relation);
            return;
        }

        const char* area = relation.tags().get_value_by_key("area");
        if (area) {
            ++m_stats.multipolygon_area_tag;
            m_writer_multipolygon_area_tag(relation);
        }

        const char* boundary = relation.tags().get_value_by_key("boundary");
        if (boundary && !std::strcmp(boundary, "administrative")) {
            ++m_stats.multipolygon_boundary_administrative_tag;
            m_writer_multipolygon_boundary_administrative_tag(relation);
        }
    }

    void boundary_relation(const osmium::Relation& relation) {
        uint64_t empty_role = 0;
        for (const auto& member : relation.members()) {
            if (member.role()[0] == '\0') {
                ++empty_role;
            }
        }
        if (empty_role) {
            m_stats.boundary_empty_role += empty_role;
            m_writer_boundary_empty_role(relation);
        }

        if (find_duplicate_ways(relation)) {
            ++m_stats.boundary_duplicate_way;
            m_writer_boundary_duplicate_way(relation);
        }

        const char* area = relation.tags().get_value_by_key("area");
        if (area) {
            ++m_stats.boundary_area_tag;
            m_writer_boundary_area_tag(relation);
        }

        // is boundary:historic or historic:boundary also okay?
        const char* boundary = relation.tags().get_value_by_key("boundary");
        if (!boundary) {
            ++m_stats.boundary_no_boundary_tag;
            m_writer_boundary_no_boundary_tag(relation);
        }
    }

public:

    CheckHandler(const std::string& directory, const options_type& options, indexes_type& indexes, const osmium::io::Header& header) :
        m_options(options),
        m_indexes(indexes),
        m_stats(),
        m_mp_filter(),
        m_writer_no_member(directory + "/relation-no-member.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_no_tag(directory + "/relation-no-tag.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_only_type_tag(directory + "/relation-only-type-tag.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_no_type_tag(directory + "/relation-no-type-tag.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_large(directory + "/relation-large.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_non_way_member(directory + "/relation-multipolygon-non-way-member.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_unknown_role(directory + "/relation-multipolygon-unknown-role.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_empty_role(directory + "/relation-multipolygon-empty-role.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_area_tag(directory + "/relation-multipolygon-area-tag.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_boundary_administrative_tag(directory + "/relation-multipolygon-boundary-administrative-tag.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_old_style(directory + "/relation-multipolygon-old-style.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_single_way(directory + "/relation-multipolygon-single-way.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_multipolygon_duplicate_way(directory + "/relation-multipolygon-duplicate-way.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_boundary_empty_role(directory + "/relation-boundary-empty-role.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_boundary_duplicate_way(directory + "/relation-boundary-duplicate-way.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_boundary_area_tag(directory + "/relation-boundary-area-tag.osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_boundary_no_boundary_tag(directory + "/relation-boundary-no-boundary-tag.osm.pbf", header, osmium::io::overwrite::allow) {
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
            ++m_stats.large;
            m_writer_large(relation);
        }

        if (relation.tags().empty()) {
            ++m_stats.no_tag;
            m_writer_no_tag(relation);
            m_indexes.no_tag_rels.set(relation.positive_id());
            add_members_to_index(relation, m_indexes.no_tag);
            return;
        }

        const char* type = relation.tags().get_value_by_key("type");
        if (!type) {
            ++m_stats.no_type_tag;
            m_writer_no_type_tag(relation);
            m_indexes.no_type_tag_rels.set(relation.positive_id());
            add_members_to_index(relation, m_indexes.no_type_tag);
            return;
        }

        if (relation.tags().size() == 1) {
            ++m_stats.only_type_tag;
            m_writer_only_type_tag(relation);
            m_indexes.only_type_tag_rels.set(relation.positive_id());
            add_members_to_index(relation, m_indexes.only_type_tag);
        }

        if (!std::strcmp(type, "multipolygon")) {
            multipolygon_relation(relation);
        } else if (!std::strcmp(type, "boundary")) {
            boundary_relation(relation);
        }
    }

    void close() {
        m_writer_no_member.close();
        m_writer_no_tag.close();
        m_writer_only_type_tag.close();
        m_writer_no_type_tag.close();
        m_writer_large.close();
        m_writer_multipolygon_non_way_member.close();
        m_writer_multipolygon_unknown_role.close();
        m_writer_multipolygon_empty_role.close();
        m_writer_multipolygon_area_tag.close();
        m_writer_multipolygon_boundary_administrative_tag.close();
        m_writer_multipolygon_old_style.close();
        m_writer_multipolygon_single_way.close();
        m_writer_multipolygon_duplicate_way.close();
        m_writer_boundary_empty_role.close();
        m_writer_boundary_duplicate_way.close();
        m_writer_boundary_area_tag.close();
        m_writer_boundary_no_boundary_tag.close();
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

class Output {

    osmium::geom::OGRFactory<>& m_factory;
    gdalcpp::Layer m_layer_points;
    gdalcpp::Layer m_layer_lines;

    osmium::io::File m_file;
    osmium::io::Writer m_writer;

    void add_layers(const osmium::OSMObject& object, const std::pair<id_map_type::const_iterator, id_map_type::const_iterator>& range) {
        const auto ts = object.timestamp().to_iso();

        for (auto it = range.first; it != range.second; ++it) {
            const auto rel_id = it->second;
            if (object.type() == osmium::item_type::node) {
                try {
                    gdalcpp::Feature feature{m_layer_points, m_factory.create_point(static_cast<const osmium::Node&>(object))};
                    feature.set_field("rel_id", static_cast<int32_t>(rel_id));
                    feature.set_field("node_id", static_cast<double>(object.id()));
                    feature.set_field("timestamp", ts.c_str());
                    feature.add_to_layer();
                } catch (osmium::geometry_error&) {
                    // ignore geometry errors
                }
            } else if (object.type() == osmium::item_type::way) {
                try {
                    gdalcpp::Feature feature{m_layer_lines, m_factory.create_linestring(static_cast<const osmium::Way&>(object))};
                    feature.set_field("rel_id", static_cast<int32_t>(rel_id));
                    feature.set_field("way_id", static_cast<int32_t>(object.id()));
                    feature.set_field("timestamp", ts.c_str());
                    feature.add_to_layer();
                } catch (osmium::geometry_error&) {
                    // ignore geometry errors
                }
            }
        }
    }

public:

    Output(const char* name, gdalcpp::Dataset& dataset, osmium::geom::OGRFactory<>& factory, const std::string& directory, const osmium::io::Header& header) :
        m_factory(factory),
        m_layer_points(dataset, std::string{"relation_"} + name + "_points", wkbPoint, {"SPATIAL_INDEX=NO"}),
        m_layer_lines(dataset, std::string{"relation_"} + name + "_lines", wkbLineString, {"SPATIAL_INDEX=NO"}),
        m_file(directory + "/relation_" + name + "_all.osm.pbf", "pbf,locations_on_ways=true"),
        m_writer(m_file, header, osmium::io::overwrite::allow) {

        m_layer_points.add_field("rel_id", OFTInteger, 10);
        m_layer_points.add_field("node_id", OFTReal, 12);
        m_layer_points.add_field("timestamp", OFTString, 20);

        m_layer_lines.add_field("rel_id", OFTInteger, 10);
        m_layer_lines.add_field("way_id", OFTInteger, 10);
        m_layer_lines.add_field("timestamp", OFTString, 20);
    }

    void operator()(const osmium::OSMObject& object, const id_maps& index) {
        const auto range = index(object.type()).equal_range(object.positive_id());
        if (range.first != range.second) {
            m_writer(object);
            add_layers(object, range);
        }
    }

    void close() {
        m_writer.close();
    }

}; // class Output

static void write_results(const std::string& input_filename, const std::string& directory, const indexes_type& indexes, const osmium::io::Header& header) {
    osmium::geom::OGRFactory<> factory;
    gdalcpp::Dataset dataset{"SQLite", directory + "/geoms-relation-problems.db", gdalcpp::SRS{factory.proj_string()}, { "SPATIALITE=TRUE", "INIT_WITH_EPSG=NO" }};
    CPLSetConfigOption("OGR_SQLITE_SYNCHRONOUS", "OFF");
    dataset.enable_auto_transactions();
    dataset.exec("PRAGMA journal_mode = OFF;");

    Output out_no_tag{"no_tag", dataset, factory, directory, header};
    Output out_only_type_tag{"only_type_tag", dataset, factory, directory, header};
    Output out_no_type_tag{"no_type_tag", dataset, factory, directory, header};

    osmium::io::Reader reader{input_filename};
    osmium::ProgressBar progress_bar{reader.file_size(), display_progress()};

    while (osmium::memory::Buffer buffer = reader.read()) {
        progress_bar.update(reader.offset());
        for (const auto& object : buffer.select<osmium::OSMObject>()) {
            out_no_tag(object, indexes.no_tag);
            out_only_type_tag(object, indexes.only_type_tag);
            out_no_type_tag(object, indexes.no_type_tag);
        }
    }

    progress_bar.done();
    reader.close();

    out_no_tag.close();
    out_only_type_tag.close();
    out_no_type_tag.close();
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

    indexes_type indexes;
    LastTimestampHandler last_timestamp_handler;
    CheckHandler handler{output_dirname, options, indexes, header};

    vout << "Reading relations and checking for problems...\n";
    osmium::ProgressBar progress_bar{reader.file_size(), display_progress()};
    while (osmium::memory::Buffer buffer = reader.read()) {
        progress_bar.update(reader.offset());
        osmium::apply(buffer, last_timestamp_handler, handler);
    }
    progress_bar.done();
    handler.close();
    reader.close();

    vout << "Writing out data files...\n";
    write_results(input_filename, output_dirname, indexes, header);

    vout << "Writing out stats...\n";
    const auto last_time{last_timestamp_handler.get_timestamp()};
    write_stats(output_dirname + "/stats-relation-problems.db", last_time, [&](std::function<void(const char*, uint64_t)>& add){
        add("relation_member_count", handler.stats().relation_members);
        add("relation_no_members", handler.stats().no_members);
        add("relation_no_tag", handler.stats().no_tag);
        add("relation_only_type_tag", handler.stats().only_type_tag);
        add("relation_no_type_tag", handler.stats().no_type_tag);
        add("relation_large", handler.stats().large);
        add("multipolygon_node_member", handler.stats().multipolygon_node_member);
        add("multipolygon_relation_member", handler.stats().multipolygon_relation_member);
        add("multipolygon_unknown_role", handler.stats().multipolygon_unknown_role);
        add("multipolygon_empty_role", handler.stats().multipolygon_empty_role);
        add("multipolygon_area_tag", handler.stats().multipolygon_area_tag);
        add("multipolygon_boundary_administrative_tag", handler.stats().multipolygon_boundary_administrative_tag);
        add("multipolygon_old_style", handler.stats().multipolygon_old_style);
        add("multipolygon_single_way", handler.stats().multipolygon_single_way);
        add("multipolygon_duplicate_way", handler.stats().multipolygon_duplicate_way);
        add("boundary_empty_role", handler.stats().boundary_empty_role);
        add("boundary_duplicate_way", handler.stats().boundary_duplicate_way);
        add("boundary_area_tag", handler.stats().boundary_area_tag);
        add("boundary_no_boundary_tag", handler.stats().boundary_no_boundary_tag);
    });

    osmium::MemoryUsage memory_usage;
    if (memory_usage.peak()) {
        vout << "Peak memory usage: " << memory_usage.peak() << " MBytes\n";
    }

    vout << "Done with " << program_name << ".\n";

    return 0;
}

