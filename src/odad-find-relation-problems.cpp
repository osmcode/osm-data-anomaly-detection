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
};

class Output {

    using id_map_type = std::vector<std::pair<osmium::unsigned_object_id_type, osmium::unsigned_object_id_type>>;

    std::string m_name;
    std::map<osmium::unsigned_object_id_type, std::vector<osmium::unsigned_object_id_type>> m_marks;
    osmium::geom::OGRFactory<>& m_factory;
    std::unique_ptr<gdalcpp::Layer> m_layer_points;
    std::unique_ptr<gdalcpp::Layer> m_layer_lines;

    osmium::io::File m_file;
    osmium::io::Writer m_writer_rel;
    osmium::io::Writer m_writer_all;

    uint64_t m_counter;

    osmium::nwr_array<id_map_type> m_id_maps;

    static std::string underscore_to_dash(const std::string& str) {
        std::string out;

        for (const char c : str) {
            out += (c == '_') ? '-' : c;
        }

        return out;
    }

    bool check_mark(osmium::unsigned_object_id_type rel_id, osmium::unsigned_object_id_type obj_id) {
        if (m_marks.count(rel_id) == 0) {
            return false;
        }
        const auto& vec = m_marks[rel_id];
        const auto range = std::equal_range(vec.begin(), vec.end(), obj_id);
        return range.first != range.second;
    }

    void add_layers(const osmium::OSMObject& object, const std::pair<id_map_type::const_iterator, id_map_type::const_iterator>& range) {
        const auto ts = object.timestamp().to_iso();

        for (auto it = range.first; it != range.second; ++it) {
            const auto rel_id = it->second;
            if (object.type() == osmium::item_type::node && m_layer_points) {
                try {
                    gdalcpp::Feature feature{*m_layer_points, m_factory.create_point(static_cast<const osmium::Node&>(object))};
                    feature.set_field("rel_id", static_cast<int32_t>(rel_id));
                    feature.set_field("node_id", static_cast<double>(object.id()));
                    feature.set_field("timestamp", ts.c_str());
                    feature.set_field("mark", 0);
                    feature.add_to_layer();
                } catch (osmium::geometry_error&) {
                    // ignore geometry errors
                }
            } else if (object.type() == osmium::item_type::way && m_layer_lines) {
                try {
                    gdalcpp::Feature feature{*m_layer_lines, m_factory.create_linestring(static_cast<const osmium::Way&>(object))};
                    feature.set_field("rel_id", static_cast<int32_t>(rel_id));
                    feature.set_field("way_id", static_cast<int32_t>(object.id()));
                    feature.set_field("timestamp", ts.c_str());
                    feature.set_field("mark", check_mark(rel_id, object.positive_id()));
                    feature.add_to_layer();
                } catch (osmium::geometry_error&) {
                    // ignore geometry errors
                }
            }
        }
    }

    void add_members_to_index(const osmium::Relation& relation) {
        for (const auto& member : relation.members()) {
            m_id_maps(member.type()).emplace_back(member.positive_ref(), relation.positive_id());
        }
    }

public:

    Output(const std::string& name, gdalcpp::Dataset& dataset, osmium::geom::OGRFactory<>& factory, const std::string& directory, const osmium::io::Header& header, bool points, bool lines) :
        m_name(name),
        m_factory(factory),
        m_layer_points(nullptr),
        m_layer_lines(nullptr),
        m_file(directory + "/" + underscore_to_dash(name) + "-all.osm.pbf", "pbf,locations_on_ways=true"),
        m_writer_rel(directory + "/" + underscore_to_dash(name) + ".osm.pbf", header, osmium::io::overwrite::allow),
        m_writer_all(m_file, header, osmium::io::overwrite::allow),
        m_counter(0),
        m_id_maps() {
        if (points) {
            m_layer_points.reset(new gdalcpp::Layer{dataset, name + "_points", wkbPoint, {"SPATIAL_INDEX=NO"}});
            m_layer_points->add_field("rel_id", OFTInteger, 10);
            m_layer_points->add_field("node_id", OFTReal, 12);
            m_layer_points->add_field("timestamp", OFTString, 20);
            m_layer_points->add_field("mark", OFTInteger, 1);
        }
        if (lines) {
            m_layer_lines.reset(new gdalcpp::Layer{dataset, name + "_lines", wkbLineString, {"SPATIAL_INDEX=NO"}});
            m_layer_lines->add_field("rel_id", OFTInteger, 10);
            m_layer_lines->add_field("way_id", OFTInteger, 10);
            m_layer_lines->add_field("timestamp", OFTString, 20);
            m_layer_lines->add_field("mark", OFTInteger, 1);
        }
    }

    const char* name() const noexcept {
        return m_name.c_str();
    }

    std::int64_t counter() const noexcept {
        return m_counter;
    }

    void add(const osmium::Relation& relation, uint64_t increment = 1, const std::vector<osmium::unsigned_object_id_type>& marks = {}) {
        m_counter += increment;
        m_writer_rel(relation);
        add_members_to_index(relation);
        if (!marks.empty()) {
            m_marks.emplace(relation.positive_id(), marks);
            auto vec = m_marks[relation.positive_id()];
        }
    }

    using id_pair = std::pair<osmium::unsigned_object_id_type, osmium::unsigned_object_id_type>;

    void write_to_all(const osmium::OSMObject& object) {
        const auto& map = m_id_maps(object.type());
        const auto range = std::equal_range(map.begin(), map.end(), std::make_pair(object.positive_id(), 0ul), [](const id_pair& a, const id_pair& b){
            return a.first < b.first;
        });
        if (range.first != range.second) {
            m_writer_all(object);
            add_layers(object, range);
        }
    }

    void prepare() {
        std::sort(m_id_maps(osmium::item_type::node).begin(), m_id_maps(osmium::item_type::node).end());
        std::sort(m_id_maps(osmium::item_type::way).begin(), m_id_maps(osmium::item_type::way).end());
        std::sort(m_id_maps(osmium::item_type::relation).begin(), m_id_maps(osmium::item_type::relation).end());
    }

    void close_writer_rel() {
        m_writer_rel.close();
    }

    void close_writer_all() {
        m_writer_all.close();
    }

}; // class Output

class Outputs {

    std::map<std::string, Output> m_outputs;
    std::string m_dirname;
    osmium::io::Header m_header;
    osmium::geom::OGRFactory<> m_factory;
    gdalcpp::Dataset m_dataset;

public:

    Outputs(const std::string& dirname, osmium::io::Header& header) :
        m_outputs(),
        m_dirname(dirname),
        m_header(header),
        m_factory(),
        m_dataset("SQLite", dirname + "/geoms-relation-problems.db", gdalcpp::SRS{m_factory.proj_string()}, { "SPATIALITE=TRUE", "INIT_WITH_EPSG=NO" }) {
        CPLSetConfigOption("OGR_SQLITE_SYNCHRONOUS", "OFF");
        m_dataset.enable_auto_transactions();
        m_dataset.exec("PRAGMA journal_mode = OFF;");
    }

    void add(const char* name, bool points = true, bool lines = true) {
        m_outputs.emplace(std::piecewise_construct,
                          std::forward_as_tuple(name),
                          std::forward_as_tuple(name, m_dataset, m_factory, m_dirname, m_header, points, lines));
    }

    Output& operator[](const char* name) {
        return m_outputs.at(name);
    }

    template <typename TFunc>
    void for_all(TFunc&& func) {
        for (auto& out : m_outputs) {
            std::forward<TFunc>(func)(out.second);
        }
    }

}; // class Outputs

struct MPFilter : public osmium::TagsFilter {

    MPFilter() : osmium::TagsFilter(true) {
        add_rule(false, "type");
        add_rule(false, "created_by");
        add_rule(false, "source");
        add_rule(false, "note");
    }

}; // struct MPFilter

class CheckHandler : public osmium::handler::Handler {

    Outputs& m_outputs;
    options_type m_options;
    stats_type m_stats;
    MPFilter m_mp_filter;

    static std::vector<osmium::unsigned_object_id_type> find_duplicate_ways(const osmium::Relation& relation) {
        std::vector<osmium::unsigned_object_id_type> duplicate_ids;

        std::vector<osmium::unsigned_object_id_type> way_ids;
        way_ids.reserve(relation.members().size());
        for (const auto& member : relation.members()) {
            if (member.type() == osmium::item_type::way) {
                way_ids.push_back(member.positive_ref());
            }
        }
        std::sort(way_ids.begin(), way_ids.end());

        auto it = way_ids.begin();
        while (it != way_ids.end()) {
            it = std::adjacent_find(it, way_ids.end());
            if (it != way_ids.end()) {
                duplicate_ids.push_back(*it);
                ++it;
            }
        }

        return duplicate_ids;
    }

    void multipolygon_relation(const osmium::Relation& relation) {
        if (relation.members().empty()) {
            return;
        }

        std::uint64_t node_member = 0;
        std::uint64_t relation_member = 0;
        std::uint64_t unknown_role = 0;
        std::uint64_t empty_role = 0;

        for (const auto& member : relation.members()) {
            switch (member.type()) {
                case osmium::item_type::node:
                    ++node_member;
                    break;
                case osmium::item_type::way:
                    if (member.role()[0] == '\0') {
                        ++empty_role;
                    } else if (std::strcmp(member.role(), "inner") &&
                               std::strcmp(member.role(), "outer")) {
                        ++unknown_role;
                    }
                    break;
                case osmium::item_type::relation:
                    ++relation_member;
                    break;
                default:
                    break;
            }
        }

        if (node_member) {
            m_outputs["multipolygon_node_member"].add(relation, node_member);
        }

        if (relation_member) {
            m_outputs["multipolygon_relation_member"].add(relation, relation_member);
        }

        if (unknown_role) {
            m_outputs["multipolygon_unknown_role"].add(relation, unknown_role);
        }

        if (empty_role) {
            m_outputs["multipolygon_empty_role"].add(relation, empty_role);
        }

        if (relation.members().size() == 1 && relation.members().cbegin()->type() == osmium::item_type::way) {
            m_outputs["multipolygon_single_way"].add(relation);
        }

        const auto duplicates = find_duplicate_ways(relation);
        if (!duplicates.empty()) {
            m_outputs["multipolygon_duplicate_way"].add(relation, 1, duplicates);
        }

        if (relation.tags().size() == 1 || std::none_of(relation.tags().cbegin(), relation.tags().cend(), std::cref(m_mp_filter))) {
            m_outputs["multipolygon_old_style"].add(relation);
            return;
        }

        const char* area = relation.tags().get_value_by_key("area");
        if (area) {
            m_outputs["multipolygon_area_tag"].add(relation);
        }

        const char* boundary = relation.tags().get_value_by_key("boundary");
        if (boundary) {
            if (!std::strcmp(boundary, "administrative")) {
                m_outputs["multipolygon_boundary_administrative_tag"].add(relation);
            } else {
                m_outputs["multipolygon_boundary_other_tag"].add(relation);
            }
        }
    }

    void boundary_relation(const osmium::Relation& relation) {
        if (relation.members().empty()) {
            return;
        }

        uint64_t empty_role = 0;
        for (const auto& member : relation.members()) {
            if (member.role()[0] == '\0') {
                ++empty_role;
            }
        }
        if (empty_role) {
            m_outputs["boundary_empty_role"].add(relation, empty_role);
        }

        const auto duplicates = find_duplicate_ways(relation);
        if (!duplicates.empty()) {
            m_outputs["boundary_duplicate_way"].add(relation, 1, duplicates);
        }

        const char* area = relation.tags().get_value_by_key("area");
        if (area) {
            m_outputs["boundary_area_tag"].add(relation);
        }

        // is boundary:historic or historic:boundary also okay?
        const char* boundary = relation.tags().get_value_by_key("boundary");
        if (!boundary) {
            m_outputs["boundary_no_boundary_tag"].add(relation);
        }
    }

public:

    CheckHandler(Outputs& outputs, const options_type& options) :
        m_outputs(outputs),
        m_options(options),
        m_stats(),
        m_mp_filter() {
    }

    void relation(const osmium::Relation& relation) {
        if (relation.timestamp() >= m_options.before_time) {
            return;
        }

        m_stats.relation_members += relation.members().size();

        if (relation.members().empty()) {
            m_outputs["relation_no_members"].add(relation);
        }

        if (relation.members().size() >= min_members_of_large_relations) {
            m_outputs["relation_large"].add(relation);
        }

        if (relation.tags().empty()) {
            m_outputs["relation_no_tag"].add(relation);
            return;
        }

        const char* type = relation.tags().get_value_by_key("type");
        if (!type) {
            m_outputs["relation_no_type_tag"].add(relation);
            return;
        }

        if (relation.tags().size() == 1) {
            m_outputs["relation_only_type_tag"].add(relation);
        }

        if (!std::strcmp(type, "multipolygon")) {
            multipolygon_relation(relation);
        } else if (!std::strcmp(type, "boundary")) {
            boundary_relation(relation);
        }
    }

    const stats_type stats() const noexcept {
        return m_stats;
    }

    void close() {
        m_outputs.for_all([](Output& output) {
            output.close_writer_rel();
        });
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
    if (options.before_time == osmium::end_of_time()) {
        vout << "  Get all objects independent of change timestamp (change with --age, -a or --before, -b)\n";
    } else {
        vout << "  Get only objects last changed before: " << options.before_time << " (change with --age, -a or --before, -b)\n";
    }

    osmium::io::Header header;
    header.set("generator", program_name);

    Outputs outputs{output_dirname, header};
    outputs.add("relation_no_members", false, false);
    outputs.add("relation_no_tag");
    outputs.add("relation_only_type_tag");
    outputs.add("relation_no_type_tag");
    outputs.add("relation_large");
    outputs.add("multipolygon_node_member", true, false);
    outputs.add("multipolygon_relation_member", false, false);
    outputs.add("multipolygon_unknown_role", false, true);
    outputs.add("multipolygon_empty_role", false, true);
    outputs.add("multipolygon_area_tag", false, true);
    outputs.add("multipolygon_boundary_administrative_tag", false, true);
    outputs.add("multipolygon_boundary_other_tag", false, true);
    outputs.add("multipolygon_old_style", false, false);
    outputs.add("multipolygon_single_way", false, true);
    outputs.add("multipolygon_duplicate_way", false, true);
    outputs.add("boundary_empty_role", false, true);
    outputs.add("boundary_duplicate_way", false, true);
    outputs.add("boundary_area_tag", false, true);
    outputs.add("boundary_no_boundary_tag", false, true);

    osmium::io::Reader reader{input_filename, osmium::osm_entity_bits::relation};

    LastTimestampHandler last_timestamp_handler;
    CheckHandler handler{outputs, options};

    vout << "Reading relations and checking for problems...\n";
    osmium::ProgressBar progress_bar{reader.file_size(), display_progress()};
    while (osmium::memory::Buffer buffer = reader.read()) {
        progress_bar.update(reader.offset());
        osmium::apply(buffer, last_timestamp_handler, handler);
    }
    progress_bar.done();
    reader.close();

    outputs.for_all([&](Output& output){
        output.prepare();
    });

    vout << "Writing out data files...\n";
    write_data_files(input_filename, outputs);

    vout << "Writing out stats...\n";
    const auto last_time{last_timestamp_handler.get_timestamp()};
    write_stats(output_dirname + "/stats-relation-problems.db", last_time, [&](std::function<void(const char*, uint64_t)>& add){
        add("relation_member_count", handler.stats().relation_members);
        outputs.for_all([&](Output& output){
            add(output.name(), output.counter());
        });
    });

    osmium::MemoryUsage memory_usage;
    if (memory_usage.peak()) {
        vout << "Peak memory usage: " << memory_usage.peak() << " MBytes\n";
    }

    vout << "Done with " << program_name << ".\n";

    return 0;
}

