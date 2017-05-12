#ifndef UTILS_HPP
#define UTILS_HPP

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
#include <string>

#include <osmium/geom/ogr.hpp>
#include <osmium/handler.hpp>
#include <osmium/io/header.hpp>
#include <osmium/util/file.hpp>

#include <gdalcpp.hpp>
#include <sqlite.hpp>

bool display_progress() noexcept {
    return osmium::util::isatty(2);
}

class HandlerWithDB : public osmium::handler::Handler {

protected:

    osmium::geom::OGRFactory<> m_factory;
    gdalcpp::Dataset m_dataset;

public:

    explicit HandlerWithDB(const std::string& name) :
        m_factory(),
        m_dataset("SQLite", name, gdalcpp::SRS{m_factory.proj_string()}, { "SPATIALITE=TRUE", "INIT_WITH_EPSG=NO" }) {
        CPLSetConfigOption("OGR_SQLITE_SYNCHRONOUS", "OFF");
        m_dataset.enable_auto_transactions();
        m_dataset.exec("PRAGMA journal_mode = OFF;");
    }

}; // Class HandlerWithDB

class LastTimestampHandler : public osmium::handler::Handler {

    osmium::Timestamp m_timestamp;

public:

    LastTimestampHandler() :
        m_timestamp(osmium::start_of_time()) {
    }

    void osm_object(const osmium::OSMObject& object) noexcept {
        if (object.timestamp() > m_timestamp) {
            m_timestamp = object.timestamp();
        }
    }

    osmium::Timestamp get_timestamp() const noexcept {
        return m_timestamp;
    }

}; // class LastTimestampHandler

template <typename TFunc>
void write_stats(const std::string& database_name, const osmium::Timestamp& timestamp, TFunc&& func) {
    Sqlite::Database db{database_name, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE};

    db.exec("CREATE TABLE IF NOT EXISTS stats (date TEXT, key TEXT, value INT64 DEFAULT 0);");

    Sqlite::Statement statement{db, "INSERT INTO stats (date, key, value) VALUES (?, ?, ?);"};

    const std::string last_time{timestamp.to_iso()};

    std::function<void(const char*, uint64_t)> add = [&](const char* name, uint64_t value) {
        statement.bind_text(last_time)
                 .bind_text(name)
                 .bind_int64(value)
                 .execute();
    };

    std::forward<TFunc>(func)(add);
}

inline bool has_locations_on_ways(const osmium::io::Header& header) {
    for (const auto& option : header) {
        if (option.second == "LocationsOnWays") {
            return true;
        }
    }

    return false;
}

#endif // UTILS_HPP
