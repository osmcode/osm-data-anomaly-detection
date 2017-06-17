
#include <map>
#include <string>
#include <tuple>
#include <utility>

#include <osmium/io/header.hpp>
#include <osmium/geom/ogr.hpp>
#include <gdalcpp.hpp>

/**
 * This is a collection of Outputs using the same database.
 */
template <typename TOutput>
class Outputs {

    std::map<std::string, TOutput> m_outputs;
    std::string m_dirname;
    osmium::io::Header m_header;
    osmium::geom::OGRFactory<> m_factory;
    gdalcpp::Dataset m_dataset;

public:

    Outputs(const std::string& dirname, const std::string& dbname, osmium::io::Header& header) :
        m_outputs(),
        m_dirname(dirname),
        m_header(header),
        m_factory(),
        m_dataset("SQLite", dirname + "/" + dbname + ".db", gdalcpp::SRS{m_factory.proj_string()}, { "SPATIALITE=TRUE", "INIT_WITH_EPSG=NO" }) {
        CPLSetConfigOption("OGR_SQLITE_SYNCHRONOUS", "OFF");
        m_dataset.enable_auto_transactions();
        m_dataset.exec("PRAGMA journal_mode = OFF;");
    }

    void add_output(const char* name, bool points = true, bool lines = true) {
        m_outputs.emplace(std::piecewise_construct,
                          std::forward_as_tuple(name),
                          std::forward_as_tuple(name, m_dataset, m_factory, m_dirname, m_header, points, lines));
    }

    TOutput& operator[](const char* name) {
        return m_outputs.at(name);
    }

    template <typename TFunc>
    void for_all(TFunc&& func) {
        for (auto& out : m_outputs) {
            std::forward<TFunc>(func)(out.second);
        }
    }

}; // class Outputs

