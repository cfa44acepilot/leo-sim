/*****************************************************************************
  filename ground_stations.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-22
  Brief Description:
    Implements the ground-station JSON parse. Reads an array of
    {name, lat, lon, [alt]} objects, validates each (dropping malformed or
    duplicate-named entries), and returns the loaded stations plus a
    StationLoadReport of counts and per-error reasons.
 *****************************************************************************/
// ground_stations.cpp -- parse and validate the ground-station catalog.

#include "ground_stations.hpp"

#include <fstream>
#include <unordered_set>

#include <nlohmann/json.hpp>

namespace leo {

StationLoadReport load_ground_stations(const std::filesystem::path& file,
                                       std::vector<GroundStation>& out) {
  using json = nlohmann::json;
  StationLoadReport report;

  std::ifstream in(file, std::ios::binary);
  if (!in) {
    report.errors.push_back("cannot open file: " + file.string());
    return report;
  }

  json doc;
  try {
    in >> doc;
  } catch (const std::exception& e) {
    report.errors.push_back(std::string("JSON parse error: ") + e.what());
    return report;
  }
  if (!doc.is_array()) {
    report.errors.push_back("top-level JSON is not an array");
    return report;
  }

  // Names must be unique across the whole catalog, including anything already in
  // `out` from a previous call.
  std::unordered_set<std::string> seen;
  for (const GroundStation& g : out) seen.insert(g.name);

  for (std::size_t i = 0; i < doc.size(); ++i) {
    try {
      const json& r = doc[i];
      GroundStation g;
      g.name = r.at("name").get<std::string>();
      g.lat_deg = r.at("lat").get<double>();
      g.lon_deg = r.at("lon").get<double>();
      g.alt_km = r.value("alt", 0.0);  // altitude is optional

      if (g.lat_deg < -90.0 || g.lat_deg > 90.0) {
        throw std::runtime_error("lat out of range: " + std::to_string(g.lat_deg));
      }
      if (g.lon_deg < -180.0 || g.lon_deg > 180.0) {
        throw std::runtime_error("lon out of range: " + std::to_string(g.lon_deg));
      }
      if (!seen.insert(g.name).second) {
        throw std::runtime_error("duplicate name: " + g.name);
      }

      out.push_back(std::move(g));
      ++report.loaded;
    } catch (const std::exception& e) {
      ++report.skipped;
      report.errors.push_back("record " + std::to_string(i) + ": " + e.what());
    }
  }
  return report;
}

}  // namespace leo
