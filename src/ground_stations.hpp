// ground_stations.hpp
//
// Ground-station catalog loader. Reads a JSON array of named geodetic sites
// (the source/dest endpoints a query can pick from). Portable, file-only, and
// validating, mirroring omm_loader's LoadReport pattern.

#ifndef LEO_GROUND_STATIONS_HPP
#define LEO_GROUND_STATIONS_HPP

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace leo {

struct GroundStation {
  std::string name;
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_km = 0.0;
};

struct StationLoadReport {
  std::size_t loaded = 0;
  std::size_t skipped = 0;
  std::vector<std::string> errors;
};

// Read `file` (JSON array of {name, lat, lon, [alt]}; alt defaults to 0) and
// append valid stations to `out`. A record is skipped (with a reason) if it is
// malformed, has a duplicate name, or has lat outside [-90,90] / lon outside
// [-180,180]. Uniqueness is checked against names already in `out` too.
StationLoadReport load_ground_stations(const std::filesystem::path& file,
                                       std::vector<GroundStation>& out);

}  // namespace leo

#endif  // LEO_GROUND_STATIONS_HPP
