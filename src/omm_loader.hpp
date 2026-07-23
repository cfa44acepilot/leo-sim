/*****************************************************************************
  filename omm_loader.hpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-22
  Brief Description:
    Interface for the portable OMM JSON loader. Declares LoadReport plus the
    load/parse entry points and the epoch<->Julian-Date conversions. File I/O
    only, no networking or platform headers, so the simulator reads the local
    cached JSON identically on every platform.
 *****************************************************************************/
// omm_loader.hpp
//
// Portable loader for the cached CelesTrak OMM JSON snapshot. It does file I/O
// only -- no networking, no platform headers -- so it compiles and runs the
// same on Windows and Linux. fetch_starlink.cpp (the optional dev tool that
// downloads the snapshot) is deliberately NOT part of this path; the simulator
// only ever reads the local data/*.json on disk.

#ifndef LEO_OMM_LOADER_HPP
#define LEO_OMM_LOADER_HPP

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "constellation.hpp"

namespace leo {

// Outcome of a load: how many records became satellites, how many were
// rejected, and a human-readable reason for each rejection (and for a
// file/parse-level failure). Counts let a caller assert on a known fixture.
struct LoadReport {
  std::size_t inserted = 0;
  std::size_t skipped = 0;
  std::vector<std::string> errors;
};

// Parse an ISO-8601 UTC timestamp "YYYY-MM-DDTHH:MM:SS[.ffffff]" (the CelesTrak
// EPOCH form: no trailing 'Z', optional microseconds) to a Julian Date. Throws
// std::runtime_error on malformed input so a bad epoch skips its whole record
// rather than silently yielding a wrong time. iso8601_to_jd("2000-01-01T12:00:00")
// == 2451545.0 (J2000).
double iso8601_to_jd(std::string_view iso);

// Inverse of iso8601_to_jd: format a Julian Date as "YYYY-MM-DDTHH:MM:SS.ffffff"
// (UTC) for human-readable snapshot/HUD labels. Standard JD -> Gregorian.
std::string jd_to_iso8601(double jd);

// Read `file` as a JSON array of OMM records and insert each valid one into
// `out` via add_satellite. A record is inserted only if every mapped field is
// present and parses; otherwise the whole record is skipped (a missing field
// must never become a silent zero that puts a satellite in the wrong shell).
// Duplicates are skipped too. Sets no grid_coord / neighbors / positions --
// that is a later pass. verify() is asserted before returning in debug builds.
LoadReport load_omm_json(const std::filesystem::path& file, Constellation& out);

}  // namespace leo

#endif  // LEO_OMM_LOADER_HPP
