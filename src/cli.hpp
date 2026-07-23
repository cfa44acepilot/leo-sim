/*****************************************************************************
  filename cli.hpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-22
  Brief Description:
    Declares the testable glue behind the `leo` command-line tool: CliOptions,
    argument parsing, endpoint resolution (station name vs. "lat,lon"), and the
    data-age helper. Factored out of main.cpp so this logic can be unit-tested
    without spawning a process.
 *****************************************************************************/
// cli.hpp
//
// Argument parsing and endpoint resolution for the `leo` command-line tool.
// Factored out of main.cpp so the glue logic (lat,lon parsing, name vs.
// coordinate resolution) is unit-testable without spawning a process.

#ifndef LEO_CLI_HPP
#define LEO_CLI_HPP

#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "simulator.hpp"

namespace leo {

// Parsed command line. Endpoints stay as raw strings here -- they are resolved
// (name vs. lat,lon) later, against a loaded Simulator.
struct CliOptions {
  std::string src;
  std::string dst;
  std::string time_iso;  // empty => use now_jd_utc()
  double hop_cost_s = 0.0;
  std::string omm = "data/starlink.json";
  std::string stations = "data/ground_stations.json";
  bool help = false;
  bool diag = false;  // dump per-hop distances + uplink slant/elevation
};

// Parse "lat,lon" into a (lat, lon) pair. Returns nullopt if the string is not
// exactly two comma-separated numbers -- the caller then treats it as a station
// name. This is the detection rule the front end relies on.
std::optional<std::pair<double, double>> parse_lat_lon(std::string_view s);

// Parse argv into opts. Returns false (with err set) on a usage error. Sets
// opts.help and returns true for -h/--help so the caller can print usage.
bool parse_args(int argc, char** argv, CliOptions& opts, std::string& err);

// Age in days of the NEWEST element epoch in the OMM file at `omm`, measured
// against the current UTC time. Returns a negative value if the file is
// missing, unreadable, or has no usable satellites (so the caller can treat
// "can't tell" as "needs a fetch"). Portable -- no networking, just a parse.
double omm_data_age_days(const std::filesystem::path& omm);

// Resolve src/dst (each a station name or a "lat,lon") and place them on sim,
// which must already be load()ed (so stations() is populated). Returns false
// with err set if a name is not in the catalog; sim state is left unchanged in
// that case (the resolution that fails happens before any placement).
bool resolve_endpoints(Simulator& sim, const std::string& src,
                       const std::string& dst, std::string& err);

}  // namespace leo

#endif  // LEO_CLI_HPP
