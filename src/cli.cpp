/*****************************************************************************
  filename cli.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-22
  Brief Description:
    Implements the `leo` command-line glue: parses argv into CliOptions,
    detects "lat,lon" versus a station name, resolves both endpoints against a
    loaded Simulator, and provides omm_data_age_days (used by `leo update` to
    decide whether the cached snapshot needs a re-pull).
 *****************************************************************************/
// cli.cpp -- argument parsing and endpoint resolution for the `leo` tool.

#include "cli.hpp"

#include <cstdlib>
#include <string>
#include <vector>

#include "constellation.hpp"
#include "epoch_health.hpp"  // check_epoch_health, now_jd_utc
#include "omm_loader.hpp"    // load_omm_json

namespace leo {

namespace {

// Trim ASCII whitespace from both ends of a view.
std::string_view trim(std::string_view s) {
  const auto is_ws = [](char ch) {
    return ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n';
  };
  while (!s.empty() && is_ws(s.front())) s.remove_prefix(1);
  while (!s.empty() && is_ws(s.back())) s.remove_suffix(1);
  return s;
}

// Parse a single trimmed token as a complete double (no trailing junk).
bool parse_double(std::string_view tok, double& out) {
  const std::string s(trim(tok));
  if (s.empty()) return false;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end != s.c_str() + s.size()) return false;  // unconsumed -> not a number
  out = v;
  return true;
}

// Resolve one endpoint token to coordinates: a "lat,lon" parses directly, else
// it is a station name looked up in the catalog. was_name records which branch
// took so the caller can honor the name-vs-geo placement distinction.
bool resolve_one(const Simulator& sim, const std::string& tok, double& lat,
                 double& lon, bool& was_name, std::string& err) {
  if (const auto ll = parse_lat_lon(tok)) {
    lat = ll->first;
    lon = ll->second;
    was_name = false;
    return true;
  }
  was_name = true;
  for (const GroundStation& st : sim.stations()) {
    if (st.name == tok) {
      lat = st.lat_deg;
      lon = st.lon_deg;
      return true;
    }
  }
  err = "unknown station name: " + tok;
  return false;
}

}  // namespace

double omm_data_age_days(const std::filesystem::path& omm) {
  // Parse into a throwaway constellation; a missing file or empty feed yields
  // zero satellites, which we report as "unknown" (negative) so the caller
  // refreshes rather than trusting nothing.
  Constellation c;
  load_omm_json(omm, c);
  if (c.satellite_count() == 0) return -1.0;
  return check_epoch_health(c, now_jd_utc()).newest_age_days;
}

std::optional<std::pair<double, double>> parse_lat_lon(std::string_view s) {
  const std::size_t comma = s.find(',');
  if (comma == std::string_view::npos) return std::nullopt;
  // Exactly one comma: a second one means it is not a simple coordinate pair.
  if (s.find(',', comma + 1) != std::string_view::npos) return std::nullopt;

  double lat = 0.0, lon = 0.0;
  if (!parse_double(s.substr(0, comma), lat)) return std::nullopt;
  if (!parse_double(s.substr(comma + 1), lon)) return std::nullopt;
  return std::make_pair(lat, lon);
}

bool parse_args(int argc, char** argv, CliOptions& opts, std::string& err) {
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      opts.help = true;
      return true;
    }
    // Flags that take a value consume the next argument.
    const auto take_value = [&](const char* name, std::string& dst) -> bool {
      if (i + 1 >= argc) {
        err = std::string("missing value for ") + name;
        return false;
      }
      dst = argv[++i];
      return true;
    };
    if (a == "--time") {
      if (!take_value("--time", opts.time_iso)) return false;
    } else if (a == "--omm") {
      if (!take_value("--omm", opts.omm)) return false;
    } else if (a == "--stations") {
      if (!take_value("--stations", opts.stations)) return false;
    } else if (a == "--hop-cost") {
      std::string v;
      if (!take_value("--hop-cost", v)) return false;
      double hop = 0.0;
      if (!parse_double(v, hop) || hop < 0.0) {
        err = "invalid --hop-cost: " + v;
        return false;
      }
      opts.hop_cost_s = hop;
    } else if (a == "--diag") {
      opts.diag = true;
    } else if (!a.empty() && a[0] == '-') {
      err = "unknown option: " + a;
      return false;
    } else {
      positional.push_back(a);
    }
  }

  if (positional.size() != 2) {
    err = "expected <src> and <dst> (got " +
          std::to_string(positional.size()) + ")";
    return false;
  }
  opts.src = positional[0];
  opts.dst = positional[1];
  return true;
}

bool resolve_endpoints(Simulator& sim, const std::string& src,
                       const std::string& dst, std::string& err) {
  double slat = 0.0, slon = 0.0, dlat = 0.0, dlon = 0.0;
  bool src_name = false, dst_name = false;
  // Validate BOTH endpoints before placing either, so an unknown name leaves
  // the simulator untouched.
  if (!resolve_one(sim, src, slat, slon, src_name, err)) return false;
  if (!resolve_one(sim, dst, dlat, dlon, dst_name, err)) return false;

  // Both are catalog names -> use the name API (already validated above, so it
  // cannot fail here). Any coordinate involved -> place by geodetic.
  if (src_name && dst_name) return sim.set_endpoints(src, dst);
  return sim.set_endpoints_geo(slat, slon, dlat, dlon);
}

}  // namespace leo
