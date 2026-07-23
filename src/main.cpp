/*****************************************************************************
  filename main.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-22
  Brief Description:
    Entry point for the `leo` command-line front end. A thin translator: parse
    args, drive a Simulator through its tiers, and format the returned Snapshot
    (labels, per-hop connectors, the fiber-latency comparison). Also hosts the
    `leo update` subcommand that shells out to the fetch tool. Computes nothing.
 *****************************************************************************/
// main.cpp -- the `leo` command-line front end.
//
// Thin translator: parse args, drive the Simulator, format the returned
// Snapshot. It computes nothing itself; every printed value comes off the
// Snapshot (the one exception is the fiber ratio, a presentation-only division
// of two Snapshot fields).

#include <cmath>
#include <cstdio>
#include <cstdlib>  // std::system
#include <cstring>  // std::strcmp
#include <filesystem>
#include <numbers>
#include <stdexcept>  // std::exception (--time parse guard)
#include <string>

#include <glm/geometric.hpp>  // glm::distance, length, normalize, dot

#include "cli.hpp"
#include "constellation.hpp"
#include "epoch_health.hpp"  // now_jd_utc
#include "isl_links.hpp"     // LinkSpec (max_range_km reference)
#include "omm_loader.hpp"    // iso8601_to_jd
#include "simulator.hpp"

namespace {

void print_usage() {
  std::puts(
      "usage: leo <src> <dst> [--time <iso8601>] [--hop-cost <seconds>]\n"
      "           [--omm <path>] [--stations <path>]\n"
      "       leo update [--max-age-days <n>] [--force] [--omm <path>]\n"
      "                  [--fetcher <path>]\n"
      "\n"
      "  <src>/<dst>: a station name (\"London\") or a \"lat,lon\" pair\n"
      "               (\"41.88,-87.63\").\n"
      "  --time:      UTC YYYY-MM-DDTHH:MM:SS[.ffffff]; defaults to now.\n"
      "  --hop-cost:  per-hop switching cost in seconds (default 0).\n"
      "  --diag:      dump per-hop distances and uplink slant/elevation.\n"
      "\n"
      "  update: refresh data/starlink.json from CelesTrak, but only if it is\n"
      "          older than --max-age-days (default 1.0); --force ignores the\n"
      "          age check. Networking is delegated to the fetch tool, not the\n"
      "          simulator.");
}

// `leo update`: detect how old the cached snapshot is and, if it is stale (or
// --force), shell out to the standalone fetch tool to download a fresh one.
// std::system keeps all networking inside fetch_starlink.exe -- the portable
// simulator/loader stays network-free, as it has from the start.
int run_update(int argc, char** argv) {
  std::string omm = "data/starlink.json";
  std::string fetcher = "fetch_starlink.exe";
  double max_age_days = 1.0;
  bool force = false;

  // argv[1] is "update"; options follow from index 2.
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    const auto take = [&](std::string& dst) -> bool {
      if (i + 1 >= argc) return false;
      dst = argv[++i];
      return true;
    };
    if (a == "--omm") {
      if (!take(omm)) { std::fprintf(stderr, "error: --omm needs a path\n"); return 2; }
    } else if (a == "--fetcher") {
      if (!take(fetcher)) { std::fprintf(stderr, "error: --fetcher needs a path\n"); return 2; }
    } else if (a == "--max-age-days") {
      std::string v;
      if (!take(v)) { std::fprintf(stderr, "error: --max-age-days needs a value\n"); return 2; }
      max_age_days = std::strtod(v.c_str(), nullptr);
    } else if (a == "--force") {
      force = true;
    } else {
      std::fprintf(stderr, "error: unknown update option: %s\n", a.c_str());
      return 2;
    }
  }

  const double age = leo::omm_data_age_days(omm);
  if (age < 0.0) {
    std::printf("no readable element data at '%s'.\n", omm.c_str());
  } else {
    std::printf("current data is %.1f days old (newest element epoch).\n", age);
  }

  // Negative age (missing/empty) always warrants a fetch.
  const bool stale = force || age < 0.0 || age > max_age_days;
  if (!stale) {
    std::printf("data is fresh (within %.1f days); not updating.\n",
                max_age_days);
    return 0;
  }

  // Resolve a relative fetcher to an absolute path: cmd.exe does not reliably
  // search the current directory, and the absolute path needs quoting because
  // the repo path can contain spaces.
  std::filesystem::path fp = fetcher;
  if (fp.is_relative() && std::filesystem::exists(fp)) {
    fp = std::filesystem::absolute(fp);
  }
  std::printf("fetching a fresh snapshot via '%s' ...\n", fp.string().c_str());
  const std::string cmd = "\"" + fp.string() + "\" fetch starlink";
  const int rc = std::system(cmd.c_str());
  if (rc != 0) {
    std::fprintf(stderr,
                 "update failed (fetch tool exit %d). CelesTrak rate-limits "
                 "repeat requests with HTTP 403 -- wait a bit and retry.\n",
                 rc);
    return 2;
  }

  const double new_age = leo::omm_data_age_days(omm);
  if (new_age >= 0.0) {
    std::printf("updated: data is now %.1f days old.\n", new_age);
  }
  return 0;
}

// Label a path node: the two ground endpoints by their query string, satellites
// by their catalog id (nicer than a bare dense index).
std::string label_node(const leo::Constellation& c, leo::NodeId n,
                       leo::NodeId src, const std::string& src_label,
                       leo::NodeId dst, const std::string& dst_label) {
  // Ground nodes carry a placeholder grid_coord{0,0}; printing "p0,s0" would
  // misleadingly read as a real plane, so show only the query label for them.
  if (c.is_ground(n)) {
    if (n == src) return src_label;
    if (n == dst) return dst_label;
    return "GROUND";
  }
  // Satellite: catalog id plus its stable plane/slot grid coordinate, which
  // makes the in-plane vs cross-plane structure of the path visible.
  const leo::GridCoord g = c.grid_coord[n];
  return "SAT-" + std::to_string(c.catalog_at(n)) + " (p" +
         std::to_string(g.plane) + ",s" + std::to_string(g.slot) + ")";
}

// Classify a satellite-satellite hop by which ISL terminal links them: slots
// 0/1 are in-plane (wired by topology, NOT range-checked), slots 2/3 are
// cross-plane (seeded with a range + line-of-sight filter). Links are
// undirected after the router build, so check both endpoints.
const char* isl_kind(const leo::Constellation& c, leo::NodeId a,
                     leo::NodeId b) {
  const auto check = [](const auto& nb, leo::NodeId other) -> const char* {
    if (nb[0] == other || nb[1] == other) return "in-plane";
    if (nb[2] == other || nb[3] == other) return "cross-plane";
    return nullptr;
  };
  if (const char* k = check(c.isl_neighbors[a], b)) return k;
  if (const char* k = check(c.isl_neighbors[b], a)) return k;
  return "unknown";
}

// Compact arrow for the main route line: ground<->sat is a plain arrow, while a
// satellite<->satellite hop is tagged by its ISL kind so the path's in-plane /
// cross-plane structure reads at a glance.
const char* hop_connector(const leo::Constellation& c, leo::NodeId a,
                          leo::NodeId b) {
  if (c.is_ground(a) || c.is_ground(b)) return " -> ";  // uplink / downlink
  const char* k = isl_kind(c, a, b);
  if (std::strcmp(k, "in-plane") == 0) return " =in-plane=> ";
  if (std::strcmp(k, "cross-plane") == 0) return " =cross=> ";
  return " =isl=> ";  // should not happen given the gating
}

// Elevation angle (deg) of `sat` above the local horizon at ground point `g`,
// using the same geocentric-up approximation connect_ground_uplinks uses.
double elevation_deg(const leo::Vec3& g, const leo::Vec3& sat) {
  const leo::Vec3 up = glm::normalize(g);
  double sin_elev = glm::dot(up, glm::normalize(sat - g));
  if (sin_elev > 1.0) sin_elev = 1.0;
  if (sin_elev < -1.0) sin_elev = -1.0;
  return std::asin(sin_elev) * 180.0 / std::numbers::pi;
}

// --diag: dump the geometry of the computed route so an illegitimately long hop
// is obvious -- per-hop straight-line distances, satellite altitudes, the
// uplink slant ranges/elevations actually available, and any ISL exceeding the
// link range budget. Diagnostic only; changes no behavior.
void print_diag(const leo::Constellation& c, const leo::Snapshot& s,
                const std::string& src_label, const std::string& dst_label) {
  const double max_range = leo::LinkSpec{}.max_range_km;
  const auto name = [&](leo::NodeId n) {
    return label_node(c, n, s.src_node, src_label, s.dst_node, dst_label);
  };

  std::printf("\n=== diagnostics ===\n");

  if (s.found && s.path.size() >= 2) {
    std::printf("path hops (straight-line distance):\n");
    for (std::size_t i = 1; i < s.path.size(); ++i) {
      const leo::NodeId a = s.path[i - 1], b = s.path[i];
      const double d = glm::distance(c.position_ecef[a], c.position_ecef[b]);
      const bool isl = !c.is_ground(a) && !c.is_ground(b);
      char tag[48] = "";
      if (isl) {
        std::snprintf(tag, sizeof(tag), "  <-- ISL (%s)%s", isl_kind(c, a, b),
                      d > max_range ? " EXCEEDS max_range" : "");
      } else {
        std::snprintf(tag, sizeof(tag), "  <-- uplink/downlink");
      }
      std::printf("  %-18s -> %-18s %9.1f km%s\n", name(a).c_str(),
                  name(b).c_str(), d, tag);
    }
    std::printf("satellite positions on path (altitude check):\n");
    for (const leo::NodeId n : s.path) {
      if (c.is_ground(n)) continue;
      const leo::Vec3 p = c.position_ecef[n];
      std::printf("  SAT-%u  |r|=%.1f km  pos=(%.1f, %.1f, %.1f)\n",
                  c.catalog_at(n), glm::length(p), p.x, p.y, p.z);
    }
  }

  // Uplink lists actually available to each endpoint, with slant + elevation.
  const auto dump_uplinks = [&](leo::NodeId gnode, const std::string& label) {
    if (!c.is_ground(gnode)) return;
    const std::size_t g = gnode - c.num_satellites;
    if (g >= c.ground_uplinks.size()) return;
    const leo::Vec3 gp = c.position_ecef[gnode];
    const std::vector<leo::NodeId>& list = c.ground_uplinks[g];
    std::printf("%s uplinks: %zu satellite(s) in view\n", label.c_str(),
                list.size());
    for (const leo::NodeId sn : list) {
      const leo::Vec3 sp = c.position_ecef[sn];
      std::printf("  SAT-%u  slant=%.1f km  elev=%.1f deg\n", c.catalog_at(sn),
                  glm::distance(gp, sp), elevation_deg(gp, sp));
    }
  };
  dump_uplinks(s.src_node, src_label);
  dump_uplinks(s.dst_node, dst_label);
  std::printf("(link budget: max ISL range %.1f km, uplink min elevation only "
              "-- no max slant)\n",
              max_range);
}

}  // namespace

int main(int argc, char** argv) {
  // Subcommand dispatch: `leo update ...` is handled before the route parser so
  // its flags don't have to fit the <src> <dst> shape.
  if (argc > 1 && std::string(argv[1]) == "update") {
    return run_update(argc, argv);
  }

  leo::CliOptions opts;
  std::string err;
  if (!leo::parse_args(argc, argv, opts, err)) {
    std::fprintf(stderr, "error: %s\n\n", err.c_str());
    print_usage();
    return 2;
  }
  if (opts.help) {
    print_usage();
    return 0;
  }

  leo::Simulator sim;
  if (!sim.load(opts.omm, opts.stations)) {
    std::fprintf(stderr,
                 "error: failed to load constellation from '%s' / '%s' "
                 "(missing file or no satellites)\n",
                 opts.omm.c_str(), opts.stations.c_str());
    return 2;
  }

  if (!leo::resolve_endpoints(sim, opts.src, opts.dst, err)) {
    std::fprintf(stderr, "error: %s\n", err.c_str());
    return 2;
  }

  // Default to "right now" -- routing the live network is the natural interactive
  // request -- otherwise honor the requested instant. A malformed timestamp must
  // take the same clean "error / exit 2" path as every other input error rather
  // than throwing uncaught.
  double t_jd = 0.0;
  if (opts.time_iso.empty()) {
    t_jd = leo::now_jd_utc();
  } else {
    try {
      t_jd = leo::iso8601_to_jd(opts.time_iso);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "error: invalid --time '%s': %s\n",
                   opts.time_iso.c_str(), e.what());
      return 2;
    }
  }

  const leo::RouteSpec spec{opts.hop_cost_s};
  const leo::Snapshot s = sim.step(t_jd, spec);
  const leo::Constellation& c = sim.constellation();

  std::printf("%s -> %s  at %s UTC\n", opts.src.c_str(), opts.dst.c_str(),
              s.utc_iso.c_str());

  if (s.found) {
    std::printf("route: ");
    for (std::size_t i = 0; i < s.path.size(); ++i) {
      // Connector before each node tags the hop kind (in-plane / cross / link).
      if (i != 0) {
        std::printf("%s", hop_connector(c, s.path[i - 1], s.path[i]));
      }
      std::printf("%s", label_node(c, s.path[i], s.src_node, opts.src,
                                   s.dst_node, opts.dst)
                            .c_str());
    }
    std::printf("\n");
    std::printf("latency: %.1f ms over %zu hops\n", s.cost_ms, s.hops);

    // The headline comparison: how the LEO path stacks up against ideal fiber.
    // The ratio can fall below 1 on a short hop where the up/down legs dominate
    // and fiber wins, so phrase faster vs. slower explicitly -- "0.8x faster"
    // would contradict itself.
    if (s.cost_ms > 0.0) {
      const double ratio = s.fiber_baseline_ms / s.cost_ms;
      if (ratio >= 1.0) {
        std::printf("%.1f ms vs %.1f ms fiber (%.1fx faster than fiber)\n",
                    s.cost_ms, s.fiber_baseline_ms, ratio);
      } else {
        std::printf("%.1f ms vs %.1f ms fiber (%.1fx slower than fiber)\n",
                    s.cost_ms, s.fiber_baseline_ms, 1.0 / ratio);
      }
    }
  } else {
    const char* reason = "no route at this time";
    switch (s.no_path_reason) {
      case leo::Snapshot::NoPath::kSourceBlind:
        reason = "no satellite in view of the source at this time";
        break;
      case leo::Snapshot::NoPath::kDestBlind:
        reason = "no satellite in view of the destination at this time";
        break;
      case leo::Snapshot::NoPath::kDisconnected:
        reason =
            "source and destination are not connected through the network at "
            "this time";
        break;
      case leo::Snapshot::NoPath::kReachable:
        break;  // not reached when !found
    }
    std::printf("no route found: %s.\n", reason);
    std::printf(
        "(this is a normal outcome over a sparse snapshot at one instant, "
        "not an error)\n");
  }

  std::printf("%zu satellites usable, %zu planes\n", s.usable_sats,
              s.num_planes);
  if (!s.epoch_health.propagation_ok) {
    std::printf(
        "warning: snapshot epoch is %.1f days from the requested time; the "
        "answer is extrapolated and may be unreliable.\n",
        s.epoch_health.newest_age_days);
  }

  if (opts.diag) print_diag(c, s, opts.src, opts.dst);

  return s.found ? 0 : 1;
}
