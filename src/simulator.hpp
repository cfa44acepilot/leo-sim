/*****************************************************************************
  filename simulator.hpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-14
  Brief Description:
    The headless driver both front ends are built on. The `leo` CLI and the
    Vulkan monitor are thin wrappers over this class, which is why proving THIS
    deterministic proves the two agree. The three-tier rule below is the
    load-bearing invariant of the whole system.
 *****************************************************************************/

// Headless driver: turns "a time + two endpoints" into a routed result plus a
// render-ready Snapshot. Structured as three tiers that run at different rates,
// so animation re-runs only the cheap per-tick work:
//   Tier 1 (per dataset): load() -- OMM + stations, build_topology, prop.init.
//   Tier 2 (per query):   set_endpoints*() -- place the two ground nodes.
//   Tier 3 (per tick):    step()/refresh() -- propagate, links, uplinks, route.
// HARD RULE: build_topology and Propagator::init run ONLY in Tier 1. Re-running
// build_topology would reorder every dense array, desync the propagator and
// renumber nodes under the renderer.

#ifndef LEO_SIMULATOR_HPP
#define LEO_SIMULATOR_HPP

#include <cstddef>      /* std::size_t                                   */
#include <cstdint>      /* fixed-width snapshot fields                   */
#include <filesystem>   /* the dataset and station paths                 */
#include <string>       /* names, the ISO timestamp, warnings            */
#include <string_view>  /* endpoint lookup without copying               */
#include <vector>       /* the path, the edges, the validity flags       */

#include <ankerl/unordered_dense.h>  /* station name -> index            */

#include "constellation.hpp"
#include "epoch_health.hpp"
#include "ground_stations.hpp"
#include "isl_links.hpp"
#include "propagator.hpp"
#include "router.hpp"
#include "topology.hpp"

namespace leo
{

// Terrestrial fiber baseline: light in glass (index ~1.467). An idealized
// one-way lower bound -- real fiber is longer with router hops -- so comparing
// the LEO path against it is conservative (generous to fiber).
constexpr double kFiberKmPerS = kLightKmPerS / 1.467;  // ~204357 km/s

// Great-circle surface distance (km) between two ECEF points: central angle
// times mean Earth radius. Free + exposed so tests can pin it directly.
double great_circle_km(const Vec3& a_ecef, const Vec3& b_ecef);

// A render-ready, self-describing result for one instant.
struct Snapshot
{
  // The answer.
  bool found = false;
  double cost_s = 0.0;
  double cost_ms = 0.0;      // cost_s * 1000, convenience for HUDs
  std::size_t hops = 0;      // path.size() - 1
  std::vector<NodeId> path;  // src..dst; empty if !found
  NodeId src_node = INVALID_NODE;
  NodeId dst_node = INVALID_NODE;
  enum class NoPath
  {
    kReachable,
    kSourceBlind,
    kDestBlind,
    kDisconnected
  };
  NoPath no_path_reason = NoPath::kReachable;
  double great_circle_km = 0.0;
  double fiber_baseline_ms = 0.0;

  // Renderer bridge. Positions are NOT copied here (single-threaded v1): the
  // renderer reads c.position_eci/ecef and is_ground from the constellation.
  double gmst_rad = 0.0;              // Earth orientation / camera lock
  double atmosphere_radius_km = 0.0;  // = LinkSpec block_radius (drawn shell)
  struct Edge
  {
    NodeId a, b;  // a < b
    enum Kind
    {
      kInPlane,
      kCrossPlane,
      kUplink
    } kind;
  };
  std::vector<Edge> edges;  // undirected, deduped, type-tagged
  std::vector<std::uint8_t>
      position_valid;  // per node: 1 if drawable this tick

  // Trustworthiness + echoed inputs.
  double t_jd = 0.0;
  std::string utc_iso;
  EpochHealth epoch_health;
  std::size_t usable_sats = 0;
  std::size_t propagation_errors = 0;
  std::size_t num_planes = 0;
  std::vector<std::string> warnings;
};

// Build the typed, deduped undirected edge list for the lattice draw (separate
// from the router's CSR). Free + exposed so tests can check the Kind tagging
// without driving a full refresh. Slots 0,1 -> kInPlane; 2,3 -> kCrossPlane;
// ground uplinks -> kUplink.
void assemble_edges(const Constellation& c, std::vector<Snapshot::Edge>& out);

class Simulator
{
 public:
  // Tier 1: load the dataset, build the static lattice, init the propagator.
  // Returns false if no satellites loaded.
  bool load(const std::filesystem::path& omm_json,
            const std::filesystem::path& stations_json,
            const ShellSpec& shell = {});

  // Tier 2: place the two endpoints, clearing any previous ground nodes first.
  // The name form returns false (leaving state unchanged) if a name is unknown.
  bool set_endpoints(std::string_view src_name, std::string_view dst_name);
  bool set_endpoints_geo(double src_lat, double src_lon, double dst_lat,
                         double dst_lon);

  // Tier 3. step = propagate then refresh. refresh re-seeds links + uplinks,
  // rebuilds the router graph, routes, and assembles the snapshot from CURRENT
  // positions -- exposed separately as the test seam (set positions by hand,
  // call refresh, no SGP4) and so endpoints can change without re-propagating.
  Snapshot step(double t_jd, const RouteSpec& spec = {});
  Snapshot refresh(double t_jd, const RouteSpec& spec = {});

  // The physical link budget (ISL max range, block radius, min velocity dot,
  // uplink min elevation) that both cross-plane seeding and the router's
  // feasibility gate consume. Settable so a frontend can drive both from one
  // spec; the default reproduces today's behavior exactly. Takes effect on the
  // next refresh()/step() -- neither reorders arrays, so it is safe per-tick.
  void set_link_spec(const LinkSpec& spec) { link_spec_ = spec; }
  const LinkSpec& link_spec() const { return link_spec_; }

  // For the renderer's marker layer (all stations) and live positions.
  const std::vector<GroundStation>& stations() const { return stations_; }
  const Constellation& constellation() const { return c_; }
  // Mutable handle for setup/tests (e.g. setting positions before refresh).
  Constellation& constellation() { return c_; }

 private:
  Constellation c_;
  Propagator prop_;
  Router router_;
  std::vector<GroundStation> stations_;
  ankerl::unordered_dense::map<std::string, std::size_t> station_by_name_;
  NodeId src_node_ = INVALID_NODE;
  NodeId dst_node_ = INVALID_NODE;
  // Single source of truth for the link budget; default-constructed to today's
  // values so an unset simulator behaves byte-for-byte as before.
  LinkSpec link_spec_;
  // build_topology warnings captured at load, carried into every snapshot.
  std::vector<std::string> load_warnings_;
};

}  // namespace leo

#endif  // LEO_SIMULATOR_HPP
