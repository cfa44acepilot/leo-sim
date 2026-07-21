/*****************************************************************************
  filename simulator.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-14
  Brief Description:
    The three-tier driver and the snapshot assembly. refresh() is where the
    per-tick graph is rebuilt -- both link-seeding passes, the uplinks, the
    router, and the route -- all from ONE LinkSpec, so the budget the links were
    seeded under and the budget the router gates edges with cannot drift apart.
 *****************************************************************************/

#include "simulator.hpp"

#include <algorithm>  /* sort/unique/clamp: the deduped edge list          */
#include <cmath>      /* std::acos: the great-circle baseline              */
#include <tuple>      /* std::tie: ordering edges by (a,b)                 */

#include <glm/geometric.hpp>  /* normalize/dot/length: geometry + culling  */

#include "ground.hpp"      /* geodetic_to_ecef, connect_ground_uplinks     */
#include "isl_links.hpp"   /* both seeding passes + the shared LinkSpec    */
#include "omm_loader.hpp"  /* load_omm_json, jd_to_iso8601                 */

namespace leo
{

namespace
{

// Mean Earth radius for the great-circle measure.
constexpr double kMeanEarthKm = 6371.0;

// A live satellite sits well above this; a zeroed (failed-prop) position is at
// the origin. Used to flag drawable nodes.
constexpr double kCullRadiusKm = 6371.0;

}  // namespace

double great_circle_km(const Vec3& a_ecef, const Vec3& b_ecef)
{
  const Vec3 na = glm::normalize(a_ecef);
  const Vec3 nb = glm::normalize(b_ecef);
  // Clamp guards acos against dot drifting just past +/-1 from rounding.
  const double cosang = std::clamp(glm::dot(na, nb), -1.0, 1.0);
  return std::acos(cosang) * kMeanEarthKm;
}

void assemble_edges(const Constellation& c, std::vector<Snapshot::Edge>& out)
{
  using Edge = Snapshot::Edge;
  out.clear();

  // Satellite ISL terminals: slots 0,1 are in-plane, 2,3 are cross-plane.
  for (std::size_t i = 0; i < c.num_satellites; ++i)
  {
    for (int t = 0; t < NUM_ISL_TERMINALS; ++t)
    {
      const NodeId nb = c.isl_neighbors[i][t];
      if (nb == INVALID_NODE)
      {
        continue;
      }
      const Edge::Kind kind = (t < 2) ? Edge::kInPlane : Edge::kCrossPlane;
      const NodeId a = std::min<NodeId>(static_cast<NodeId>(i), nb);
      const NodeId b = std::max<NodeId>(static_cast<NodeId>(i), nb);
      out.push_back(Edge{a, b, kind});
    }
  }

  // Ground uplinks.
  for (std::size_t g = 0; g < c.ground_count(); ++g)
  {
    const NodeId gnode = static_cast<NodeId>(c.num_satellites + g);
    for (const NodeId s : c.ground_uplinks[g])
    {
      out.push_back(
          Edge{std::min(gnode, s), std::max(gnode, s), Edge::kUplink});
    }
  }

  // Sort + unique on (a,b): the symmetric in-plane pair (emitted from both
  // ends) collapses to one entry. In-plane and cross-plane never share a pair
  // (one is same-plane, the other adjacent-plane), so no Kind conflict on
  // dedup.
  std::sort(out.begin(), out.end(), [](const Edge& x, const Edge& y)
            { return std::tie(x.a, x.b) < std::tie(y.a, y.b); });
  out.erase(std::unique(out.begin(), out.end(), [](const Edge& x, const Edge& y)
                        { return x.a == y.a && x.b == y.b; }),
            out.end());
}

bool Simulator::load(const std::filesystem::path& omm_json,
                     const std::filesystem::path& stations_json,
                     const ShellSpec& shell)
{
  // Fresh dataset: reset everything so a reload doesn't accrete state.
  c_ = Constellation{};
  stations_.clear();
  station_by_name_.clear();
  src_node_ = INVALID_NODE;
  dst_node_ = INVALID_NODE;

  load_omm_json(omm_json, c_);
  if (c_.satellite_count() == 0)
  {
    return false;  // nothing to simulate
  }

  const BuildReport br = build_topology(c_, shell);  // Tier 1 only
  load_warnings_ = br.warnings;

  load_ground_stations(stations_json, stations_);
  for (std::size_t i = 0; i < stations_.size(); ++i)
  {
    station_by_name_[stations_[i].name] = i;
  }

  prop_.init(c_);  // Tier 1 only; pairs records to the sorted satellite order
  return true;
}

bool Simulator::set_endpoints(std::string_view src_name,
                              std::string_view dst_name)
{
  // Resolve both names BEFORE mutating, so an unknown name leaves state intact.
  const auto si = station_by_name_.find(std::string(src_name));
  const auto di = station_by_name_.find(std::string(dst_name));
  if (si == station_by_name_.end() || di == station_by_name_.end())
  {
    return false;
  }
  const GroundStation& s = stations_[si->second];
  const GroundStation& d = stations_[di->second];
  return set_endpoints_geo(s.lat_deg, s.lon_deg, d.lat_deg, d.lon_deg);
}

bool Simulator::set_endpoints_geo(double src_lat, double src_lon,
                                  double dst_lat, double dst_lon)
{
  // Drop old endpoints (add_ground_node only appends), then place the two new
  // ones in the tail. Satellite prefix and topology are untouched.
  c_.clear_ground();
  src_node_ = c_.add_ground_node(geodetic_to_ecef(src_lat, src_lon, 0.0));
  dst_node_ = c_.add_ground_node(geodetic_to_ecef(dst_lat, dst_lon, 0.0));
  return true;
}

Snapshot Simulator::step(double t_jd, const RouteSpec& spec)
{
  prop_.propagate(c_, t_jd);
  return refresh(t_jd, spec);
}

Snapshot Simulator::refresh(double t_jd, const RouteSpec& spec)
{
  // Per-tick graph from the current positions, then route. The settable member
  // link_spec_ drives BOTH link seeding passes and the router's feasibility
  // gate, so the range/LOS budget the router enforces matches the one the links
  // were seeded under -- one spec, so a frontend override stays consistent.
  //
  // Both passes are geometric and both run before the router is built: slots
  // 0/1 hold in-plane links to the satellites actually nearest ahead/behind
  // within the group (NOT build_topology's slot-order ring, which on real data
  // links satellites thousands of km apart and leaves the router routing round
  // short links that exist), slots 2/3 hold the cross-plane links.
  seed_in_plane_links(c_, link_spec_);
  seed_cross_plane_links(c_, link_spec_);
  connect_ground_uplinks(c_);
  router_.build(c_, link_spec_);
  const RouteResult r = router_.astar(c_, src_node_, dst_node_, spec);

  Snapshot s;
  s.found = r.found;
  s.cost_s = r.cost_s;
  s.cost_ms = r.cost_s * 1000.0;
  s.hops = r.path.empty() ? 0 : r.path.size() - 1;
  s.path = r.path;
  s.src_node = src_node_;
  s.dst_node = dst_node_;

  // Why no route: a blind endpoint (no satellite in view) is distinct from a
  // genuinely disconnected graph.
  const auto uplinks_empty = [&](NodeId n)
  {
    if (!c_.is_ground(n))
    {
      return true;
    }
    const std::size_t g = n - c_.num_satellites;
    return g >= c_.ground_uplinks.size() || c_.ground_uplinks[g].empty();
  };
  if (r.found)
  {
    s.no_path_reason = Snapshot::NoPath::kReachable;
  }
  else if (uplinks_empty(src_node_))
  {
    s.no_path_reason = Snapshot::NoPath::kSourceBlind;
  }
  else if (uplinks_empty(dst_node_))
  {
    s.no_path_reason = Snapshot::NoPath::kDestBlind;
  }
  else
  {
    s.no_path_reason = Snapshot::NoPath::kDisconnected;
  }

  if (src_node_ != INVALID_NODE && dst_node_ != INVALID_NODE)
  {
    s.great_circle_km = great_circle_km(c_.position_ecef[src_node_],
                                        c_.position_ecef[dst_node_]);
    s.fiber_baseline_ms = s.great_circle_km / kFiberKmPerS * 1000.0;
  }

  s.gmst_rad = gmst_rad(t_jd);
  s.atmosphere_radius_km = link_spec_.block_radius_km;  // same spec as routing
  assemble_edges(c_, s.edges);

  // Cull zeroed/dead satellites; ground nodes are always drawable.
  const std::size_t n = c_.node_count();
  s.position_valid.assign(n, 0);
  for (std::size_t i = 0; i < n; ++i)
  {
    const bool ok = c_.is_ground(static_cast<NodeId>(i)) ||
                    glm::length(c_.position_ecef[i]) >= kCullRadiusKm;
    s.position_valid[i] = ok ? 1 : 0;
  }

  s.t_jd = t_jd;
  s.utc_iso = jd_to_iso8601(t_jd);
  s.epoch_health = check_epoch_health(c_, t_jd);
  s.usable_sats = prop_.usable_count();
  s.propagation_errors = prop_.last_error_count();
  s.num_planes = c_.num_planes;
  s.warnings = load_warnings_;
  if (!s.epoch_health.propagation_ok)
  {
    s.warnings.push_back(
        "snapshot epoch is stale for propagation (elements too old)");
  }
  return s;
}

}  // namespace leo
