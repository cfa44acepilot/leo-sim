// router.hpp
//
// Lowest-latency routing over the constellation graph: in-plane and cross-plane
// ISL links plus ground uplinks. Two algorithms share one search core --
// Dijkstra (reference) and A* (primary) -- so "A* must equal Dijkstra" is a real
// test of the same code path. Edge weights are computed on the fly from
// positions; only the adjacency (a symmetric CSR) is materialized.
//
// build() must run AFTER seed_cross_plane_links and connect_ground_uplinks, and
// the constellation must not be re-propagated between build() and a route (the
// search reads position_ecef for weights).

#ifndef LEO_ROUTER_HPP
#define LEO_ROUTER_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include "constellation.hpp"
#include "isl_links.hpp"  // LinkSpec + earth_clear, reused for edge feasibility

namespace leo {

// Laser ISLs (and free-space uplinks) travel at the speed of light.
constexpr double kLightKmPerS = 299792.458;

struct RouteSpec {
  // Flat per-hop switching/processing cost folded into each edge. ~1e-3..5e-3 s
  // for realism; 0 makes latency pure light-time.
  double hop_cost_s = 0.0;
};

struct RouteResult {
  bool found = false;
  double cost_s = 0.0;             // total latency, seconds
  std::vector<NodeId> path;        // src..dst inclusive; empty if !found
  std::size_t nodes_expanded = 0;  // non-stale pops; lets tests compare A*/Dijkstra
};

// Latency of the edge u-v: light-time over the straight-line distance plus the
// flat hop cost. Both endpoints carry position_ecef (satellites and ground
// alike), so this is uniform. Exposed so tests can re-sum a path's cost.
double edge_seconds(const Constellation& c, NodeId u, NodeId v,
                    const RouteSpec& spec);

class Router {
 public:
  // Rebuild the symmetric CSR from the current lattice + uplinks (wholesale,
  // once per tick). Satellite<->satellite edges are admitted only if physically
  // feasible against current positions (within spec.max_range_km and not
  // Earth-occluded), so an over-long structural in-plane link never becomes
  // routable. Pass the same LinkSpec the per-tick link seeding used.
  void build(const Constellation& c, const LinkSpec& spec = {});

  RouteResult dijkstra(const Constellation& c, NodeId src, NodeId dst,
                       const RouteSpec& spec = {});
  RouteResult astar(const Constellation& c, NodeId src, NodeId dst,
                    const RouteSpec& spec = {});

  // True if b is in a's CSR adjacency row. Diagnostic / test helper.
  bool has_edge(NodeId a, NodeId b) const;

 private:
  RouteResult search(const Constellation& c, NodeId src, NodeId dst,
                     const RouteSpec& spec, bool use_heuristic);

  std::vector<std::uint32_t> offsets_;  // size node_count()+1, prefix sum
  std::vector<NodeId> neighbors_;       // size = total directed half-edges
  // Reused scratch so repeated routes don't reallocate (the cheap stand-in for
  // a custom arena; a real arena is a later refinement if profiling asks).
  std::vector<double> dist_;
  std::vector<NodeId> came_;
};

}  // namespace leo

#endif  // LEO_ROUTER_HPP
