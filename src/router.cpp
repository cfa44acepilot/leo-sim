/*****************************************************************************
  filename router.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-22
  Brief Description:
    Builds the symmetric CSR from the ISL slots and ground uplinks (each link
    added both ways, then sorted and deduped so asymmetric picks still route),
    gating each sat-to-sat edge by range and Earth clearance. One lazy-deletion
    heap search backs both dijkstra (h=0) and astar (light-time heuristic).
 *****************************************************************************/
// router.cpp -- symmetric CSR build + shared Dijkstra/A* search core.

#include "router.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <tuple>

#include <glm/geometric.hpp>  // glm::distance

#include "isl_links.hpp"  // earth_clear (edge feasibility)

namespace leo {

double edge_seconds(const Constellation& c, NodeId u, NodeId v,
                    const RouteSpec& spec) {
  return glm::distance(c.position_ecef[u], c.position_ecef[v]) / kLightKmPerS +
         spec.hop_cost_s;
}

void Router::build(const Constellation& c, const LinkSpec& spec) {
  const std::size_t n = c.node_count();
  std::vector<std::vector<NodeId>> adj(n);

  // Always add BOTH directions: an asymmetric cross-plane pick (i lists j but
  // not vice versa) still yields a usable j->i edge. In-plane edges arrive from
  // both ends and the later dedup collapses them. This is also the one
  // chokepoint where physical feasibility is enforced, so an over-long or
  // occluded structural link from ANY source never becomes routable -- topology
  // owns "structurally adjacent", the router owns "usable this tick".
  const auto link = [&](NodeId a, NodeId b) {
    if (a == b) return;
    // Ground<->satellite uplinks are already filtered by connect_ground_uplinks
    // (min elevation / ~1100 km slant) and have a surface endpoint, so the
    // limb-grazing earth_clear test would wrongly reject every one -- admit
    // them as-is. Only satellite<->satellite edges get the physical gate.
    if (!c.is_ground(a) && !c.is_ground(b)) {
      const Vec3& pa = c.position_ecef[a];
      const Vec3& pb = c.position_ecef[b];
      if (glm::distance(pa, pb) > spec.max_range_km) return;       // too far
      if (!earth_clear(pa, pb, spec.block_radius_km)) return;      // occluded
    }
    adj[a].push_back(b);
    adj[b].push_back(a);
  };

  // ISL edges: every non-empty terminal of every satellite.
  for (std::size_t i = 0; i < c.num_satellites; ++i) {
    for (int t = 0; t < NUM_ISL_TERMINALS; ++t) {
      const NodeId nb = c.isl_neighbors[i][t];
      if (nb != INVALID_NODE) link(static_cast<NodeId>(i), nb);
    }
  }

  // Ground uplinks: ground node (num_satellites + g) to each visible satellite.
  // Satellites never store ground nodes in ISL slots and ground nodes carry no
  // ISL slots, so uplinks are the only ground/satellite edges -- no special case.
  for (std::size_t g = 0; g < c.ground_count(); ++g) {
    const NodeId gnode = static_cast<NodeId>(c.num_satellites + g);
    for (const NodeId s : c.ground_uplinks[g]) link(gnode, s);
  }

  // Sort + dedup each row, then flatten into CSR (offsets_ = prefix sum).
  offsets_.assign(n + 1, 0);
  for (std::size_t i = 0; i < n; ++i) {
    std::vector<NodeId>& a = adj[i];
    std::sort(a.begin(), a.end());
    a.erase(std::unique(a.begin(), a.end()), a.end());
    offsets_[i + 1] = offsets_[i] + static_cast<std::uint32_t>(a.size());
  }
  neighbors_.clear();
  neighbors_.reserve(offsets_[n]);
  for (std::size_t i = 0; i < n; ++i) {
    neighbors_.insert(neighbors_.end(), adj[i].begin(), adj[i].end());
  }
}

bool Router::has_edge(NodeId a, NodeId b) const {
  for (std::uint32_t k = offsets_[a]; k < offsets_[a + 1]; ++k) {
    if (neighbors_[k] == b) return true;
  }
  return false;
}

RouteResult Router::search(const Constellation& c, NodeId src, NodeId dst,
                           const RouteSpec& spec, bool use_heuristic) {
  RouteResult res;

  // Trivial route: no edges, zero latency.
  if (src == dst) {
    res.found = true;
    res.path = {src};
    return res;
  }

  const std::size_t n = c.node_count();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  dist_.assign(n, kInf);
  came_.assign(n, INVALID_NODE);

  // Admissible+consistent heuristic: straight-line light-time to dst, never more
  // than the remaining real time (which also pays hop_cost_s >= 0). A* with this
  // h returns the same cost as Dijkstra (h == 0). dst position is fixed.
  const Vec3 dst_pos = c.position_ecef[dst];
  const auto h = [&](NodeId node) -> double {
    if (!use_heuristic) return 0.0;
    return glm::distance(c.position_ecef[node], dst_pos) / kLightKmPerS;
  };

  // Min-heap of (priority, g, node). Lazy deletion: a pop whose g exceeds the
  // recorded dist_ is stale and skipped, so no separate closed set is needed.
  using QItem = std::tuple<double, double, NodeId>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;

  dist_[src] = 0.0;
  pq.emplace(h(src), 0.0, src);

  while (!pq.empty()) {
    const auto [prio, g, node] = pq.top();
    pq.pop();
    if (g > dist_[node]) continue;  // stale entry

    ++res.nodes_expanded;
    if (node == dst) break;  // first non-stale pop of dst is optimal

    for (std::uint32_t k = offsets_[node]; k < offsets_[node + 1]; ++k) {
      const NodeId v = neighbors_[k];
      const double ng = g + edge_seconds(c, node, v, spec);
      if (ng < dist_[v]) {
        dist_[v] = ng;
        came_[v] = node;
        pq.emplace(ng + h(v), ng, v);
      }
    }
  }

  if (dist_[dst] == kInf) return res;  // disconnected: found stays false

  // Walk predecessors dst -> src, then reverse into src..dst order.
  res.found = true;
  res.cost_s = dist_[dst];
  for (NodeId at = dst; at != src; at = came_[at]) res.path.push_back(at);
  res.path.push_back(src);
  std::reverse(res.path.begin(), res.path.end());
  return res;
}

RouteResult Router::dijkstra(const Constellation& c, NodeId src, NodeId dst,
                             const RouteSpec& spec) {
  return search(c, src, dst, spec, /*use_heuristic=*/false);
}

RouteResult Router::astar(const Constellation& c, NodeId src, NodeId dst,
                          const RouteSpec& spec) {
  return search(c, src, dst, spec, /*use_heuristic=*/true);
}

}  // namespace leo
