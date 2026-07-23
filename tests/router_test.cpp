/*****************************************************************************
  filename router_test.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-22
  Brief Description:
    Unit tests for the Router over hand-built graphs (no SGP4): set
    position_ecef, isl_neighbors, and ground_uplinks directly, then build() and
    route. Covers CSR symmetry, edge feasibility gating (range and Earth
    clearance), and the guarantee that A* and Dijkstra return equal costs.
 *****************************************************************************/
// router_test.cpp -- routing over hand-built graphs (no SGP4): set position_ecef,
// isl_neighbors, and ground_uplinks directly, then build() and route.

#include "router.hpp"

#include <cmath>

#include <gtest/gtest.h>

#include "constellation.hpp"

namespace {

using leo::Constellation;
using leo::edge_seconds;
using leo::kLightKmPerS;
using leo::NodeId;
using leo::Router;
using leo::RouteResult;
using leo::RouteSpec;
using leo::Satellite;
using leo::Vec3;

// Add a satellite at a fixed ECEF position; returns its NodeId (insertion order,
// no build_topology so node index == add order).
NodeId sat(Constellation& c, leo::CatalogId cid, const Vec3& pos) {
  Satellite s;
  s.catalog_id = cid;
  const NodeId i = c.add_satellite(s);
  c.position_ecef[i] = pos;
  return i;
}

// Set one directed ISL terminal a[slot] -> b (build() makes it undirected).
void isl(Constellation& c, NodeId a, int slot, NodeId b) {
  c.isl_neighbors[a][slot] = b;
}

// Verify a found path is internally consistent against the CSR and weights.
void check_path(const Router& r, const Constellation& c, const RouteResult& res,
                NodeId src, NodeId dst, const RouteSpec& spec) {
  ASSERT_TRUE(res.found);
  ASSERT_FALSE(res.path.empty());
  EXPECT_EQ(res.path.front(), src);
  EXPECT_EQ(res.path.back(), dst);
  double sum = 0.0;
  for (std::size_t k = 1; k < res.path.size(); ++k) {
    EXPECT_TRUE(r.has_edge(res.path[k - 1], res.path[k]));
    sum += edge_seconds(c, res.path[k - 1], res.path[k], spec);
  }
  EXPECT_NEAR(sum, res.cost_s, 1e-9);
}

TEST(Router, SrcEqualsDst) {
  Constellation c;
  sat(c, 1, Vec3(7000, 0, 0));
  Router r;
  r.build(c);

  const RouteResult res = r.dijkstra(c, 0, 0);
  EXPECT_TRUE(res.found);
  EXPECT_DOUBLE_EQ(res.cost_s, 0.0);
  ASSERT_EQ(res.path.size(), 1u);
  EXPECT_EQ(res.path[0], 0u);
}

TEST(Router, TwoLinkedNodesCostIsLightTime) {
  Constellation c;
  const NodeId a = sat(c, 1, Vec3(7000, 0, 0));
  const NodeId b = sat(c, 2, Vec3(7000, 1000, 0));  // 1000 km apart
  isl(c, a, 0, b);  // single direction; build() symmetrizes
  Router r;
  r.build(c);

  const RouteResult res = r.dijkstra(c, a, b);
  ASSERT_TRUE(res.found);
  EXPECT_NEAR(res.cost_s, 1000.0 / kLightKmPerS, 1e-12);

  RouteSpec spec;
  spec.hop_cost_s = 2e-3;
  const RouteResult res2 = r.dijkstra(c, a, b, spec);
  EXPECT_NEAR(res2.cost_s, 1000.0 / kLightKmPerS + 2e-3, 1e-12);
}

// Diamond where 0-1-3 (200 km) beats 0-2-3 (~1300 km).
TEST(Router, KnownShortestPath) {
  Constellation c;
  // Offset into realistic LEO (radius > earth) so the router's earth_clear gate
  // admits the edges; translation preserves all distances, so the diamond and
  // its expected path/cost are unchanged.
  const NodeId n0 = sat(c, 1, Vec3(6900, 0, 0));
  const NodeId n1 = sat(c, 2, Vec3(7000, 0, 0));
  const NodeId n2 = sat(c, 3, Vec3(6900, 1000, 0));
  const NodeId n3 = sat(c, 4, Vec3(7100, 0, 0));
  isl(c, n0, 0, n1);
  isl(c, n0, 1, n2);
  isl(c, n1, 0, n3);
  isl(c, n2, 0, n3);
  Router r;
  r.build(c);

  const RouteResult res = r.dijkstra(c, n0, n3);
  ASSERT_TRUE(res.found);
  EXPECT_EQ(res.path, (std::vector<NodeId>{n0, n1, n3}));
  EXPECT_NEAR(res.cost_s, 200.0 / kLightKmPerS, 1e-12);
  check_path(r, c, res, n0, n3, {});
}

// A cross-plane slot set on only one endpoint must still route both ways.
TEST(Router, AsymmetricEdgeRoutesBothDirections) {
  Constellation c;
  const NodeId a = sat(c, 1, Vec3(7000, 0, 0));
  const NodeId b = sat(c, 2, Vec3(7000, 800, 0));
  isl(c, a, 2, b);  // only a knows about b
  Router r;
  r.build(c);

  const RouteResult fwd = r.dijkstra(c, a, b);
  const RouteResult rev = r.dijkstra(c, b, a);
  ASSERT_TRUE(fwd.found);
  ASSERT_TRUE(rev.found);
  EXPECT_NEAR(fwd.cost_s, rev.cost_s, 1e-12);
  EXPECT_EQ(rev.path, (std::vector<NodeId>{b, a}));
}

TEST(Router, NoPathReturnsNotFound) {
  Constellation c;
  // Realistic LEO radii (see KnownShortestPath); two deliberately unlinked
  // components.
  const NodeId a = sat(c, 1, Vec3(6900, 0, 0));
  const NodeId b = sat(c, 2, Vec3(7000, 0, 0));
  const NodeId x = sat(c, 3, Vec3(6900, 5000, 0));
  const NodeId y = sat(c, 4, Vec3(7000, 5000, 0));
  isl(c, a, 0, b);  // component {a,b}
  isl(c, x, 0, y);  // component {x,y}
  Router r;
  r.build(c);

  const RouteResult res = r.dijkstra(c, a, y);
  EXPECT_FALSE(res.found);
  EXPECT_TRUE(res.path.empty());
}

// Build a 3x3 grid and compare A* to Dijkstra over several pairs.
TEST(Router, AstarEqualsDijkstra) {
  Constellation c;
  NodeId id[9];
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      // Offset into LEO so the earth_clear gate admits the grid edges; the
      // 100 km spacing keeps every edge well under max_range.
      id[row * 3 + col] = sat(c, row * 3 + col + 1,
                              Vec3(6900.0 + col * 100.0, row * 100.0, 0.0));
    }
  }
  // Edge to the right and down neighbor; build() adds the reverse edges.
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const NodeId here = id[row * 3 + col];
      if (col < 2) isl(c, here, 0, id[row * 3 + col + 1]);
      if (row < 2) isl(c, here, 1, id[(row + 1) * 3 + col]);
    }
  }
  Router r;
  r.build(c);

  const std::pair<NodeId, NodeId> pairs[] = {
      {id[0], id[8]}, {id[2], id[6]}, {id[0], id[5]}, {id[4], id[8]}};
  for (const double hop : {0.0, 3e-3}) {
    RouteSpec spec;
    spec.hop_cost_s = hop;
    for (const auto& [s, d] : pairs) {
      const RouteResult dj = r.dijkstra(c, s, d, spec);
      const RouteResult as = r.astar(c, s, d, spec);
      ASSERT_TRUE(dj.found);
      ASSERT_TRUE(as.found);
      EXPECT_NEAR(as.cost_s, dj.cost_s, 1e-9);
      // A* with a consistent heuristic never expands more than Dijkstra.
      EXPECT_LE(as.nodes_expanded, dj.nodes_expanded);
      check_path(r, c, as, s, d, spec);
    }
  }
}

// Ground -> uplink sat -> ISL -> downlink sat -> ground, end to end.
TEST(Router, GroundToGroundEndToEnd) {
  Constellation c;
  const NodeId s0 = sat(c, 1, Vec3(7000, 0, 0));
  const NodeId s1 = sat(c, 2, Vec3(7000, 500, 0));
  isl(c, s0, 0, s1);  // ISL, 500 km

  const NodeId gsrc = c.add_ground_node(Vec3(6378, 0, 0));    // under s0
  const NodeId gdst = c.add_ground_node(Vec3(6378, 500, 0));  // under s1
  c.ground_uplinks.assign(c.ground_count(), {});
  c.ground_uplinks[0] = {s0};  // gsrc sees s0
  c.ground_uplinks[1] = {s1};  // gdst sees s1

  Router r;
  r.build(c);

  const RouteResult res = r.dijkstra(c, gsrc, gdst);
  ASSERT_TRUE(res.found);
  EXPECT_EQ(res.path, (std::vector<NodeId>{gsrc, s0, s1, gdst}));
  const double expect = (622.0 + 500.0 + 622.0) / kLightKmPerS;
  EXPECT_NEAR(res.cost_s, expect, 1e-9);
  check_path(r, c, res, gsrc, gdst, {});
}

// A structurally-adjacent in-plane pair that is too far apart must NOT yield a
// routable edge: the router's feasibility gate rejects it even though topology
// linked it. (Both endpoints stay above the block radius so this isolates the
// max_range check, not earth_clear.)
TEST(Router, InPlaneOverMaxRangeIsNotRoutable) {
  Constellation c;
  const NodeId a = sat(c, 1, Vec3(6900, 0, 0));
  const NodeId b = sat(c, 2, Vec3(6900, 6000, 0));  // 6000 km, well over budget
  isl(c, a, 0, b);  // in-plane fore link, wired by topology without range check
  Router r;
  r.build(c);

  EXPECT_FALSE(r.has_edge(a, b));  // gate dropped the over-long edge
  const RouteResult res = r.dijkstra(c, a, b);
  EXPECT_FALSE(res.found);  // it was the only edge, so no route remains
}

// Pins the operational budget (3000 km): a ~3500 km pair would have routed under
// the old 5400 km range but must now be rejected.
TEST(Router, InPlaneAtOperationalLimitIsNotRoutable) {
  Constellation c;
  const NodeId a = sat(c, 1, Vec3(6900, 0, 0));
  const NodeId b = sat(c, 2, Vec3(6900, 3500, 0));  // 3500 km: < 5400 but > 3000
  isl(c, a, 0, b);
  Router r;
  r.build(c);

  EXPECT_FALSE(r.has_edge(a, b));
  const RouteResult res = r.dijkstra(c, a, b);
  EXPECT_FALSE(res.found);
}

// Regression guard: the same in-plane link within range still routes.
TEST(Router, InPlaneWithinRangeRoutes) {
  Constellation c;
  const NodeId a = sat(c, 1, Vec3(6900, 0, 0));
  const NodeId b = sat(c, 2, Vec3(6900, 1000, 0));  // 1000 km < 5400
  isl(c, a, 0, b);
  Router r;
  r.build(c);

  EXPECT_TRUE(r.has_edge(a, b));
  const RouteResult res = r.dijkstra(c, a, b);
  ASSERT_TRUE(res.found);
  EXPECT_NEAR(res.cost_s, 1000.0 / kLightKmPerS, 1e-12);
}

}  // namespace
