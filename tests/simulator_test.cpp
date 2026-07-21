// simulator_test.cpp -- prerequisites + three-tier wiring (positions by hand).

#include "simulator.hpp"

#include <algorithm>
#include <cmath>       /* std::asin/atan2/sqrt (the subsatellite point)      */
#include <filesystem>

#include <gtest/gtest.h>

#include "constellation.hpp"
#include "topology.hpp"

namespace {

using leo::assemble_edges;
using leo::Constellation;
using leo::great_circle_km;
using leo::NodeId;
using leo::Satellite;
using leo::ShellSpec;
using leo::Simulator;
using leo::Snapshot;
using leo::Vec3;

std::filesystem::path fixture(const char* name) {
  return std::filesystem::path(LEO_TEST_DATA_DIR) / name;
}

void add_sat(Constellation& c, leo::CatalogId cid, double raan, double ma) {
  Satellite s;
  s.catalog_id = cid;
  s.inclination = 53.0;
  s.raan = raan;
  s.mean_anomaly = ma;
  s.epoch_jd = 2461202.62;
  c.add_satellite(s);
}

const Snapshot::Edge* find_edge(const std::vector<Snapshot::Edge>& es, NodeId a,
                                NodeId b) {
  const NodeId lo = std::min(a, b), hi = std::max(a, b);
  for (const auto& e : es) {
    if (e.a == lo && e.b == hi) return &e;
  }
  return nullptr;
}

// clear_ground drops the tail back to the post-topology state.
TEST(Simulator, ClearGroundRestoresPostTopologyState) {
  Constellation c;
  add_sat(c, 1, 10.0, 0.0);
  add_sat(c, 2, 25.0, 0.0);
  leo::build_topology(c, ShellSpec{});

  c.add_ground_node(Vec3(6378, 0, 0));
  c.add_ground_node(Vec3(0, 6378, 0));
  ASSERT_EQ(c.ground_count(), 2u);

  c.clear_ground();
  EXPECT_EQ(c.node_count(), c.satellite_count());
  EXPECT_EQ(c.ground_count(), 0u);
  EXPECT_TRUE(c.ground_uplinks.empty());
  EXPECT_TRUE(c.verify());
}

// Equator points 90 deg apart span a quarter of the great circle.
TEST(Simulator, GreatCircleQuarterCircumference) {
  const double q = great_circle_km(Vec3(6378, 0, 0), Vec3(0, 6378, 0));
  EXPECT_NEAR(q, 3.14159265358979 / 2.0 * 6371.0, 1.0);  // ~10007.5 km
}

// One in-plane, one cross-plane, one uplink edge -> three Edges, right Kinds.
TEST(Simulator, TypedEdgeAssembly) {
  Constellation c;
  add_sat(c, 1, 10.0, 0.0);    // plane 0
  add_sat(c, 2, 10.0, 120.0);  // plane 0 (in-plane partner of sat 1)
  add_sat(c, 3, 25.0, 0.0);    // plane 1
  leo::build_topology(c, ShellSpec{});  // wires in-plane slots 0,1

  const NodeId n1 = c.find(1)->index;
  const NodeId n2 = c.find(2)->index;
  const NodeId n3 = c.find(3)->index;

  // Hand-set one cross-plane terminal and one ground uplink.
  c.isl_neighbors[n1][2] = n3;
  const NodeId g = c.add_ground_node(Vec3(6378, 0, 0));
  c.ground_uplinks.assign(c.ground_count(), {});
  c.ground_uplinks[0] = {n2};

  std::vector<Snapshot::Edge> edges;
  assemble_edges(c, edges);

  const Snapshot::Edge* in_plane = find_edge(edges, n1, n2);
  const Snapshot::Edge* cross = find_edge(edges, n1, n3);
  const Snapshot::Edge* uplink = find_edge(edges, g, n2);
  ASSERT_NE(in_plane, nullptr);
  ASSERT_NE(cross, nullptr);
  ASSERT_NE(uplink, nullptr);
  EXPECT_EQ(in_plane->kind, Snapshot::Edge::kInPlane);
  EXPECT_EQ(cross->kind, Snapshot::Edge::kCrossPlane);
  EXPECT_EQ(uplink->kind, Snapshot::Edge::kUplink);
}

// A source whose uplink list is empty reports kSourceBlind, not a generic fail.
TEST(Simulator, NoPathSourceBlind) {
  Simulator sim;
  ASSERT_TRUE(sim.load(fixture("test_omm.json"), fixture("test_stations.json")));

  Constellation& c = sim.constellation();
  const NodeId a = c.find(270001)->index;
  const NodeId b = c.find(44714)->index;
  c.position_ecef[a] = Vec3(6900, 0, 0);
  c.position_ecef[b] = Vec3(6900, 1000, 0);
  c.velocity_eci[a] = Vec3(0, 1, 0);
  c.velocity_eci[b] = Vec3(0, 1, 0);

  // Source on the far (-x) side sees no satellite; dest over the sats does.
  sim.set_endpoints_geo(0.0, 180.0, 0.0, 0.0);
  const Snapshot s = sim.refresh(2461202.6);

  EXPECT_FALSE(s.found);
  EXPECT_EQ(s.no_path_reason, Snapshot::NoPath::kSourceBlind);
}

// Tier wiring: a reachable query fills a self-consistent snapshot, and a second
// query swaps endpoints without leaking a ground tail.
TEST(Simulator, RefreshAssemblesConsistentSnapshot) {
  Simulator sim;
  ASSERT_TRUE(sim.load(fixture("test_omm.json"), fixture("test_stations.json")));

  Constellation& c = sim.constellation();
  const NodeId a = c.find(270001)->index;
  const NodeId b = c.find(44714)->index;
  c.position_ecef[a] = Vec3(6900, 0, 0);
  c.position_ecef[b] = Vec3(6900, 1000, 0);  // 1000 km from a, same direction
  c.velocity_eci[a] = Vec3(0, 1, 0);
  c.velocity_eci[b] = Vec3(0, 1, 0);

  sim.set_endpoints_geo(0.0, 0.0, 0.0, 8.24);  // both under the satellites
  const std::size_t nodes = c.node_count();
  const Snapshot s = sim.refresh(2461202.6);

  // Snapshot fields are populated.
  EXPECT_GT(s.gmst_rad, 0.0);
  EXPECT_LT(s.gmst_rad, 6.2832);
  EXPECT_FALSE(s.epoch_health.empty);
  EXPECT_GT(s.great_circle_km, 0.0);
  EXPECT_EQ(s.t_jd, 2461202.6);
  EXPECT_EQ(s.num_planes, c.num_planes);

  // Edges are present and validly typed.
  ASSERT_FALSE(s.edges.empty());
  for (const auto& e : s.edges) {
    EXPECT_LT(e.a, e.b);
    EXPECT_TRUE(e.kind == Snapshot::Edge::kInPlane ||
                e.kind == Snapshot::Edge::kCrossPlane ||
                e.kind == Snapshot::Edge::kUplink);
  }

  // If a path was found, it spans the two endpoints.
  if (s.found) {
    ASSERT_FALSE(s.path.empty());
    EXPECT_EQ(s.path.front(), s.src_node);
    EXPECT_EQ(s.path.back(), s.dst_node);
  }

  // Re-query: endpoints swap, no leaked ground tail.
  sim.set_endpoints_geo(10.0, 20.0, -10.0, -20.0);
  EXPECT_EQ(c.ground_count(), 2u);
  EXPECT_EQ(c.node_count(), nodes);
}

// --- determinism -----------------------------------------------------------
//
// WHY THIS EXISTS: the `leo` CLI and the Vulkan monitor must answer the same
// question with the same route. Neither owns any routing logic -- both are thin
// wrappers that call Simulator::load / set_endpoints / step -- so proving the
// LIBRARY returns an identical Snapshot for identical inputs proves the two
// front-ends agree, without launching a renderer. Nothing in the pipeline draws
// on a clock, a random source, or a hash order, so the comparison is exact
// double equality, not a tolerance: a tolerance here would quietly pass the very
// nondeterminism the test is meant to catch.

// The instant every determinism case is pinned to. Close to the fixture's
// STARLINK-1008 epoch (2026-06-11), so SGP4 runs in its accurate regime rather
// than extrapolating years out.
constexpr double kFixedJd = 2461202.6;

// The two endpoints are placed relative to a satellite's own subsatellite point,
// so the test states what it means ("under the satellite" / "on the far side of
// the world") instead of hard-coding coordinates that only work by luck.
constexpr double kNearOffsetDeg = 2.0;  /* both endpoints see the same sat    */
constexpr leo::CatalogId kFixtureShellSat = 44714;  /* STARLINK-1008: the one
                                                       record whose epoch is
                                                       near kFixedJd           */

// -----------------------------------------------------------------------------
// Function:    subsatellite_point
// Description: The geocentric lat/lon directly beneath a satellite. Used to aim
//              the endpoints at real geometry rather than guessed coordinates.
// Input:       p        -- the satellite's ECEF position, km
//              lat_deg  -- receives the latitude, degrees
//              lon_deg  -- receives the longitude, degrees
// Outputs:     None; the two out-params are the result.
// -----------------------------------------------------------------------------
void subsatellite_point(const Vec3& p, double& lat_deg, double& lon_deg) {
  const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  constexpr double kRadToDeg = 57.29577951308232;  /* 180/pi                  */
  lat_deg = std::asin(p.z / r) * kRadToDeg;
  lon_deg = std::atan2(p.y, p.x) * kRadToDeg;
}

// -----------------------------------------------------------------------------
// Function:    route_once
// Description: One complete, independent run of the pipeline: a FRESH Simulator
//              loads the fixture, places the endpoints, and steps to kFixedJd.
//              A fresh instance (not a re-step of a warm one) is the point --
//              it rules out a cache or leftover state making two runs agree.
// Input:       src_lat, src_lon -- source endpoint, degrees
//              dst_lat, dst_lon -- destination endpoint, degrees
// Outputs:     The Snapshot for those inputs.
// -----------------------------------------------------------------------------
Snapshot route_once(double src_lat,
                    double src_lon,
                    double dst_lat,
                    double dst_lon) {
  Simulator sim;                                 /* a clean pipeline each time */
  EXPECT_TRUE(
      sim.load(fixture("test_omm.json"), fixture("test_stations.json"),
               ShellSpec{}));                    /* the fixed shell            */
  EXPECT_TRUE(sim.set_endpoints_geo(src_lat, src_lon, dst_lat, dst_lon));
  return sim.step(kFixedJd);
}

// -----------------------------------------------------------------------------
// Function:    expect_identical
// Description: Assert two Snapshots are the same ANSWER: same verdict, same
//              reason, same exact cost, same hop count, same ordered node path.
// Input:       a, b -- the two snapshots to compare
// Outputs:     None; failures are reported through gtest.
// -----------------------------------------------------------------------------
void expect_identical(const Snapshot& a, const Snapshot& b) {
  EXPECT_EQ(a.found, b.found);
  EXPECT_EQ(a.no_path_reason, b.no_path_reason);
  EXPECT_EQ(a.cost_s, b.cost_s);      // exact: deterministic, no RNG, no clock
  EXPECT_EQ(a.cost_ms, b.cost_ms);
  EXPECT_EQ(a.hops, b.hops);
  EXPECT_EQ(a.src_node, b.src_node);  // the ground tail must renumber the same
  EXPECT_EQ(a.dst_node, b.dst_node);
  EXPECT_EQ(a.path, b.path);          // the ordered NodeId sequence, hop by hop
}

// Identical inputs -> identical route, on the REACHABLE branch. Both endpoints
// are placed a couple of degrees either side of the same satellite's
// subsatellite point, so each can see it and the route is ground->sat->ground.
TEST(Simulator, DeterministicSnapshotForReachablePair) {
  // Probe run: where is the satellite at kFixedJd? Deriving the endpoints from
  // the propagated position keeps the test hermetic (the committed fixture is
  // the only input) without hard-coding a lat/lon that a fixture edit would
  // silently invalidate.
  Simulator probe;
  ASSERT_TRUE(probe.load(fixture("test_omm.json"),
                         fixture("test_stations.json"), ShellSpec{}));
  probe.set_endpoints_geo(0.0, 0.0, 0.0, 1.0);  // placeholder; only the sat matters
  probe.step(kFixedJd);
  const leo::Satellite* sat = probe.constellation().find(kFixtureShellSat);
  ASSERT_NE(sat, nullptr);
  double lat = 0.0, lon = 0.0;                   /* the subsatellite point     */
  subsatellite_point(probe.constellation().position_ecef[sat->index], lat, lon);

  const Snapshot a = route_once(lat, lon - kNearOffsetDeg,
                                lat, lon + kNearOffsetDeg);
  const Snapshot b = route_once(lat, lon - kNearOffsetDeg,
                                lat, lon + kNearOffsetDeg);

  ASSERT_TRUE(a.found) << "endpoints under the satellite should route";
  EXPECT_EQ(a.no_path_reason, Snapshot::NoPath::kReachable);
  EXPECT_GT(a.hops, 0u);
  expect_identical(a, b);
}

// The same guarantee on the NO-PATH branch: a source on the far side of the
// world from the only usable satellite is blind, and must be blind IDENTICALLY
// every run -- same reason code, same (empty) path -- so a front-end never
// reports "no route" one run and a route the next.
TEST(Simulator, DeterministicSnapshotForNoPathPair) {
  Simulator probe;
  ASSERT_TRUE(probe.load(fixture("test_omm.json"),
                         fixture("test_stations.json"), ShellSpec{}));
  probe.set_endpoints_geo(0.0, 0.0, 0.0, 1.0);
  probe.step(kFixedJd);
  const leo::Satellite* sat = probe.constellation().find(kFixtureShellSat);
  ASSERT_NE(sat, nullptr);
  double lat = 0.0, lon = 0.0;
  subsatellite_point(probe.constellation().position_ecef[sat->index], lat, lon);

  // Antipode of the subsatellite point: the satellite is below the horizon
  // there, so the source has no uplink at all.
  const double anti_lat = -lat;                  /* mirrored across the equator*/
  const double anti_lon = lon > 0.0 ? lon - 180.0 : lon + 180.0;

  const Snapshot a = route_once(anti_lat, anti_lon, lat, lon);
  const Snapshot b = route_once(anti_lat, anti_lon, lat, lon);

  ASSERT_FALSE(a.found) << "the antipodal source should see no satellite";
  EXPECT_TRUE(a.path.empty());
  expect_identical(a, b);
}

}  // namespace
