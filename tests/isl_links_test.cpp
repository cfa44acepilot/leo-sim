// isl_links_test.cpp -- cross-plane seeding with controlled, hand-set geometry.

#include "isl_links.hpp"

#include <gtest/gtest.h>

#include "constellation.hpp"
#include "topology.hpp"

namespace {

using leo::Constellation;
using leo::earth_clear;
using leo::INVALID_NODE;
using leo::LinkSpec;
using leo::NodeId;
using leo::Satellite;
using leo::seed_cross_plane_links;
using leo::ShellSpec;
using leo::Vec3;

// Add a shell satellite; only inclination/raan/mean_anomaly steer build_topology
// (incl fixed at 53 so all are kept; raan picks the plane, ma the slot).
void add(Constellation& c, leo::CatalogId cid, double raan, double ma) {
  Satellite s;
  s.catalog_id = cid;
  s.inclination = 53.0;
  s.raan = raan;
  s.mean_anomaly = ma;
  s.epoch_jd = 2461202.62;
  c.add_satellite(s);
}

NodeId node_of(const Constellation& c, leo::CatalogId cid) {
  const Satellite* s = c.find(cid);
  return s ? s->index : INVALID_NODE;
}

// Place a node (by catalog id) at an explicit ECEF position; all velocities
// default to +y so the direction filter passes unless a test overrides it.
void place(Constellation& c, leo::CatalogId cid, const Vec3& pos) {
  const NodeId i = node_of(c, cid);
  c.position_ecef[i] = pos;
  c.velocity_eci[i] = Vec3(0.0, 1.0, 0.0);
}

// --- earth_clear geometry, tested directly --------------------------------

TEST(IslLinks, EarthClearGrazingClearsAndDips) {
  // Symmetric pair about +y: closest approach to the origin is (0, y, 0).
  // y just above block_radius -> clears; just below -> blocked.
  EXPECT_TRUE(earth_clear(Vec3(2000, 6460, 0), Vec3(-2000, 6460, 0), 6458.0));
  EXPECT_FALSE(earth_clear(Vec3(2000, 6400, 0), Vec3(-2000, 6400, 0), 6458.0));
}

// --- seeding ---------------------------------------------------------------

// Three planes (RAAN 10/25/40). For a node in the middle plane, the lo-side
// (slot 2) and hi-side (slot 3) nearest neighbors are known by construction.
TEST(IslLinks, NearestPickFillsBothCrossPlaneSlots) {
  Constellation c;
  add(c, 100, 10.0, 0.0);    // plane 0
  add(c, 101, 10.0, 120.0);
  add(c, 102, 10.0, 240.0);
  add(c, 200, 25.0, 0.0);    // plane 1 (focal)
  add(c, 201, 25.0, 120.0);
  add(c, 300, 40.0, 0.0);    // plane 2
  add(c, 301, 40.0, 120.0);
  ASSERT_EQ(leo::build_topology(c, ShellSpec{}).num_planes, 3u);

  // Focal at +x; nearest in plane 0 is 101, nearest in plane 2 is 300.
  place(c, 200, Vec3(6900, 0, 0));
  place(c, 100, Vec3(6900, 500, 0));
  place(c, 101, Vec3(6900, 200, 0));   // closest on the lo side
  place(c, 102, Vec3(6900, 800, 0));
  place(c, 201, Vec3(6900, 50, 0));
  place(c, 300, Vec3(6900, -200, 0));  // closest on the hi side
  place(c, 301, Vec3(6900, -600, 0));

  seed_cross_plane_links(c);

  const NodeId f = node_of(c, 200);
  EXPECT_EQ(c.isl_neighbors[f][2], node_of(c, 101));  // lo side
  EXPECT_EQ(c.isl_neighbors[f][3], node_of(c, 300));  // hi side
  EXPECT_TRUE(c.verify());
}

// A candidate on the far side of Earth is occluded: link rejected even with the
// range filter opened up, isolating earth_clear as the cause.
TEST(IslLinks, OcclusionRejectsLink) {
  Constellation c;
  add(c, 1, 10.0, 0.0);   // plane 0
  add(c, 2, 25.0, 0.0);   // plane 1
  ASSERT_EQ(leo::build_topology(c, ShellSpec{}).num_planes, 2u);

  place(c, 1, Vec3(7000, 0, 0));
  place(c, 2, Vec3(-7000, 0, 0));  // opposite side; sight line crosses origin

  LinkSpec spec;
  spec.max_range_km = 20000.0;  // remove range as a confound
  seed_cross_plane_links(c, spec);

  // Two planes -> only slot 3 is ever seeded; it must be empty here.
  EXPECT_EQ(c.isl_neighbors[node_of(c, 1)][3], INVALID_NODE);
  EXPECT_TRUE(c.verify());
}

// A candidate just past max_range_km is rejected.
TEST(IslLinks, RangeRejectsLink) {
  Constellation c;
  add(c, 1, 10.0, 0.0);
  add(c, 2, 25.0, 0.0);
  ASSERT_EQ(leo::build_topology(c, ShellSpec{}).num_planes, 2u);

  place(c, 1, Vec3(6900, 0, 0));
  place(c, 2, Vec3(6900, 5410, 0));  // 5410 km away > default 5400 km

  seed_cross_plane_links(c);
  EXPECT_EQ(c.isl_neighbors[node_of(c, 1)][3], INVALID_NODE);
}

// A counter-moving candidate (opposite velocity) is rejected.
TEST(IslLinks, DirectionRejectsLink) {
  Constellation c;
  add(c, 1, 10.0, 0.0);
  add(c, 2, 25.0, 0.0);
  ASSERT_EQ(leo::build_topology(c, ShellSpec{}).num_planes, 2u);

  place(c, 1, Vec3(6900, 0, 0));
  place(c, 2, Vec3(6900, 200, 0));
  c.velocity_eci[node_of(c, 2)] = Vec3(0.0, -1.0, 0.0);  // opposite direction

  seed_cross_plane_links(c);
  EXPECT_EQ(c.isl_neighbors[node_of(c, 1)][3], INVALID_NODE);
}

// With a single plane there are no adjacent planes: slots 2,3 stay empty.
TEST(IslLinks, SinglePlaneHasNoCrossLinks) {
  Constellation c;
  add(c, 1, 10.0, 0.0);
  add(c, 2, 10.0, 120.0);
  add(c, 3, 10.0, 240.0);
  ASSERT_EQ(leo::build_topology(c, ShellSpec{}).num_planes, 1u);

  place(c, 1, Vec3(6900, 0, 0));
  place(c, 2, Vec3(6900, 200, 0));
  place(c, 3, Vec3(6900, 400, 0));

  seed_cross_plane_links(c);
  for (std::size_t i = 0; i < c.num_satellites; ++i) {
    EXPECT_EQ(c.isl_neighbors[i][2], INVALID_NODE);
    EXPECT_EQ(c.isl_neighbors[i][3], INVALID_NODE);
  }
  EXPECT_TRUE(c.verify());
}

// --- in-plane seeding ------------------------------------------------------
//
// These replace nothing: build_topology's slot-order ring is still asserted by
// topology_test as the INITIAL state. What is tested here is the per-tick
// geometric seeding that overwrites it, which is what the router actually uses.

using leo::seed_in_plane_links;

// The basic contract: two co-orbital satellites near each other link, and the
// link lands in slots 0/1 -- fore for the one ahead, aft for the one behind.
// Both fly +y, so 2 (at larger y) is AHEAD of 1.
TEST(IslLinks, CoOrbitalPairLinksInPlane) {
  Constellation c;
  add(c, 1, 10.0, 0.0);
  add(c, 2, 10.0, 120.0);
  ASSERT_EQ(leo::build_topology(c, ShellSpec{}).num_planes, 1u);

  place(c, 1, Vec3(6900, 0, 0));
  place(c, 2, Vec3(6900, 400, 0));  // 400 km ahead along +y, well within range

  seed_in_plane_links(c);

  const NodeId a = node_of(c, 1);
  const NodeId b = node_of(c, 2);
  EXPECT_EQ(c.isl_neighbors[a][0], b);  // 2 is ahead of 1  -> 1's fore slot
  EXPECT_EQ(c.isl_neighbors[b][1], a);  // 1 is behind 2    -> 2's aft slot
  EXPECT_TRUE(c.verify());
}

// A pair beyond max_range_km does not link, even though it is the only same-group
// candidate there is -- the whole point of the fix is that the seeded links are
// physically usable, so the router's gate never has to throw one away.
TEST(IslLinks, TooFarPairDoesNotLinkInPlane) {
  Constellation c;
  add(c, 1, 10.0, 0.0);
  add(c, 2, 10.0, 120.0);
  ASSERT_EQ(leo::build_topology(c, ShellSpec{}).num_planes, 1u);

  place(c, 1, Vec3(6900, 0, 0));
  place(c, 2, Vec3(6900, 3100, 0));  // 3100 km > default max_range 3000 km

  seed_in_plane_links(c);

  const NodeId a = node_of(c, 1);
  EXPECT_EQ(c.isl_neighbors[a][0], INVALID_NODE);
  EXPECT_EQ(c.isl_neighbors[a][1], INVALID_NODE);
  EXPECT_TRUE(c.verify());
}

// A chain: each satellite spends one terminal forward and one back, so a middle
// satellite links to BOTH neighbors. Three co-orbital sats in a line along the
// direction of travel (+y) at 0 / 400 / 500 km.
TEST(IslLinks, MutualPairingChainsAlongTrack) {
  Constellation c;
  add(c, 1, 10.0, 0.0);
  add(c, 2, 10.0, 120.0);
  add(c, 3, 10.0, 240.0);
  ASSERT_EQ(leo::build_topology(c, ShellSpec{}).num_planes, 1u);

  place(c, 1, Vec3(6900, 0, 0));
  place(c, 2, Vec3(6900, 400, 0));
  place(c, 3, Vec3(6900, 500, 0));

  seed_in_plane_links(c);

  const NodeId a = node_of(c, 1);
  const NodeId b = node_of(c, 2);
  const NodeId d = node_of(c, 3);
  EXPECT_EQ(c.isl_neighbors[a][0], b);  // 1 -fore-> 2
  EXPECT_EQ(c.isl_neighbors[b][1], a);  // 2 -aft--> 1  (mutual)
  EXPECT_EQ(c.isl_neighbors[b][0], d);  // 2 -fore-> 3
  EXPECT_EQ(c.isl_neighbors[d][1], b);  // 3 -aft--> 2  (mutual)
  EXPECT_TRUE(c.verify());
}

// MUTUAL pairing with the terminal cap, and the squeeze-out it implies. The
// squeeze needs a geometry where DISTANCE order differs from ALONG-TRACK order,
// which is the normal case in orbit (and impossible to see in a straight line).
//
// All fly +y. A is at the origin of the group; B is 400 km ahead along-track; D
// is 350 km ahead along-track but 300 km off to the side, so:
//     A->B = 400, A->D = 461, B->D = 304.
// A's nearest neighbor AHEAD is therefore B. But B's nearest neighbor BEHIND is
// D (304 < 400), not A -- so B never points back, and A is left with NO in-plane
// link despite having a feasible neighbor 400 km away. One-to-one laser terminals
// really do strand a satellite in a crowd; the router must see that, not a link
// that cannot exist.
TEST(IslLinks, MutualPairingSqueezesOutTheThirdSatellite) {
  Constellation c;
  add(c, 1, 10.0, 0.0);
  add(c, 2, 10.0, 120.0);
  add(c, 3, 10.0, 240.0);
  ASSERT_EQ(leo::build_topology(c, ShellSpec{}).num_planes, 1u);

  place(c, 1, Vec3(6900, 0, 0));      // A
  place(c, 2, Vec3(6900, 400, 0));    // B: 400 km ahead of A
  place(c, 3, Vec3(6900, 350, 300));  // D: nearer to B than A is

  seed_in_plane_links(c);

  const NodeId a = node_of(c, 1);
  const NodeId b = node_of(c, 2);
  const NodeId d = node_of(c, 3);
  EXPECT_EQ(c.isl_neighbors[b][1], d);  // B's aft goes to D, the closer one
  EXPECT_EQ(c.isl_neighbors[d][0], b);  // and D points back  (mutual)
  EXPECT_EQ(c.isl_neighbors[a][0], INVALID_NODE);  // A's pick was not returned
  EXPECT_EQ(c.isl_neighbors[a][1], INVALID_NODE);  // nothing behind A at all

  // Degree cap: no satellite holds more than two in-plane terminals -- true by
  // construction (one fore, one aft), asserted here so a future rewrite cannot
  // quietly break it.
  for (std::size_t i = 0; i < c.num_satellites; ++i) {
    int degree = 0;
    for (int s = 0; s < 2; ++s)
      if (c.isl_neighbors[i][s] != INVALID_NODE) ++degree;
    EXPECT_LE(degree, 2);
  }
  EXPECT_TRUE(c.verify());
}

// Seeding is idempotent and OVERWRITES build_topology's ring: a satellite the
// ring wired to a far-away slot neighbor ends up with no in-plane link once the
// geometry says there is none. This is the bug from instructions41 in miniature.
TEST(IslLinks, SeedingOverwritesTheSlotOrderRing) {
  Constellation c;
  add(c, 1, 10.0, 0.0);
  add(c, 2, 10.0, 180.0);
  ASSERT_EQ(leo::build_topology(c, ShellSpec{}).num_planes, 1u);

  const NodeId a = node_of(c, 1);
  const NodeId b = node_of(c, 2);
  // build_topology rings the plane, so the two are slot-wired to each other...
  ASSERT_EQ(c.isl_neighbors[a][0], b);

  // ...but they are on opposite sides of the Earth: far apart and occluded.
  place(c, 1, Vec3(6900, 0, 0));
  place(c, 2, Vec3(-6900, 0, 0));

  seed_in_plane_links(c);

  EXPECT_EQ(c.isl_neighbors[a][0], INVALID_NODE);  // the ring link is gone
  EXPECT_EQ(c.isl_neighbors[a][1], INVALID_NODE);
  EXPECT_EQ(c.isl_neighbors[b][0], INVALID_NODE);
  EXPECT_EQ(c.isl_neighbors[b][1], INVALID_NODE);
  seed_in_plane_links(c);                          // idempotent: same result
  EXPECT_EQ(c.isl_neighbors[a][0], INVALID_NODE);
  EXPECT_TRUE(c.verify());
}

}  // namespace
