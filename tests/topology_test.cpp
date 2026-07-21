// topology_test.cpp -- lattice scaffold from synthetic, controlled elements.

#include "topology.hpp"

#include <cmath>  // std::fabs (raan_dot magnitude assertions)

#include <gtest/gtest.h>

#include "constellation.hpp"
#include "isl_links.hpp"  // seed_cross_plane_links (grid-exists assertion)

namespace {

using leo::Constellation;
using leo::INVALID_NODE;
using leo::NodeId;
using leo::Satellite;
using leo::seed_cross_plane_links;
using leo::ShellSpec;
using leo::Vec3;

// Synthetic satellite with just the fields build_topology reads. A shared epoch
// keeps all keepers inside the coherence window (no spurious warning).
Satellite make_sat(leo::CatalogId cid, double incl, double raan, double ma) {
  Satellite s;
  s.catalog_id = cid;
  s.inclination = incl;
  s.raan = raan;
  s.mean_anomaly = ma;
  s.epoch_jd = 2461202.62;
  return s;
}

// Two planes 15 deg apart, 3 satellites each with scrambled mean anomalies; the
// build must sort each plane into slot order by mean anomaly.
TEST(Topology, TwoPlanesSortBySlotByMeanAnomaly) {
  Constellation c;
  // Plane at RAAN 10: catalog 1/2/3 with MA 200/50/120 -> slot order 2,3,1.
  c.add_satellite(make_sat(1, 53.0, 10.0, 200.0));
  c.add_satellite(make_sat(2, 53.0, 10.0, 50.0));
  c.add_satellite(make_sat(3, 53.0, 10.0, 120.0));
  // Plane at RAAN 25: catalog 4/5/6 with MA 300/10/150 -> slot order 5,6,4.
  c.add_satellite(make_sat(4, 53.0, 25.0, 300.0));
  c.add_satellite(make_sat(5, 53.0, 25.0, 10.0));
  c.add_satellite(make_sat(6, 53.0, 25.0, 150.0));

  const leo::BuildReport rep = leo::build_topology(c, ShellSpec{});
  EXPECT_EQ(rep.kept, 6u);
  EXPECT_EQ(rep.dropped, 0u);
  EXPECT_EQ(rep.num_planes, 2u);
  EXPECT_EQ(c.num_planes, 2u);

  // plane_offsets brackets two equal planes of three.
  ASSERT_EQ(c.plane_offsets.size(), 3u);
  EXPECT_EQ(c.plane_offsets[0], 0u);
  EXPECT_EQ(c.plane_offsets[1], 3u);
  EXPECT_EQ(c.plane_offsets[2], 6u);

  // Plane 0 (RAAN 10), ascending MA: cat 2 (50), 3 (120), 1 (200).
  EXPECT_EQ(c.catalog_at(0), 2u);
  EXPECT_EQ(c.catalog_at(1), 3u);
  EXPECT_EQ(c.catalog_at(2), 1u);
  // Plane 1 (RAAN 25), ascending MA: cat 5 (10), 6 (150), 4 (300).
  EXPECT_EQ(c.catalog_at(3), 5u);
  EXPECT_EQ(c.catalog_at(4), 6u);
  EXPECT_EQ(c.catalog_at(5), 4u);

  // grid_coord plane/slot for a couple of representative rows.
  EXPECT_EQ(c.grid_coord[0].plane, 0u);
  EXPECT_EQ(c.grid_coord[0].slot, 0u);
  EXPECT_EQ(c.grid_coord[2].plane, 0u);
  EXPECT_EQ(c.grid_coord[2].slot, 2u);
  EXPECT_EQ(c.grid_coord[3].plane, 1u);
  EXPECT_EQ(c.grid_coord[3].slot, 0u);
  EXPECT_EQ(c.grid_coord[5].plane, 1u);
  EXPECT_EQ(c.grid_coord[5].slot, 2u);

  // In-plane wrap: the last slot's fore is the plane's FIRST node, never the
  // next plane's first node.
  EXPECT_EQ(c.isl_neighbors[2][0], 0u);  // plane 0 last -> plane 0 first
  EXPECT_NE(c.isl_neighbors[2][0], 3u);  // NOT plane 1's first node
  EXPECT_EQ(c.isl_neighbors[0][1], 2u);  // plane 0 first aft -> plane 0 last
  EXPECT_EQ(c.isl_neighbors[5][0], 3u);  // plane 1 last fore -> plane 1 first

  // Interior fore/aft are the immediate neighbors.
  EXPECT_EQ(c.isl_neighbors[1][0], 2u);
  EXPECT_EQ(c.isl_neighbors[1][1], 0u);

  // Cross-plane terminals stay empty; bridge round-trips; whole thing verifies.
  for (std::size_t i = 0; i < c.num_satellites; ++i) {
    EXPECT_EQ(c.isl_neighbors[i][2], INVALID_NODE);
    EXPECT_EQ(c.isl_neighbors[i][3], INVALID_NODE);
    EXPECT_EQ(c.by_catalog.at(c.catalog_at(static_cast<NodeId>(i))).index, i);
  }
  EXPECT_TRUE(c.verify());
}

// A satellite outside the inclination band is dropped and erased from the map.
TEST(Topology, DropsOutOfShellSatellite) {
  Constellation c;
  c.add_satellite(make_sat(1, 53.0, 10.0, 0.0));
  c.add_satellite(make_sat(2, 53.0, 10.0, 90.0));
  c.add_satellite(make_sat(99, 70.0, 10.0, 0.0));  // polar shell -> dropped

  const leo::BuildReport rep = leo::build_topology(c, ShellSpec{});
  EXPECT_EQ(rep.dropped, 1u);
  EXPECT_EQ(rep.kept, 2u);
  EXPECT_EQ(c.node_count(), 2u);
  EXPECT_EQ(c.satellite_count(), 2u);
  EXPECT_EQ(c.find(99), nullptr);            // gone from the structure
  EXPECT_FALSE(c.by_catalog.contains(99u));  // and from the cold map
  EXPECT_TRUE(c.verify());
}

// RAAN is circular: satellites at 358 and 2 deg are 4 deg apart across 0, so a
// 5 deg gap must keep them in ONE plane, not split them.
TEST(Topology, StraddleZeroStaysOnePlane) {
  Constellation c;
  c.add_satellite(make_sat(1, 53.0, 358.0, 10.0));
  c.add_satellite(make_sat(2, 53.0, 0.0, 20.0));
  c.add_satellite(make_sat(3, 53.0, 2.0, 30.0));

  const leo::BuildReport rep = leo::build_topology(c, ShellSpec{});
  EXPECT_EQ(rep.num_planes, 1u);
  EXPECT_EQ(c.num_planes, 1u);
  ASSERT_EQ(c.plane_offsets.size(), 2u);
  EXPECT_EQ(c.plane_offsets[0], 0u);
  EXPECT_EQ(c.plane_offsets[1], 3u);
  for (std::size_t i = 0; i < c.num_satellites; ++i) {
    EXPECT_EQ(c.grid_coord[i].plane, 0u);
  }
  EXPECT_TRUE(c.verify());
}

// A lone satellite in a plane has no in-plane neighbor; a 2-satellite plane has
// fore == aft (the single other node).
TEST(Topology, PlaneSizeOneAndTwoNeighbors) {
  Constellation c;
  c.add_satellite(make_sat(1, 53.0, 10.0, 0.0));    // plane 0: size 1
  c.add_satellite(make_sat(2, 53.0, 100.0, 0.0));   // plane 1: size 2
  c.add_satellite(make_sat(3, 53.0, 100.0, 30.0));

  const leo::BuildReport rep = leo::build_topology(c, ShellSpec{});
  EXPECT_EQ(rep.num_planes, 2u);
  ASSERT_EQ(c.plane_offsets.size(), 3u);
  EXPECT_EQ(c.plane_offsets[1], 1u);  // plane 0 ends after one node
  EXPECT_EQ(c.plane_offsets[2], 3u);

  // Size-1 plane (node 0): both in-plane slots stay empty.
  EXPECT_EQ(c.isl_neighbors[0][0], INVALID_NODE);
  EXPECT_EQ(c.isl_neighbors[0][1], INVALID_NODE);

  // Size-2 plane (nodes 1,2): fore and aft are the same other node.
  EXPECT_EQ(c.isl_neighbors[1][0], 2u);
  EXPECT_EQ(c.isl_neighbors[1][1], 2u);
  EXPECT_EQ(c.isl_neighbors[2][0], 1u);
  EXPECT_EQ(c.isl_neighbors[2][1], 1u);

  // Cross-plane slots still empty everywhere.
  for (std::size_t i = 0; i < c.num_satellites; ++i) {
    EXPECT_EQ(c.isl_neighbors[i][2], INVALID_NODE);
    EXPECT_EQ(c.isl_neighbors[i][3], INVALID_NODE);
  }
  EXPECT_TRUE(c.verify());
}

// A ShellSpec that matches nothing must not crash: the filter dropping every
// satellite is a legitimate outcome that has to leave a sane, empty, verifiable
// constellation. Pins the zero-keeper guard in build_topology.
TEST(Topology, AllSatellitesDroppedLeavesEmptyValidState) {
  Constellation c;
  c.add_satellite(make_sat(1, 53.0, 10.0, 0.0));
  c.add_satellite(make_sat(2, 53.0, 10.0, 90.0));
  c.add_satellite(make_sat(3, 53.0, 25.0, 0.0));

  ShellSpec polar;
  polar.inclination_deg = 80.0;  // none of the 53 deg sats fall in this band

  const leo::BuildReport rep = leo::build_topology(c, polar);
  EXPECT_EQ(rep.kept, 0u);
  EXPECT_EQ(rep.dropped, 3u);
  EXPECT_EQ(rep.num_planes, 0u);
  EXPECT_EQ(c.num_satellites, 0u);
  EXPECT_EQ(c.node_count(), 0u);
  EXPECT_EQ(c.num_planes, 0u);
  EXPECT_TRUE(c.plane_offsets.empty());
  EXPECT_TRUE(c.by_catalog.empty());
  EXPECT_TRUE(c.verify());  // must not abort
}

// Fixed-plane-count binning (the default) snaps strays to their nearest plane,
// reports the right plane count on densely-spaced data, and yields a real grid
// that cross-plane seeding can populate -- the case the gap method collapses.
TEST(Topology, FixedPlaneBinningSnapsStraysAndBuildsGrid) {
  Constellation c;
  // Five planes at realistic ~5 deg RAAN spacing, three satellites each.
  for (int p = 0; p < 5; ++p) {
    const double raan = p * 5.0;
    for (int j = 0; j < 3; ++j) {
      c.add_satellite(make_sat(100 + p * 10 + j, 53.0, raan, j * 120.0));
    }
  }
  // Strays at in-between RAANs; each must snap to its nearest nominal plane,
  // not spawn a junk one-satellite plane.
  c.add_satellite(make_sat(900, 53.0, 2.0, 60.0));   // nearest plane RAAN 0
  c.add_satellite(make_sat(901, 53.0, 6.0, 60.0));   // nearest plane RAAN 5
  c.add_satellite(make_sat(902, 53.0, 12.0, 60.0));  // nearest plane RAAN 10

  const leo::BuildReport rep = leo::build_topology(c, ShellSpec{});  // 72 planes
  EXPECT_EQ(rep.kept, 18u);
  EXPECT_EQ(rep.num_planes, 5u);  // five occupied bins: not 1, not inflated
  EXPECT_EQ(c.num_planes, 5u);

  // The RAAN-2 stray shares a plane with the RAAN-0 satellites.
  const NodeId stray = c.find(900)->index;
  const NodeId plane0_sat = c.find(100)->index;
  EXPECT_EQ(c.grid_coord[stray].plane, c.grid_coord[plane0_sat].plane);

  // Positions/velocities chosen so adjacent-plane neighbors are in range, in
  // line of sight, and co-moving. Set AFTER build (it reorders the arrays).
  for (std::size_t i = 0; i < c.num_satellites; ++i) {
    const double y = c.grid_coord[i].plane * 30.0;
    const double z = c.grid_coord[i].slot * 5.0;
    c.position_ecef[i] = Vec3(6900.0, y, z);
    c.velocity_eci[i] = Vec3(0.0, 1.0, 0.0);
  }

  seed_cross_plane_links(c);

  // A real multi-plane grid lets cross-plane terminals (slots 2/3) fill --
  // impossible when everything collapses into one plane.
  std::size_t cross = 0;
  for (std::size_t i = 0; i < c.num_satellites; ++i) {
    if (c.isl_neighbors[i][2] != INVALID_NODE) ++cross;
    if (c.isl_neighbors[i][3] != INVALID_NODE) ++cross;
  }
  EXPECT_GT(cross, 0u);
  EXPECT_TRUE(c.verify());
}

// Synthetic satellite carrying the extra elements raan_dot needs (mean motion,
// eccentricity) plus an explicit epoch, so a precession test can space epochs
// apart. Mirrors make_sat but for the correction path.
leo::Satellite make_orbit_sat(leo::CatalogId cid, double incl, double raan,
                              double ma, double mean_motion, double ecc,
                              double epoch_jd) {
  leo::Satellite s;
  s.catalog_id = cid;
  s.inclination = incl;
  s.raan = raan;
  s.mean_anomaly = ma;
  s.mean_motion = mean_motion;
  s.eccentricity = ecc;
  s.epoch_jd = epoch_jd;
  return s;
}

// raan_dot must be NEGATIVE (nodal regression for a prograde orbit) and about
// -4.5 deg/day for a Starlink 53 deg shell -- this pins the formula magnitude and
// sign independently of the clustering path that consumes it.
TEST(Topology, RaanDotIsNegativeAndStarlinkMagnitude) {
  const double omega_dot = leo::raan_dot(15.05, 53.0, 0.0);  // rev/day, deg, e
  EXPECT_LT(omega_dot, 0.0);           // prograde -> RAAN regresses
  EXPECT_NEAR(omega_dot, -4.5, 0.5);   // published Starlink rate ~ -4.6 deg/day
  // Cranking inclination toward 90 deg drives cos(i) -> 0, so |Omega_dot| shrinks.
  EXPECT_LT(std::fabs(leo::raan_dot(15.05, 85.0, 0.0)), std::fabs(omega_dot));
}

// Sign-pin: two satellites in the SAME true orbit, observed 3 days apart, have
// raw RAANs that differ by the precession drift and split under raw binning. With
// the correction ON they must land in the SAME bin (co-orbital sats cluster
// TIGHTER, not looser -- the whole point). Built so raw_i = R - raan_dot*(t_ref -
// epoch_i), the exact inverse of the correction, so both project back to R.
TEST(Topology, PrecessionCorrectionClustersCoOrbitalSats) {
  constexpr double kRef = 2461202.62;  // reference (newest) epoch, JD
  constexpr double kR = 100.0;         // shared true RAAN at kRef, deg
  constexpr double kMeanMotion = 15.05;
  const double drift_per_day = leo::raan_dot(kMeanMotion, 53.0, 0.0);  // < 0

  // Sat A is at the reference epoch (raw == true RAAN). Sat B is 3 days older, so
  // its raw RAAN is R - raan_dot*3 (higher, since raan_dot < 0) -- a real orbit
  // seen at an earlier epoch. ~13.5 deg apart: several 5 deg bins.
  const double raw_b = kR - drift_per_day * 3.0;
  EXPECT_GT(raw_b, kR);  // sanity: older satellite reports a higher raw RAAN

  // RAW binning (correction OFF): the two spread across bins -> two planes.
  {
    Constellation c;
    c.add_satellite(make_orbit_sat(1, 53.0, kR, 0.0, kMeanMotion, 0.0, kRef));
    c.add_satellite(
        make_orbit_sat(2, 53.0, raw_b, 30.0, kMeanMotion, 0.0, kRef - 3.0));
    const leo::BuildReport rep = leo::build_topology(c, ShellSpec{});
    EXPECT_EQ(rep.num_planes, 2u);  // co-orbital pair wrongly split by raw RAAN
  }

  // CORRECTED binning (correction ON): both project back to R -> one plane.
  {
    Constellation c;
    c.add_satellite(make_orbit_sat(1, 53.0, kR, 0.0, kMeanMotion, 0.0, kRef));
    c.add_satellite(
        make_orbit_sat(2, 53.0, raw_b, 30.0, kMeanMotion, 0.0, kRef - 3.0));
    ShellSpec spec;
    spec.correct_raan_precession = true;
    const leo::BuildReport rep = leo::build_topology(c, spec);
    EXPECT_EQ(rep.num_planes, 1u);          // corrected RAAN reunites the orbit
    EXPECT_EQ(c.grid_coord[0].plane, c.grid_coord[1].plane);
  }
}

// Altitude band: two 53 deg satellites, one at ~550 km (mean motion 15.05
// rev/day) and one at ~1150 km (13.29 rev/day). Only altitude distinguishes
// them. With the band OFF (tol 0) both survive -> back-compat; with it ON both
// bands must pass, so the wrong-altitude satellite is dropped.
TEST(Topology, AltitudeBandDropsWrongAltitudeSat) {
  // tol 0 (default): filter disabled, both kept.
  {
    Constellation c;
    c.add_satellite(make_orbit_sat(1, 53.0, 10.0, 0.0, 15.05, 0.0, 2461202.62));
    c.add_satellite(make_orbit_sat(2, 53.0, 10.0, 90.0, 13.29, 0.0, 2461202.62));
    const leo::BuildReport rep = leo::build_topology(c, ShellSpec{});
    EXPECT_EQ(rep.kept, 2u);     // altitude filter off -> nothing dropped for it
    EXPECT_EQ(rep.dropped, 0u);
    EXPECT_TRUE(c.verify());
  }
  // tol 25 km around 550 km: the ~1150 km satellite fails the altitude band.
  {
    Constellation c;
    c.add_satellite(make_orbit_sat(1, 53.0, 10.0, 0.0, 15.05, 0.0, 2461202.62));
    c.add_satellite(make_orbit_sat(2, 53.0, 10.0, 90.0, 13.29, 0.0, 2461202.62));
    ShellSpec spec;
    spec.altitude_km = 550.0;     // shell-1 nominal altitude
    spec.altitude_tol_km = 25.0;  // enabling the band
    const leo::BuildReport rep = leo::build_topology(c, spec);
    EXPECT_EQ(rep.kept, 1u);          // only the ~550 km satellite survives
    EXPECT_EQ(rep.dropped, 1u);
    EXPECT_EQ(c.catalog_at(0), 1u);   // and it is cat 1, the in-band one
    EXPECT_TRUE(c.verify());
  }
}

// num_planes == 0 keeps the original gap-based clustering available.
TEST(Topology, GapModeFallbackWhenNumPlanesZero) {
  Constellation c;
  c.add_satellite(make_sat(1, 53.0, 10.0, 0.0));
  c.add_satellite(make_sat(2, 53.0, 25.0, 0.0));  // 15 deg gap > 5 deg threshold

  ShellSpec spec;
  spec.num_planes = 0;  // force the gap path
  const leo::BuildReport rep = leo::build_topology(c, spec);
  EXPECT_EQ(rep.num_planes, 2u);
  EXPECT_TRUE(c.verify());
}

}  // namespace
