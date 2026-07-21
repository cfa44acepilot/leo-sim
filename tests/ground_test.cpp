// ground_test.cpp -- geodetic placement, ground-node append, uplink visibility.

#include "ground.hpp"

#include <algorithm>

#include <gtest/gtest.h>

#include "constellation.hpp"
#include "topology.hpp"

namespace {

using leo::connect_ground_uplinks;
using leo::Constellation;
using leo::geodetic_to_ecef;
using leo::INVALID_NODE;
using leo::NodeId;
using leo::Satellite;
using leo::ShellSpec;
using leo::Vec3;

void add_sat(Constellation& c, leo::CatalogId cid, double raan, double ma) {
  Satellite s;
  s.catalog_id = cid;
  s.inclination = 53.0;
  s.raan = raan;
  s.mean_anomaly = ma;
  s.epoch_jd = 2461202.62;
  c.add_satellite(s);
}

bool contains(const std::vector<NodeId>& v, NodeId x) {
  return std::find(v.begin(), v.end(), x) != v.end();
}

// Geodetic -> ECEF against hand-computed axis points.
TEST(Ground, GeodeticToEcefAxisPoints) {
  const double b = 6356.752;  // semi-minor axis a*sqrt(1-e2)
  const Vec3 eq0 = geodetic_to_ecef(0.0, 0.0, 0.0);
  EXPECT_NEAR(eq0.x, 6378.137, 1e-3);
  EXPECT_NEAR(eq0.y, 0.0, 1e-3);
  EXPECT_NEAR(eq0.z, 0.0, 1e-3);

  const Vec3 eq90 = geodetic_to_ecef(0.0, 90.0, 0.0);
  EXPECT_NEAR(eq90.x, 0.0, 1e-3);
  EXPECT_NEAR(eq90.y, 6378.137, 1e-3);
  EXPECT_NEAR(eq90.z, 0.0, 1e-3);

  const Vec3 pole = geodetic_to_ecef(90.0, 0.0, 0.0);
  EXPECT_NEAR(pole.x, 0.0, 1e-3);
  EXPECT_NEAR(pole.y, 0.0, 1e-3);
  EXPECT_NEAR(pole.z, b, 1e-3);
}

// add_ground_node lands in the tail without disturbing the satellite prefix.
TEST(Ground, AddGroundNodeAppendsToTail) {
  Constellation c;
  add_sat(c, 1, 10.0, 0.0);
  add_sat(c, 2, 25.0, 0.0);
  leo::build_topology(c, ShellSpec{});

  const std::size_t before = c.node_count();
  const std::size_t sats = c.satellite_count();
  const NodeId g = c.add_ground_node(Vec3(6378.137, 0, 0));

  EXPECT_EQ(g, static_cast<NodeId>(before));   // appended at the old end
  EXPECT_EQ(c.satellite_count(), sats);        // prefix untouched
  EXPECT_EQ(c.ground_count(), 1u);
  EXPECT_TRUE(c.is_ground(g));
  EXPECT_EQ(c.node_count(), before + 1);       // every array grew by one
  EXPECT_EQ(c.position_eci.size(), before + 1);
  EXPECT_EQ(c.catalog_id.size(), before + 1);

  for (int t = 0; t < leo::NUM_ISL_TERMINALS; ++t) {
    EXPECT_EQ(c.isl_neighbors[g][t], INVALID_NODE);  // no laser terminals
  }
  EXPECT_FALSE(c.by_catalog.contains(c.catalog_id[g]));  // catalog_id 0 sentinel
  EXPECT_TRUE(c.verify());
}

// Visibility from a station at (6378.137,0,0) where local up = (1,0,0).
TEST(Ground, UplinkElevationFilter) {
  Constellation c;
  add_sat(c, 1, 10.0, 0.0);   // overhead
  add_sat(c, 2, 25.0, 0.0);   // 30 deg elevation
  add_sat(c, 3, 40.0, 0.0);   // on the horizon (0 deg)
  add_sat(c, 4, 55.0, 0.0);   // below the horizon

  const NodeId g = c.add_ground_node(Vec3(6378.137, 0, 0));

  // Place satellites directly (no SGP4) for controlled geometry. Order added ==
  // node order (no build_topology), but use find() to be index-agnostic.
  const NodeId s1 = c.find(1)->index;
  const NodeId s2 = c.find(2)->index;
  const NodeId s3 = c.find(3)->index;
  const NodeId s4 = c.find(4)->index;
  c.position_ecef[s1] = Vec3(6928.137, 0, 0);    // straight up -> 90 deg
  c.position_ecef[s2] = Vec3(6878.137, 866, 0);  // to=(500,866,0) -> 30 deg
  c.position_ecef[s3] = Vec3(6378.137, 600, 0);  // to=(0,600,0) -> 0 deg
  c.position_ecef[s4] = Vec3(6278.137, 600, 0);  // to=(-100,600,0) -> <0 deg

  connect_ground_uplinks(c);  // default 25 deg minimum

  ASSERT_EQ(c.ground_uplinks.size(), 1u);
  const std::vector<NodeId>& up = c.ground_uplinks[0];
  EXPECT_TRUE(contains(up, s1));   // 90 deg
  EXPECT_TRUE(contains(up, s2));   // 30 deg >= 25
  EXPECT_FALSE(contains(up, s3));  // 0 deg
  EXPECT_FALSE(contains(up, s4));  // below horizon
  EXPECT_EQ(up.size(), 2u);
  EXPECT_TRUE(c.verify());

  // The ground node itself was not mistaken for a visible "satellite".
  EXPECT_FALSE(contains(up, g));
}

}  // namespace
