// epoch_health_test.cpp -- snapshot freshness measurement and go/no-go gating.

#include "epoch_health.hpp"

#include <gtest/gtest.h>

#include "constellation.hpp"

namespace {

using leo::check_epoch_health;
using leo::Constellation;
using leo::EpochHealth;
using leo::EpochPolicy;
using leo::now_jd_utc;
using leo::Satellite;

// A satellite carrying only the epoch we want to test.
Satellite at_epoch(leo::CatalogId cid, double epoch_jd) {
  Satellite s;
  s.catalog_id = cid;
  s.epoch_jd = epoch_jd;
  return s;
}

constexpr double kJ2000 = 2451545.0;

// A tight, recent snapshot passes both gates.
TEST(EpochHealth, FreshTightSnapshotIsHealthy) {
  Constellation c;
  c.add_satellite(at_epoch(1, kJ2000));
  c.add_satellite(at_epoch(2, kJ2000 + 0.1));  // 2.4 h spread

  const EpochHealth h = check_epoch_health(c, kJ2000 + 1.0);  // 1 day old
  EXPECT_FALSE(h.empty);
  EXPECT_DOUBLE_EQ(h.min_jd, kJ2000);
  EXPECT_DOUBLE_EQ(h.max_jd, kJ2000 + 0.1);
  EXPECT_NEAR(h.spread_days, 0.1, 1e-9);
  EXPECT_NEAR(h.newest_age_days, 0.9, 1e-9);
  EXPECT_TRUE(h.cluster_ok);
  EXPECT_TRUE(h.propagation_ok);
}

// A wide epoch spread fails clustering but, if recent, can still propagate --
// this is exactly the cached starlink.json situation.
TEST(EpochHealth, WideSpreadFailsClusterButMayPropagate) {
  Constellation c;
  c.add_satellite(at_epoch(1, kJ2000));
  c.add_satellite(at_epoch(2, kJ2000 + 5.0));  // 5-day spread

  const EpochHealth h = check_epoch_health(c, kJ2000 + 6.0);  // newest 1d old
  EXPECT_NEAR(h.spread_days, 5.0, 1e-9);
  EXPECT_FALSE(h.cluster_ok);     // spread > 0.25 day
  EXPECT_TRUE(h.propagation_ok);  // newest element still fresh
}

// An old newest-epoch fails the propagation gate even if tightly grouped.
TEST(EpochHealth, StaleSnapshotFailsPropagation) {
  Constellation c;
  c.add_satellite(at_epoch(1, kJ2000));
  c.add_satellite(at_epoch(2, kJ2000 + 0.05));

  const EpochHealth h = check_epoch_health(c, kJ2000 + 10.0);  // 10 days old
  EXPECT_NEAR(h.newest_age_days, 9.95, 1e-9);
  EXPECT_TRUE(h.cluster_ok);
  EXPECT_FALSE(h.propagation_ok);  // age > 7 days
}

// Thresholds are configurable.
TEST(EpochHealth, PolicyThresholdsAreHonored) {
  Constellation c;
  c.add_satellite(at_epoch(1, kJ2000));
  c.add_satellite(at_epoch(2, kJ2000 + 0.5));

  EpochPolicy lenient;
  lenient.max_cluster_spread_days = 1.0;       // allow the 0.5-day spread
  lenient.max_propagation_age_days = 30.0;     // allow a month-old element
  const EpochHealth h = check_epoch_health(c, kJ2000 + 20.0, lenient);
  EXPECT_TRUE(h.cluster_ok);
  EXPECT_TRUE(h.propagation_ok);
}

// An empty constellation has nothing to trust: both flags stay false.
TEST(EpochHealth, EmptyConstellationIsNotHealthy) {
  Constellation c;
  const EpochHealth h = check_epoch_health(c, kJ2000);
  EXPECT_TRUE(h.empty);
  EXPECT_FALSE(h.cluster_ok);
  EXPECT_FALSE(h.propagation_ok);
}

// now_jd_utc() should land in a sane modern range (between 2020 and 2100).
TEST(EpochHealth, NowJdIsInModernRange) {
  const double now = now_jd_utc();
  EXPECT_GT(now, 2458849.5);  // 2020-01-01
  EXPECT_LT(now, 2488069.5);  // 2100-01-01
}

}  // namespace
