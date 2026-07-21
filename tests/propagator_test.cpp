// propagator_test.cpp -- pins GMST, SGP4 units/epoch/frame against references.

#include "propagator.hpp"

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "constellation.hpp"

namespace {

using leo::Constellation;
using leo::gmst_rad;
using leo::NodeId;
using leo::Propagator;
using leo::Satellite;
using leo::Vec3;

double mag(const Vec3& v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double dist(const Vec3& a, const Vec3& b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                   (a.z - b.z) * (a.z - b.z));
}

NodeId add(Constellation& c, leo::CatalogId cid, double epoch_jd, double n,
           double ecc, double incl, double raan, double argp, double ma,
           double bstar, double ndot) {
  Satellite s;
  s.catalog_id = cid;
  s.epoch_jd = epoch_jd;
  s.mean_motion = n;        // rev/day
  s.eccentricity = ecc;
  s.inclination = incl;     // deg
  s.raan = raan;            // deg
  s.arg_pericenter = argp;  // deg
  s.mean_anomaly = ma;      // deg
  s.bstar = bstar;
  s.mean_motion_dot = ndot;
  s.mean_motion_ddot = 0.0;
  return c.add_satellite(s);
}

// GMST at J2000 (2000-01-01T12:00 UTC) is ~280.46 deg (~4.895 rad). Pins the
// sidereal-time formula independently of SGP4.
TEST(Propagator, GmstAtJ2000) {
  const double g = gmst_rad(2451545.0);
  EXPECT_NEAR(g, 4.894961, 1e-4);
  EXPECT_NEAR(g * 180.0 / std::numbers::pi, 280.4606, 1e-3);
}

// End-to-end SGP4 reference: Vallado SGP4-VER sat 00005 at tsince = 0 must match
// the published TEME position (from 00005.e) within ~1 km. This is the proof the
// unit conversions and epoch handling are correct.
TEST(Propagator, Sgp4ReferenceSat5) {
  Constellation c;
  // Sat 5 mean elements from SGP4-VER.TLE; epoch 2000-06-27T18:50:19.7336 UTC.
  const double epoch_jd = 2451723.28495062;
  add(c, 5, epoch_jd, 10.82419157, 0.1859667, 34.2682, 348.7242, 331.7664,
      19.3264, 0.000028098, 0.00000023);

  Propagator p;
  p.init(c);
  ASSERT_EQ(p.usable_count(), 1u);

  // tsince = 0: validates init and coordinate output (all time-dependent terms
  // vanish here). Reference from third_party/sgp4/reference/00005.e, t = 0 s.
  p.propagate(c, epoch_jd);
  const Vec3 expect_t0(7022.46529266, -1400.08296755, 0.03995155);  // km, TEME
  EXPECT_LT(dist(c.position_eci[0], expect_t0), 1.0);
  EXPECT_EQ(p.last_error_count(), 0u);

  // tsince = 360 min: exercises the time-dependent SGP4 terms that t=0 cannot.
  // Reference from 00005.e, t = 21600 s (= 360 min) row, verbatim digits.
  p.propagate(c, epoch_jd + 360.0 / 1440.0);
  const Vec3 expect_t360(-7154.03120202, -3783.17682504, -3536.19412294);
  EXPECT_LT(dist(c.position_eci[0], expect_t360), 1.0);
  EXPECT_EQ(p.last_error_count(), 0u);
}

// ECEF is a pure rotation of ECI, so magnitudes must match exactly.
TEST(Propagator, EcefIsRotationOfEci) {
  Constellation c;
  const double epoch_jd = 2451723.28495062;
  add(c, 5, epoch_jd, 10.82419157, 0.1859667, 34.2682, 348.7242, 331.7664,
      19.3264, 0.000028098, 0.00000023);

  Propagator p;
  p.init(c);
  p.propagate(c, epoch_jd + 0.3);  // arbitrary time so GMST != 0

  EXPECT_NEAR(mag(c.position_ecef[0]), mag(c.position_eci[0]), 1e-6);
}

// A Starlink record: at its own epoch the radius is a plausible LEO value, and
// after one orbital period the satellite is back near where it started.
TEST(Propagator, StarlinkRadiusAndPeriodicity) {
  Constellation c;
  // STARLINK-1008, epoch 2026-06-11T02:53:00.859776 UTC.
  const double epoch_jd = 2461202.6201488;
  const double n = 15.4932797;  // rev/day
  add(c, 44714, epoch_jd, n, 0.0002756, 53.1508, 97.7628, 106.4068, 253.7245,
      0.0010720968, 0.00059679);

  Propagator p;
  p.init(c);
  ASSERT_EQ(p.usable_count(), 1u);

  p.propagate(c, epoch_jd);
  const Vec3 start = c.position_eci[0];
  const double r = mag(start);
  EXPECT_GT(r, 6700.0);  // plausible LEO radius (km)
  EXPECT_LT(r, 7100.0);

  // One orbital period later, roughly back to the start (J2 precession and the
  // Kozai-vs-mean-motion difference leave a small gap, hence the loose bound).
  const double period_days = 1.0 / n;
  p.propagate(c, epoch_jd + period_days);
  EXPECT_LT(dist(c.position_eci[0], start), 300.0);
}

}  // namespace
