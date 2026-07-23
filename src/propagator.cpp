/*****************************************************************************
  filename propagator.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-22
  Brief Description:
    The only translation unit that sees the vendored Vallado SGP4. init() calls
    sgp4init directly with each satellite's mean elements (unit-converted exactly
    as twoline2rv), and propagate() advances to a Julian Date filling ECI (TEME)
    position/velocity, then rotates by GMST into ECEF. gmst_rad() lives here too.
 *****************************************************************************/
// propagator.cpp -- SGP4 propagation. The ONLY translation unit that sees SGP4.

#include "propagator.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <vector>

#include "SGP4.h"  // vendored Vallado reference; private to this file

namespace leo {

namespace {

constexpr double kDeg2Rad = std::numbers::pi / 180.0;
// Vallado's xpdotp = 1440(60 X 24) min/day / 2pi: converts rev/day <-> rad/min.
constexpr double kXpdotp = 1440.0 / (2.0 * std::numbers::pi);
// SGP4's epoch argument is days since 1949-12-31 00:00 UT == JD 2433281.5.
constexpr double kSgp4EpochJd = 2433281.5;

}  // namespace

double gmst_rad(double jd_ut1) {
  // IAU-82 GMST, copied from Vallado's gstime rather than calling it, so this
  // formula is independently pinned by the J2000 unit test.
  constexpr double twopi = 2.0 * std::numbers::pi;
  const double tut1 = (jd_ut1 - 2451545.0) / 36525.0;
  const double sec = -6.2e-6 * tut1 * tut1 * tut1 + 0.093104 * tut1 * tut1 +
                     (876600.0 * 3600.0 + 8640184.812866) * tut1 + 67310.54841;
  // 360 deg / 86400 s = 1/240 deg per second; convert to radians, wrap.
  double g = std::fmod(sec * kDeg2Rad / 240.0, twopi);
  if (g < 0.0) g += twopi;
  return g;
}

// elsetrec is large and SGP4-specific, so the whole store hides behind pImpl.
struct Propagator::Impl {
  std::vector<elsetrec> rec;     // one SGP4 record per NodeId
  std::vector<double> epoch_jd;  // each satellite's own epoch (JD)
  std::vector<char> ok;          // 1 if sgp4init succeeded for that satellite
  // Catalog id captured at init: these records are paired to NodeIds, which are
  // only stable if the constellation is not reordered after init(). Stored so
  // propagate() can assert the pairing still holds.
  std::vector<CatalogId> catalog_id;
  std::size_t usable = 0;
  std::size_t last_errors = 0;
};

Propagator::Propagator() : impl_(std::make_unique<Impl>()) {}
Propagator::~Propagator() = default;
Propagator::Propagator(Propagator&&) noexcept = default;
Propagator& Propagator::operator=(Propagator&&) noexcept = default;

void Propagator::init(const Constellation& c) {
  Impl& m = *impl_;
  const std::size_t n = c.satellite_count();
  m.rec.assign(n, elsetrec{});
  m.epoch_jd.assign(n, 0.0);
  m.ok.assign(n, 0);
  m.catalog_id.assign(n, 0);
  m.usable = 0;
  m.last_errors = 0;

  for (std::size_t i = 0; i < n; ++i) {
    const Satellite& s = c.by_catalog.at(c.catalog_id[i]);
    m.epoch_jd[i] = s.epoch_jd;
    m.catalog_id[i] = c.catalog_id[i];  // remember the pairing for propagate()

    // Unit conversion is the #1 SGP4 bug source. OMM stores degrees and
    // rev/day; SGP4 wants radians and rad/min, scaled EXACTLY as Vallado's
    // twoline2rv does before its own sgp4init call.
    const double no_kozai = s.mean_motion / kXpdotp;             // rad/min
    const double ndot = s.mean_motion_dot / (kXpdotp * 1440.0);  // rad/min^2
    const double nddot =
        s.mean_motion_ddot / (kXpdotp * 1440.0 * 1440.0);        // rad/min^3
    const double inclo = s.inclination * kDeg2Rad;
    const double nodeo = s.raan * kDeg2Rad;
    const double argpo = s.arg_pericenter * kDeg2Rad;
    const double mo = s.mean_anomaly * kDeg2Rad;
    const double epoch = s.epoch_jd - kSgp4EpochJd;  // days past 1949-12-31

    // satnum is only a label (unused in the math) and satrec.satnum is char[6];
    // keep it <= 5 chars so MSVC's strcpy_s inside sgp4init cannot overrun.
    char satn[6];
    std::snprintf(satn, sizeof(satn), "%u", s.catalog_id % 100000u);

    // no_kozai is the published (Kozai) mean motion; sgp4init un-Kozais it.
    const bool good = SGP4Funcs::sgp4init(wgs72, 'i', satn, epoch, s.bstar, ndot,
                                          nddot, s.eccentricity, argpo, inclo,
                                          mo, no_kozai, nodeo, m.rec[i]);
    // sgp4init runs one sgp4 step at t=0, so satrec.error reflects a bad set.
    m.ok[i] = (good && m.rec[i].error == 0) ? 1 : 0;
    if (m.ok[i]) ++m.usable;
  }
}

void Propagator::propagate(Constellation& c, double t_jd) {
  Impl& m = *impl_;
  const std::size_t n = m.rec.size();
  m.last_errors = 0;

  // The TEME->ECEF rotation angle is the same for every satellite at t_jd.
  const double theta = gmst_rad(t_jd);
  const double cg = std::cos(theta);
  const double sg = std::sin(theta);

  for (std::size_t i = 0; i < n; ++i) {
    // The records are paired to NodeIds from init(); a reorder in between would
    // silently propagate each node with another satellite's elements. Catch it.
    assert(c.catalog_id[i] == m.catalog_id[i]);

    if (!m.ok[i]) {
      c.position_eci[i] = Vec3(0.0);
      c.velocity_eci[i] = Vec3(0.0);
      c.position_ecef[i] = Vec3(0.0);
      continue;
    }

    // Each satellite advances from its OWN epoch; SGP4 tsince is in minutes.
    const double tsince = (t_jd - m.epoch_jd[i]) * 1440.0;
    double r[3], v[3];
    const bool good = SGP4Funcs::sgp4(m.rec[i], tsince, r, v);
    if (!good || m.rec[i].error != 0) {
      // A failed step (e.g. decayed) must not write garbage; zero and count it.
      c.position_eci[i] = Vec3(0.0);
      c.velocity_eci[i] = Vec3(0.0);
      c.position_ecef[i] = Vec3(0.0);
      ++m.last_errors;
      continue;
    }

    // TEME treated as ECI at this fidelity (the TEME->true-ECI rotation is
    // sub-arcsecond and irrelevant for visualization-grade work).
    c.position_eci[i] = Vec3(r[0], r[1], r[2]);  // km
    c.velocity_eci[i] = Vec3(v[0], v[1], v[2]);  // km/s

    // ECEF = ROT3(theta) * TEME: rotate about z by GMST. velocity_ecef is not
    // needed yet, so it is intentionally not produced.
    c.position_ecef[i] =
        Vec3(cg * r[0] + sg * r[1], -sg * r[0] + cg * r[1], r[2]);
  }
}

std::size_t Propagator::usable_count() const { return impl_->usable; }
std::size_t Propagator::last_error_count() const { return impl_->last_errors; }
bool Propagator::usable(NodeId i) const { return impl_->ok[i] != 0; }

}  // namespace leo
