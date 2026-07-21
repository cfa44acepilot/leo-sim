// propagator.hpp
//
// SGP4-based propagator: advances every satellite to a simulation time (Julian
// Date), filling position_eci/velocity_eci (TEME, treated as ECI) and
// position_ecef. The SGP4 library type lives only in propagator.cpp -- it is
// hidden here behind a pImpl so SGP4.h never leaks into the rest of the build.
//
// Build/run order: load_omm_json -> build_topology -> Propagator::init(c) ->
// Propagator::propagate(c, t_jd). init() must run AFTER build_topology so the
// per-satellite SGP4 records line up with the final sorted (plane, slot) order.

#ifndef LEO_PROPAGATOR_HPP
#define LEO_PROPAGATOR_HPP

#include <cstddef>
#include <memory>

#include "constellation.hpp"

namespace leo {

// Greenwich Mean Sidereal Time (radians, in [0, 2pi)) for a UT1 Julian Date.
// UTC is used as UT1 (sub-second error; consistent with the sim ignoring leap
// seconds). Standalone so the J2000 unit test pins the sidereal-time formula.
double gmst_rad(double jd_ut1);

class Propagator {
 public:
  Propagator();
  ~Propagator();
  Propagator(Propagator&&) noexcept;
  Propagator& operator=(Propagator&&) noexcept;
  Propagator(const Propagator&) = delete;
  Propagator& operator=(const Propagator&) = delete;

  // Build one initialized SGP4 record per satellite in [0, satellite_count()),
  // reading cold elements via by_catalog and converting units. A satellite
  // whose sgp4init reports an error is marked unusable, not propagated.
  void init(const Constellation& c);

  // Advance every satellite to absolute time t_jd (each from its OWN epoch) and
  // write position_eci/velocity_eci (km, km/s, TEME) and position_ecef (km).
  // Unusable satellites and per-step SGP4 errors leave their state zeroed.
  void propagate(Constellation& c, double t_jd);

  std::size_t usable_count() const;      // satellites that initialized cleanly
  std::size_t last_error_count() const;  // sgp4 step errors in last propagate
  bool usable(NodeId i) const;           // did satellite i initialize cleanly

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace leo

#endif  // LEO_PROPAGATOR_HPP
