// epoch_health.hpp
//
// Snapshot freshness gate. OMM/TLE elements are only trustworthy near their
// epoch: plane clustering needs all epochs tightly grouped, and SGP4-style
// propagation degrades as the newest element ages. This module measures both
// from the loaded Constellation so a caller can refuse to propagate (or warn)
// on stale data instead of silently producing garbage. Intended to run BEFORE
// any propagation, and is also the general form of the spread warning that
// build_topology emits.

#ifndef LEO_EPOCH_HEALTH_HPP
#define LEO_EPOCH_HEALTH_HPP

#include "constellation.hpp"

namespace leo {

// Thresholds for the two age-sensitive stages. Defaults match what we use
// elsewhere: tight intra-snapshot spread for clustering, ~1 week of usable
// accuracy for propagation.
struct EpochPolicy {
  double max_cluster_spread_days = 0.25;
  double max_propagation_age_days = 7.0;
};

// Measured freshness plus go/no-go flags. Ages are in days.
struct EpochHealth {
  bool empty = true;             // no satellites to measure
  double min_jd = 0.0;
  double max_jd = 0.0;
  double spread_days = 0.0;      // max_jd - min_jd (intra-snapshot coherence)
  double newest_age_days = 0.0;  // reference_jd - max_jd (age of freshest)
  bool cluster_ok = false;       // spread small enough to cluster planes
  bool propagation_ok = false;   // newest element fresh enough to propagate
};

// Julian Date for "now" in UTC from the system clock, so age checks don't have
// to thread a clock through every layer. Leap seconds are ignored (TLE epochs
// are UTC and this is a coarse staleness check, not a time standard).
double now_jd_utc();

// Summarize every satellite's epoch_jd against reference_jd -- usually
// now_jd_utc(), or the planned propagation start epoch. On an empty
// constellation both flags stay false (nothing to trust).
EpochHealth check_epoch_health(const Constellation& c, double reference_jd,
                               const EpochPolicy& policy = {});

}  // namespace leo

#endif  // LEO_EPOCH_HEALTH_HPP
