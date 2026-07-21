// epoch_health.cpp -- measure snapshot freshness against a reference epoch.

#include "epoch_health.hpp"

#include <algorithm>
#include <chrono>

namespace leo {

double now_jd_utc() {
  // The Unix epoch 1970-01-01T00:00:00Z is JD 2440587.5; add elapsed days.
  using namespace std::chrono;
  const double secs =
      duration_cast<duration<double>>(system_clock::now().time_since_epoch())
          .count();
  return 2440587.5 + secs / 86400.0;
}

EpochHealth check_epoch_health(const Constellation& c, double reference_jd,
                               const EpochPolicy& policy) {
  EpochHealth h;

  // Scan the cold records; min/max of epoch_jd are all the inputs we need.
  bool seen = false;
  for (const auto& kv : c.by_catalog) {
    const double e = kv.second.epoch_jd;
    if (!seen) {
      h.min_jd = e;
      h.max_jd = e;
      seen = true;
    } else {
      h.min_jd = std::min(h.min_jd, e);
      h.max_jd = std::max(h.max_jd, e);
    }
  }

  h.empty = !seen;
  if (h.empty) return h;  // nothing to measure; both flags stay false

  h.spread_days = h.max_jd - h.min_jd;
  h.newest_age_days = reference_jd - h.max_jd;
  h.cluster_ok = h.spread_days <= policy.max_cluster_spread_days;
  // A negative age (epoch in the future of the reference) is still fine to
  // propagate; only an element older than the budget is rejected.
  h.propagation_ok = h.newest_age_days <= policy.max_propagation_age_days;
  return h;
}

}  // namespace leo
