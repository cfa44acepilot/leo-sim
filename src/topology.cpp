/*****************************************************************************
  filename topology.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-14
  Brief Description:
    Builds the static lattice scaffold from the published elements alone (no
    positions needed): filter to one shell, cluster into RAAN groups, sort by
    (plane, mean anomaly), and wire the initial in-plane ring.

    That ring is only the STARTING state. seed_in_plane_links overwrites it from
    real geometry every tick, so what is built here is a scaffold, not the graph
    anything routes on.
 *****************************************************************************/

#include "topology.hpp"

#include <algorithm>  /* sort/stable_partition: the shell layout           */
#include <cassert>    /* invariants the layout must not violate            */
#include <cmath>      /* cos/pow: the J2 nodal-regression rate             */
#include <cstdint>    /* fixed-width grid coordinates                      */
#include <numbers>    /* std::numbers::pi: degree/radian conversion        */
#include <numeric>    /* prefix sums for plane_offsets                     */

#include "epoch_health.hpp"  /* the shared cluster-spread threshold        */

namespace leo
{

namespace
{

// Shared physical constants. constexpr (typed) rather than #define per the
// style; the preprocessor is reserved for genuine ALL_UPPERCASE macros.
constexpr double J2 = 1.08262668e-3;       // Earth oblateness coefficient
constexpr double MU_KM3_S2 = 398600.4418;  // Earth GM, km^3/s^2
constexpr double RE_KM = 6378.137;         // Earth equatorial radius, km

// One shell candidate, snapshotting the cold elements we cluster/sort on so the
// algorithm never has to chase the hashmap again after the keepers are chosen.
// mean_motion/inclination/eccentricity feed raan_dot for the optional
// precession correction; they are otherwise unused by the raw path.
struct Keeper
{
  NodeId old_index;
  double raan;
  double mean_anomaly;
  double epoch_jd;
  double mean_motion;   // rev/day (OMM native); for raan_dot only
  double inclination;   // deg; for raan_dot only
  double eccentricity;  // dimensionless; for raan_dot only
};

// Semi-major axis (km) from mean motion (rev/day) via Kepler's third law,
// n^2 * a^3 = MU. OMM publishes mean motion in rev/day, so convert to rad/s
// first (one rev = 2*pi rad, one day = 86400 s). Shared by raan_dot and the
// altitude filter so the a-from-n math lives in exactly one place.
double semimajor_axis_km(double mean_motion_rev_day)
{
  const double n_rad_s = mean_motion_rev_day * 2.0 * std::numbers::pi / 86400.0;
  return std::cbrt(MU_KM3_S2 / (n_rad_s * n_rad_s));
}

}  // namespace

double raan_dot(double mean_motion_rev_day, double inclination_deg,
                double eccentricity)
{
  const double pi = std::numbers::pi;  // for deg<->rad and rev<->rad

  // The physics wants n in rad/s (same conversion semimajor_axis_km uses).
  const double n_rad_s = mean_motion_rev_day * 2.0 * pi / 86400.0;
  const double a_km = semimajor_axis_km(mean_motion_rev_day);  // Kepler, km
  const double i_rad = inclination_deg * pi / 180.0;  // cos(i) needs radians
  const double re_over_a = RE_KM / a_km;              // (Re/a) appears squared
  const double one_minus_e2 =
      1.0 - eccentricity * eccentricity;  // squared below

  // Omega_dot in rad/s. Negative for a prograde orbit (cos i > 0).
  const double omega_dot_rad_s = -1.5 * n_rad_s * J2 * re_over_a * re_over_a *
                                 std::cos(i_rad) /
                                 (one_minus_e2 * one_minus_e2);
  // Convert rad/s -> deg/day so it multiplies a Julian-day epoch delta
  // directly: rad->deg is (180/pi), s->day is 86400.
  return omega_dot_rad_s * (180.0 / pi) * 86400.0;
}

double orbit_altitude_km(double mean_motion_rev_day)
{
  // Semi-major axis minus Earth's equatorial radius = altitude of a circular
  // orbit. Reuses the shared Kepler helper so the a-from-n math lives once.
  return semimajor_axis_km(mean_motion_rev_day) - RE_KM;
}

/* ***************************************************************************
 * Function : shell_presets
 * Description : The built-in single-shell presets (Starlink Gen1), 53-deg main
 *               first so index 0 matches a default-constructed ShellSpec.
 * Input : none.
 * Outputs : const reference to a function-local static list of ShellPreset.
 * ************************************************************************* */
const std::vector<ShellPreset>& shell_presets()
{
  // Built once on first use. Plane counts are best PUBLIC ESTIMATES for the
  // Starlink Gen1 shells, not authoritative figures -- enough to lay out a
  // representative Walker lattice per shell.
  static const std::vector<ShellPreset> presets = {
      // 53-deg main shell. Index 0 and byte-identical to a default ShellSpec:
      // altitude filter OFF (tol 0), so this preset reproduces today's exact
      // behavior and every existing test. Its altitude population is smeared
      // (many lower deorbiting sats share 53 deg), so a band here would gut it.
      {"53 deg (main)", ShellSpec{53.0, 1.0, 5.0, 72, 550.0, 0.0, false}},
      // 70-deg shell. This inclination sits cleanly near ~570 km, so a +/-30 km
      // band isolates the real sub-shell while dropping only a handful of
      // off-altitude strays. ~36 planes (estimate).
      {"70 deg", ShellSpec{70.0, 1.0, 5.0, 36, 570.0, 30.0, false}},
      // 97.6-deg polar shell. RETROGRADE (incl > 90): raan_dot's cos(i) term
      // goes negative, so the J2 precession sign flips automatically -- the
      // formula uses cos(i) directly and needs no special case (precession
      // correction stays OFF here anyway). A +/-30 km band around 560 km
      // isolates it from co-inclination sats at other altitudes. ~6 planes.
      {"97.6 deg (polar)", ShellSpec{97.6, 1.0, 5.0, 6, 560.0, 30.0, false}},
  };
  return presets;
}

BuildReport build_topology(Constellation& c, const ShellSpec& spec)
{
  BuildReport report;

  // --- Step 1: select keepers by inclination (and optional altitude) band
  // -----
  std::vector<Keeper> keepers;
  keepers.reserve(c.num_satellites);
  std::vector<CatalogId> dropped_cids;
  for (std::size_t i = 0; i < c.num_satellites; ++i)
  {
    const CatalogId cid = c.catalog_id[i];
    const Satellite& sat = c.by_catalog.at(cid);
    // Inclination band: the primary shell selector, always applied.
    const bool incl_ok = std::fabs(sat.inclination - spec.inclination_deg) <=
                         spec.inclination_tol_deg;
    // Altitude band: only when enabled (tol > 0), reject other sub-shells that
    // share the inclination. Altitude comes from mean motion via Kepler, so no
    // extra data is needed. With tol == 0 alt_ok stays true -> byte-identical
    // to the inclination-only keeper set.
    bool alt_ok = true;
    if (spec.altitude_tol_km > 0.0)
    {
      const double altitude_km = semimajor_axis_km(sat.mean_motion) - RE_KM;
      alt_ok =
          std::fabs(altitude_km - spec.altitude_km) <= spec.altitude_tol_km;
    }
    if (incl_ok && alt_ok)
    {
      keepers.push_back({static_cast<NodeId>(i), sat.raan, sat.mean_anomaly,
                         sat.epoch_jd, sat.mean_motion, sat.inclination,
                         sat.eccentricity});
    }
    else
    {
      dropped_cids.push_back(cid);
    }
  }
  report.kept = keepers.size();
  report.dropped = dropped_cids.size();

  // --- Step 2: epoch-coherence warning
  // ---------------------------------------- RAAN precesses several deg/day; if
  // the keepers' epochs are spread out the raw RAANs won't cluster cleanly, so
  // warn (but never fail) on a wide spread.
  if (!keepers.empty())
  {
    double lo_epoch = keepers.front().epoch_jd;
    double hi_epoch = keepers.front().epoch_jd;
    for (const Keeper& k : keepers)
    {
      lo_epoch = std::min(lo_epoch, k.epoch_jd);
      hi_epoch = std::max(hi_epoch, k.epoch_jd);
    }
    // Same threshold the freshness gate uses, kept in one place (EpochPolicy)
    // so the two can't drift apart. The measurement here stays keeper-only.
    if (hi_epoch - lo_epoch > EpochPolicy{}.max_cluster_spread_days)
    {
      // No figure in the text: the threshold lives in EpochPolicy, so quoting
      // it here would silently go stale if that default changes.
      report.warnings.push_back(
          "keeper epochs are spread wider than the clustering tolerance; RAAN "
          "clustering may merge or split planes (use a fresher snapshot)");
    }
  }

  const std::size_t k = keepers.size();

  // --- Step 2b: precession-corrected RAAN (opt-in)
  // ---------------------------- Binning on raw RAAN scatters one real orbit
  // across bins because RAAN precesses between the keepers' differing epochs.
  // When asked, project every keeper's RAAN to a COMMON reference epoch and
  // cluster on that instead. Only keepers[i].raan is rewritten here -- the
  // sort/gather/wire pipeline below is untouched and reads the corrected value
  // transparently.
  if (spec.correct_raan_precession && k > 0)
  {
    // Reference epoch = the NEWEST keeper epoch. Choosing the newest means
    // every correction ADVANCES a past RAAN forward in time; no keeper is
    // extrapolated beyond data it doesn't have, and the freshest satellites are
    // the anchor.
    double t_ref_jd =
        keepers.front().epoch_jd;  // seed, then take the max below
    for (const Keeper& kp : keepers)
    {
      t_ref_jd = std::max(t_ref_jd, kp.epoch_jd);
    }
    for (Keeper& kp : keepers)
    {
      // Omega(t_ref) = raw + Omega_dot * (t_ref - epoch): raan_dot is deg/day
      // and (t_ref - epoch) is in JD days, so the product is degrees. Adding it
      // (with raan_dot < 0 for prograde and t_ref - epoch >= 0) pulls an older,
      // less- precessed satellite forward onto its group -- the sign the
      // sign-check test pins. A wrong sign would push them apart instead.
      const double drift_deg =
          raan_dot(kp.mean_motion, kp.inclination, kp.eccentricity) *
          (t_ref_jd - kp.epoch_jd);
      double corrected = kp.raan + drift_deg;  // project to the reference epoch
      corrected = std::fmod(corrected, 360.0);  // fold into a single revolution
      if (corrected < 0.0)
      {
        corrected += 360.0;  // fmod keeps the sign; RAAN must live in [0,360)
      }
      kp.raan =
          corrected;  // downstream binning/sort now see the corrected RAAN
    }
  }

  // --- Step 3: cluster by RAAN into planes, RAAN treated as circular
  // ---------- Sort by RAAN. The gap method needs it; binning does not, but it
  // is cheap and keeps a single code path feeding step 4.
  std::sort(keepers.begin(), keepers.end(),
            [](const Keeper& a, const Keeper& b) { return a.raan < b.raan; });

  std::vector<std::size_t> plane_of(k, 0);  // plane per RAAN-sorted keeper
  std::size_t num_planes = 0;

  if (spec.num_planes > 0)
  {
    // Fixed-plane-count binning: snap each satellite to the nearest evenly
    // spaced nominal plane. Robust to a densely-filled RAAN circle (where the
    // gap method finds no empty stripe and collapses to one plane). Strays and
    // maneuvering satellites snap to their nearest plane instead of spawning
    // junk single-satellite planes.
    const long long planes = static_cast<long long>(spec.num_planes);
    const double width = 360.0 / static_cast<double>(spec.num_planes);

    std::vector<std::size_t> raw(k);  // nominal bin index per keeper
    for (std::size_t i = 0; i < k; ++i)
    {
      // Round to the nearest plane center; mod wraps 360 -> 0 (RAAN circular),
      // and the extra + planes keeps a negative result non-negative.
      const long long b = std::llround(keepers[i].raan / width);
      raw[i] = static_cast<std::size_t>(((b % planes) + planes) % planes);
    }

    // Compact the occupied bins to a dense, gap-free numbering in increasing
    // order, so plane_offsets has no zero-width planes (verify() requires
    // contiguous, non-empty planes). An empty nominal plane simply vanishes.
    std::vector<std::size_t> occupied(raw);
    std::sort(occupied.begin(), occupied.end());
    occupied.erase(std::unique(occupied.begin(), occupied.end()),
                   occupied.end());
    for (std::size_t i = 0; i < k; ++i)
    {
      plane_of[i] = static_cast<std::size_t>(
          std::lower_bound(occupied.begin(), occupied.end(), raw[i]) -
          occupied.begin());
    }
    num_planes = occupied.size();
  }
  else if (k > 0)
  {
    // Gap-based fallback (spec.num_planes == 0): a RAAN gap wider than the
    // threshold marks a plane boundary. Works when planes are cleanly
    // separated; the synthetic topology tests rely on this path.
    const auto gap_after = [&](std::size_t j) -> double
    {
      if (j + 1 < k)
      {
        return keepers[j + 1].raan - keepers[j].raan;
      }
      return keepers.front().raan + 360.0 - keepers.back().raan;
    };

    // Start the walk so planes come out in increasing RAAN. Normally the wrap
    // gap is a boundary and we start at the smallest RAAN. If it is NOT a
    // boundary, a plane straddles 0 deg, so start just past the first real
    // boundary to keep that plane contiguous instead of split across the seam.
    std::size_t start = 0;
    if (gap_after(k - 1) <= spec.raan_gap_deg)
    {
      for (std::size_t j = 0; j + 1 < k; ++j)
      {
        if (gap_after(j) > spec.raan_gap_deg)
        {
          start = j + 1;
          break;
        }
      }
    }

    std::size_t plane = 0;
    for (std::size_t step = 0; step < k; ++step)
    {
      const std::size_t pos = (start + step) % k;
      plane_of[pos] = plane;
      // Bump the plane when we cross a boundary, except on the closing step
      // (that gap returns us to `start`, not into a new plane).
      const bool last = (step + 1 == k);
      if (!last && gap_after(pos) > spec.raan_gap_deg)
      {
        ++plane;
      }
    }
    num_planes = plane + 1;
  }
  report.num_planes = num_planes;
  c.num_planes = num_planes;

  // --- Step 4: final order = keepers sorted by (plane, mean_anomaly)
  // ----------
  std::vector<std::size_t> perm(k);
  std::iota(perm.begin(), perm.end(), 0);
  std::sort(perm.begin(), perm.end(),
            [&](std::size_t a, std::size_t b)
            {
              if (plane_of[a] != plane_of[b])
              {
                return plane_of[a] < plane_of[b];
              }
              return keepers[a].mean_anomaly < keepers[b].mean_anomaly;
            });

  std::vector<NodeId> order(k);           // new dense index -> OLD index
  std::vector<std::size_t> plane_new(k);  // new dense index -> plane
  for (std::size_t i = 0; i < k; ++i)
  {
    order[i] = keepers[perm[i]].old_index;
    plane_new[i] = plane_of[perm[i]];
  }

  // --- Step 5: gather-rebuild every dense array, then drop non-keepers
  // -------- Build a fresh array new[i] = old[order[i]] and swap it in; never
  // sort the parallel arrays in place (that would desync them mid-permutation).
  const auto gather = [&](auto& vec)
  {
    using V = std::decay_t<decltype(vec)>;
    V rebuilt;
    rebuilt.reserve(k);
    for (std::size_t i = 0; i < k; ++i)
    {
      rebuilt.push_back(vec[order[i]]);
    }
    vec.swap(rebuilt);
  };
  gather(c.position_eci);
  gather(c.velocity_eci);
  gather(c.position_ecef);
  gather(c.isl_neighbors);
  gather(c.grid_coord);
  gather(c.catalog_id);
  c.num_satellites = k;

  // Dropped satellites are absent from `order`; remove their cold records too.
  for (const CatalogId cid : dropped_cids)
  {
    c.by_catalog.erase(cid);
  }

  // --- Step 6: rebuild the NodeId bridge for the new layout
  // -------------------
  for (std::size_t i = 0; i < k; ++i)
  {
    c.by_catalog.at(c.catalog_id[i]).index = static_cast<NodeId>(i);
  }

  // --- Step 7: plane_offsets = prefix sum of per-plane counts
  // ----------------- With no keepers there are no planes; leave plane_offsets
  // empty so the result is the valid "pre-topology" empty state verify()
  // accepts, not {0} (which a num_planes==0 constellation must not carry).
  if (num_planes > 0)
  {
    c.plane_offsets.assign(num_planes + 1, 0);
    for (std::size_t i = 0; i < k; ++i)
    {
      ++c.plane_offsets[plane_new[i] + 1];
    }
    for (std::size_t p = 0; p < num_planes; ++p)
    {
      c.plane_offsets[p + 1] += c.plane_offsets[p];
    }
  }
  else
  {
    c.plane_offsets.clear();
  }

  // --- Step 8: grid_coord (plane + slot within plane)
  // -------------------------
  for (std::size_t i = 0; i < k; ++i)
  {
    const std::size_t p = plane_new[i];
    c.grid_coord[i].plane = static_cast<std::uint16_t>(p);
    c.grid_coord[i].slot = static_cast<std::uint16_t>(i - c.plane_offsets[p]);
  }

  // --- Step 9: wire in-plane neighbors (fore = slot 0, aft = slot 1)
  // ---------- Each plane is a ring over its dense range; fore/aft wrap at the
  // boundary.
  for (std::size_t p = 0; p < num_planes; ++p)
  {
    const NodeId plo = c.plane_offsets[p];
    const NodeId phi = c.plane_offsets[p + 1] - 1;  // plane non-empty here
    for (NodeId i = plo; i <= phi; ++i)
    {
      auto& nb = c.isl_neighbors[i];
      // Start clean: gathered slots are stale OLD indices; cross-plane stays
      // empty and a size-1 plane must keep its in-plane slots empty too.
      nb.fill(INVALID_NODE);
      if (phi > plo)
      {  // skip a lone satellite: it has no in-plane neighbor
        nb[0] = (i == phi) ? plo : i + 1;  // fore (toward higher slot, wraps)
        nb[1] = (i == plo) ? phi : i - 1;  // aft  (toward lower slot, wraps)
      }
    }
  }

  // --- Step 10: the scaffold must be internally consistent
  // --------------------
  assert(c.verify());
  return report;
}

}  // namespace leo
