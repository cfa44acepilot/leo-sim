/*****************************************************************************
  filename topology.hpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-14
  Brief Description:
    The static lattice scaffold builder, plus the orbital helpers (raan_dot,
    orbit_altitude_km) and the shell presets the monitor's Shell dropdown picks
    between. Runs ONCE per dataset, before any propagation.
 *****************************************************************************/

// Static lattice scaffold builder. Takes the raw, insertion-order Constellation
// straight out of load_omm_json (all shells mixed together) and reduces it to a
// single shell laid out as a regular grid: satellites sorted by (plane, slot),
// plane_offsets filled, grid_coord assigned, and in-plane (fore/aft) ISL
// neighbors wired. Runs ONCE after load, before any propagation. Needs no
// positions -- it works purely from the published orbital elements.

#ifndef LEO_TOPOLOGY_HPP
#define LEO_TOPOLOGY_HPP

#include <cstddef>      /* std::size_t                                   */
#include <string>       /* the build report's warnings                   */
#include <string_view>  /* preset names: compile-time literals, not owned */
#include <vector>       /* the preset list and the warning list          */

#include "constellation.hpp"  /* the thing being laid out                */

namespace leo
{

// Which satellites form the shell and how planes are cut. v1 selects by
// inclination band alone; an altitude band is intentionally deferred.
struct ShellSpec
{
  double inclination_deg = 53.0;     // shell center inclination
  double inclination_tol_deg = 1.0;  // keep |incl - center| <= tol
  double raan_gap_deg = 5.0;  // RAAN gap that starts a new plane (gap mode)

  // Number of evenly spaced nominal planes to snap satellites to. 72 is the
  // plane count of Starlink's 53-degree shell. This fixed-bin assignment is
  // robust to a densely-filled RAAN circle, where gap detection finds no empty
  // stripe and collapses everything into one plane. Set to 0 to fall back to
  // the gap-based method (raan_gap_deg), which the synthetic gap tests rely on.
  std::size_t num_planes = 72;

  // Altitude band, to reject other sub-shells that share the inclination. The
  // 53 +/- 1 deg band alone admits satellites at several ALTITUDES, so a RAAN
  // group ends up mixing multiple real orbits (~112 sats/bin, 8000-13000 km
  // in-line neighbor gaps). Keeping only satellites near one altitude makes a
  // group one true orbit. Altitude is derived from mean motion (a = (MU/n^2)^
  // (1/3), altitude = a - Re). altitude_tol_km == 0 DISABLES the filter, so the
  // default is byte-identical to the inclination-only behavior; a positive
  // tolerance enables it.
  double altitude_km = 550.0;    // Starlink shell-1 nominal altitude
  double altitude_tol_km = 0.0;  // 0 -> filter off (back-compat); >0 enables

  // Cluster on RAAN projected to a common reference epoch instead of raw
  // published RAAN. Default false -> byte-identical to the raw behavior and all
  // existing tests. See the KNOWN LIMITATION note below: binning on raw RAAN
  // scatters one real orbit across bins because RAAN precesses (J2 nodal
  // regression) between the satellites' differing epochs. When true,
  // build_topology removes each keeper's drift to the newest keeper epoch via
  // raan_dot() before binning; only the value fed to the binning changes, the
  // whole downstream layout (sort/gather/plane_offsets/grid_coord/wiring) is
  // the same. This is the more-accurate clustering, not a second "view".
  bool correct_raan_precession = false;
};

// KNOWN LIMITATION of the fixed RAAN-bin clustering (mitigated by the OPT-IN
// ShellSpec::correct_raan_precession above; OFF by default). Binning on raw
// RAAN assumes a clean Walker plane count AND a common epoch. On a live
// multi-epoch snapshot that assumption breaks: J2 nodal regression makes each
// satellite's RAAN precess over time, so co-orbital satellites observed at
// different epochs drift into different bins, and satellites from genuinely
// different orbits can share a RAAN band and land together. A bin is therefore
// a RAAN NEIGHBORHOOD, not guaranteed a single co-orbital ring -- which is why
// in-line (fore/aft) neighbor distances within a raw "plane" can be 7000-13000
// km instead of the ~1000 km a real ring would give. Nothing downstream is
// wrong (routing's feasibility filter rejects those long links), but the
// grouping over-promises; the monitor UI calls these "RAAN groups" rather than
// "orbital planes".
//
// THE FIX (implemented behind the flag): before binning, project each
// satellite's RAAN to a common reference epoch by removing its J2 nodal-
// regression drift, using raan_dot() below. Cluster on the corrected RAAN.

// Nodal regression (RAAN precession) rate from mean orbital elements, in
// DEG/DAY. Negative for a prograde orbit (cos i > 0), which is the sign that
// lets a past RAAN be advanced forward to a later reference epoch. Inputs: mean
// motion in rev/day (OMM native), inclination in degrees, eccentricity
// dimensionless. Physics:
//   Omega_dot = -1.5 * n * J2 * (Re/a)^2 * cos(i) / (1 - e^2)^2
// with a from n via Kepler's third law (a = (MU / n^2)^(1/3)). Exposed (not
// file-local) so a unit test can pin the formula and its sign directly.
double raan_dot(double mean_motion_rev_day, double inclination_deg,
                double eccentricity);

// Circular-orbit altitude in km above the equatorial radius, from mean motion
// (rev/day). altitude = semi-major axis (a = (MU/n^2)^(1/3)) - Re. Exposed so a
// render-time "same shell" check can reuse the a-from-n math instead of
// duplicating it. Read-only; touches no constellation state.
double orbit_altitude_km(double mean_motion_rev_day);

// A named ShellSpec, so a front end can offer the known Starlink Gen1 shells as
// menu choices without hardcoding the numbers. ONE shell is modeled at a time
// (the whole pipeline reduces the catalog to a single shell); switching preset
// is a Tier-1 rebuild, not a live tweak.
struct ShellPreset
{
  std::string_view name;  // human label for a menu ("53 deg (main)")
  ShellSpec spec;         // the filter/cluster parameters that isolate it
};

// The built-in shell presets, 53-deg main FIRST so a default-constructed sim
// and index 0 agree. Plane counts are best public estimates for Starlink Gen1
// and are commented as such at the definition. Returned by const reference to a
// function-local static (built once), so callers can iterate it for a dropdown.
const std::vector<ShellPreset>& shell_presets();

// Outcome of the build: how many satellites stayed in the shell, how many were
// dropped, the resulting plane count, and any non-fatal warnings (e.g. epochs
// too spread out for RAAN to cluster cleanly).
struct BuildReport
{
  std::size_t kept = 0;
  std::size_t dropped = 0;
  std::size_t num_planes = 0;
  std::vector<std::string> warnings;
};

// Filter to the shell, cluster into planes by RAAN (treating RAAN as circular),
// sort by (plane, mean_anomaly), reorder every dense array to match, rebuild
// the NodeId bridge, and wire in-plane neighbors. Leaves cross-plane ISL
// terminals empty. Asserts verify() before returning in debug builds.
BuildReport build_topology(Constellation& c, const ShellSpec& spec);

}  // namespace leo

#endif  // LEO_TOPOLOGY_HPP
