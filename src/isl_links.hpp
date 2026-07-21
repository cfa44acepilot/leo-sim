/*****************************************************************************
  filename isl_links.hpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-14
  Brief Description:
    Inter-satellite-link seeding, in-plane and cross-plane. The notes below say
    why the in-plane links are seeded HERE, per tick, rather than left to
    build_topology's static wiring: that distinction is the difference between a
    router that uses the links which physically exist and one that routes the
    long way round past them.
 *****************************************************************************/

// Inter-satellite-link seeding, in-plane and cross-plane. Every one of a
// satellite's four ISL terminals is filled here, from the CURRENT geometry:
//   * slots 0,1 -- in-plane (same RAAN group): nearest feasible neighbor ahead
//                  and behind, mutually paired (seed_in_plane_links)
//   * slots 2,3 -- cross-plane: nearest feasible satellite in each adjacent
//                  plane (seed_cross_plane_links)
//
// WHY IN-PLANE IS SEEDED HERE AND NOT LEFT TO build_topology: build_topology
// wires slots 0/1 in SLOT ORDER (the ring it sorts by mean anomaly within a
// RAAN bin). On real data a RAAN bin is not one co-orbital ring, so slot order
// is not spatial order: those links ran thousands of km, mostly failed the
// router's feasibility gate, and bore almost no relation to the satellites that
// were actually near each other. The router was left routing the long way round
// past short links that plainly existed. Seeding in-plane links geometrically,
// per tick -- exactly as cross-plane links have always been seeded -- makes the
// links the router uses the links that are physically there. build_topology's
// wiring survives as the initial (pre-propagation) state.
//
// Runs AFTER Propagator::propagate() -- it reads position_ecef (geometry/LOS)
// and velocity_eci (direction). It does NOT compute or store edge weights;
// routing (step 8) derives those from positions and treats links as undirected.

#ifndef LEO_ISL_LINKS_HPP
#define LEO_ISL_LINKS_HPP

#include <vector>  /* the pick arrays returned by compute_in_plane_picks */

#include "constellation.hpp"  /* Constellation, NodeId, Vec3, the ISL slots  */

namespace leo
{

// Link feasibility tunables.
//
// GEOMETRY: max_range_km and block_radius_km are not independent. Two
// satellites at orbital radius R whose line of sight just grazes a shell of
// radius r can be at most
//        max_separation = 2 * sqrt(R^2 - r^2)
// apart before the sight line dips inside that shell (each satellite is
// sqrt(R^2 - r^2) from the tangent point; the chord is twice that). For
// Starlink shell 1, R ~= 6921 km (Earth 6378 + ~550 km altitude) and the
// grazing shell r = block_radius_km = 6458 km:
//        2 * sqrt(6921^2 - 6458^2) ~= 4980 km.
// So the PHYSICAL ceiling is ~5000 km; the old 5400 admitted links that would
// clip the atmosphere. Changing one of {max_range_km, block_radius_km} without
// the other is therefore unphysical -- they are coupled by the formula above.
struct LinkSpec
{
  // Earth radius (~6378 km) plus ~80 km grazing margin: a sight line whose
  // closest approach to Earth's center dips below this is blocked by the
  // atmosphere/limb. Also the "usable position" threshold -- a failed
  // propagation step zeroes a position, putting it well inside this radius.
  // Coupled to max_range_km via the grazing formula in this struct's header.
  double block_radius_km = 6458.0;

  // Maximum usable ISL range. Two distinct limits bound this:
  //   * geometric grazing ceiling ~5000 km (the hard physical limit above), and
  //   * operational terminal range, shorter due to laser pointing/power/
  //     acquisition -- published Starlink ISL figures suggest ~1500-3000 km.
  // We default to the OPERATIONAL value; the grazing ceiling is the absolute
  // bound it must stay under. The operational figure is an estimate.
  double max_range_km = 3000.0;

  double min_vel_dot = 0.0;  // require same direction of travel (dot > this)
};

// True iff the straight segment a-b does NOT pass within block_radius of
// Earth's center. Uses the closest approach of the segment to the origin;
// because both endpoints sit above block_radius, a closest point at a clamped
// endpoint is automatically clear -- Earth only blocks when the perpendicular
// foot lands strictly inside the segment.
bool earth_clear(const Vec3& a, const Vec3& b, double block_radius_km);

// Recompute every satellite's two cross-plane neighbors (slots 2,3) from the
// current positions. Idempotent: usable as the initial seed and as a naive
// per-tick refresh (just call again). Hysteresis to damp link chatter is a
// later refinement. Picks may be asymmetric; routing resolves symmetry.
void seed_cross_plane_links(Constellation& c, const LinkSpec& spec = {});

// Each satellite's DIRECTED in-plane choices: the nearest feasible same-group
// satellite ahead (pick_fore) and behind (pick_aft), or INVALID_NODE. "Ahead"
// and "behind" are the sign of the along-track component, so this is spatial
// order, not slot order. Both arrays are resized to num_satellites.
//
// Exposed separately from the seeding because a pick is not yet a link -- the
// seeding keeps only the MUTUAL ones -- and a diagnostic that wants to explain
// why some near neighbor did not link needs the raw choices. Sharing this one
// function is what keeps the explanation and the links in agreement.
void compute_in_plane_picks(const Constellation& c, const LinkSpec& spec,
                            std::vector<NodeId>& pick_fore,
                            std::vector<NodeId>& pick_aft);

// Recompute every satellite's two in-plane neighbors (slots 0,1) from the
// current positions, overwriting build_topology's slot-order wiring. Idempotent
// per tick, like the cross-plane seeding.
//
// A link is written only when the pair chose EACH OTHER (i's fore is j and j's
// aft is i, or vice versa). One-to-one is what a laser terminal actually is: a
// satellite that several neighbors aim at can only point back at one of them.
// The in-plane terminal cap of 2 needs no separate pass -- a satellite's mutual
// links are a subset of its at-most-two picks (one fore, one aft) -- so degree
// is capped by construction. Slots are cleared first, so a satellite with no
// reciprocating neighbor simply has none this tick.
void seed_in_plane_links(Constellation& c, const LinkSpec& spec = {});

}  // namespace leo

#endif  // LEO_ISL_LINKS_HPP
