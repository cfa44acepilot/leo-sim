/*****************************************************************************
  filename isl_links.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-14
  Brief Description:
    Seeds every ISL terminal from propagated geometry: slots 0/1 (in-plane) and
    slots 2/3 (cross-plane). Both passes run each tick, from the same positions
    the router prices its edges with -- which is what keeps the links the router
    uses and the links the renderer draws one and the same set.
 *****************************************************************************/

#include "isl_links.hpp"

#include <algorithm>  /* std::clamp: the segment closest-approach parameter */
#include <cstddef>    /* std::size_t                                        */
#include <limits>     /* infinity as the nearest-neighbor search seed       */

#include <glm/geometric.hpp>  /* dot/length/distance: all of the geometry   */

namespace leo
{

bool earth_clear(const Vec3& a, const Vec3& b, double block_radius_km)
{
  const Vec3 d = b - a;
  const double dd = glm::dot(d, d);
  // t parametrizes the foot of the perpendicular from the origin onto the line;
  // clamp to [0,1] so we measure the closest point on the SEGMENT, not the
  // infinite line. dd==0 (coincident endpoints) collapses to testing a.
  double t = 0.0;
  if (dd > 0.0)
  {
    t = -glm::dot(a, d) / dd;
  }
  t = std::clamp(t, 0.0, 1.0);
  const Vec3 closest = a + t * d;
  return glm::length(closest) >= block_radius_km;
}

namespace
{

// Nearest satellite in plane `plane` to satellite `i` that passes every link
// filter, or INVALID_NODE if none. position_ecef drives distance/line-of-sight;
// velocity_eci (TEME, free of Earth-rotation) drives the direction check.
NodeId best_in_plane(const Constellation& c, const LinkSpec& spec,
                     std::size_t i, std::size_t plane)
{
  const Vec3& pi = c.position_ecef[i];
  const Vec3& vi = c.velocity_eci[i];
  const NodeId begin = c.plane_offsets[plane];
  const NodeId end = c.plane_offsets[plane + 1];

  NodeId best = INVALID_NODE;
  double best_dist = std::numeric_limits<double>::infinity();
  for (NodeId j = begin; j < end; ++j)
  {
    const Vec3& pj = c.position_ecef[j];
    if (glm::length(pj) < spec.block_radius_km)
    {
      continue;  // j unusable
    }

    const double dist = glm::distance(pi, pj);
    if (dist > spec.max_range_km)
    {
      continue;  // out of laser range
    }
    if (dist >= best_dist)
    {
      continue;  // not an improvement
    }
    if (glm::dot(vi, c.velocity_eci[j]) <= spec.min_vel_dot)
    {
      continue;  // dir
    }
    if (!earth_clear(pi, pj, spec.block_radius_km))
    {
      continue;  // LOS (costliest)
    }

    best_dist = dist;
    best = j;
  }
  return best;
}

}  // namespace

// -----------------------------------------------------------------------------
// Function:    compute_in_plane_picks
// Description: Every satellite's nearest feasible same-group neighbor ahead and
//              behind. See the header for why this is separate from the
//              seeding.
// Input:       c    -- propagated constellation (positions + velocities)
//              spec -- range / line-of-sight / direction budget
//              pick_fore, pick_aft -- resized and filled, INVALID_NODE = none
// Outputs:     None; the two arrays are the result.
// -----------------------------------------------------------------------------
void compute_in_plane_picks(const Constellation& c, const LinkSpec& spec,
                            std::vector<NodeId>& pick_fore,
                            std::vector<NodeId>& pick_aft)
{
  const std::size_t n = c.num_satellites;
  pick_fore.assign(n, INVALID_NODE);
  pick_aft.assign(n, INVALID_NODE);
  if (c.num_planes == 0)
  {
    return;  // no groups yet -> nothing to scan
  }

  for (std::size_t i = 0; i < n; ++i)
  {
    const Vec3& pi = c.position_ecef[i];
    // A failed propagation zeroes the position, putting it inside the block
    // radius. Such a satellite links to nothing this tick.
    if (glm::length(pi) < spec.block_radius_km)
    {
      continue;
    }
    const Vec3& vi = c.velocity_eci[i];
    const double vlen = glm::length(vi); /* along-track needs a dir  */
    if (vlen <= 0.0)
    {
      continue;
    }
    const Vec3 vhat = vi / vlen; /* unit along-track (TEME)  */

    // Scan only the satellite's own RAAN group; plane_offsets brackets it.
    const std::size_t p = c.grid_coord[i].plane;
    const NodeId begin = c.plane_offsets[p];
    const NodeId end = c.plane_offsets[p + 1];
    double best_fore = std::numeric_limits<double>::infinity();
    double best_aft = std::numeric_limits<double>::infinity();
    for (NodeId j = begin; j < end; ++j)
    {
      if (j == static_cast<NodeId>(i))
      {
        continue; /* not its own neighbor     */
      }
      const Vec3& pj = c.position_ecef[j];
      if (glm::length(pj) < spec.block_radius_km)
      {
        continue;  // j unusable
      }
      const double dist = glm::distance(pi, pj);
      if (dist > spec.max_range_km)
      {
        continue;  // out of range
      }
      if (glm::dot(vi, c.velocity_eci[j]) <= spec.min_vel_dot)
      {
        continue;  // travelling the other way -> not a co-orbital neighbor
      }
      if (!earth_clear(pi, pj, spec.block_radius_km))
      {
        continue;  // LOS, costliest
      }
      // The sign of the along-track component splits ahead from behind; keeping
      // the nearest on each side gives at most one fore and one aft pick.
      const double along = glm::dot(pj - pi, vhat);
      if (along >= 0.0)
      {
        if (dist < best_fore)
        {
          best_fore = dist;
          pick_fore[i] = j;
        }
      }
      else if (dist < best_aft)
      {
        best_aft = dist;
        pick_aft[i] = j;
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Function:    seed_in_plane_links
// Description: Fill slots 0,1 with the MUTUAL in-plane pairs (see the header).
// Input:       c    -- propagated constellation; slots 0,1 are overwritten
//              spec -- the same link budget the router will gate edges with
// Outputs:     None; c.isl_neighbors[*][0..1] is the result.
// -----------------------------------------------------------------------------
void seed_in_plane_links(Constellation& c, const LinkSpec& spec)
{
  const std::size_t n = c.num_satellites;
  std::vector<NodeId> pick_fore; /* directed choices, fore   */
  std::vector<NodeId> pick_aft;  /* directed choices, aft    */
  compute_in_plane_picks(c, spec, pick_fore, pick_aft);

  // True iff a aimed one of its two in-plane terminals at b.
  const auto picked = [&](NodeId a, NodeId b)
  { return pick_fore[a] == b || pick_aft[a] == b; };

  for (std::size_t i = 0; i < n; ++i)
  {
    auto& nb = c.isl_neighbors[i];
    // Recompute from scratch (idempotent refresh), which also drops the
    // slot-order wiring build_topology left as the initial state.
    nb[0] = INVALID_NODE;
    nb[1] = INVALID_NODE;
    const NodeId self = static_cast<NodeId>(i);
    // Keep a pick only if it is reciprocated: the far end must be pointing a
    // terminal back at us, or there is no link to point at.
    if (pick_fore[i] != INVALID_NODE && picked(pick_fore[i], self))
    {
      nb[0] = pick_fore[i];
    }
    if (pick_aft[i] != INVALID_NODE && picked(pick_aft[i], self))
    {
      nb[1] = pick_aft[i];
    }
  }
}

void seed_cross_plane_links(Constellation& c, const LinkSpec& spec)
{
  const std::size_t n = c.num_satellites;
  const std::size_t planes = c.num_planes;

  for (std::size_t i = 0; i < n; ++i)
  {
    auto& nb = c.isl_neighbors[i];
    // Recompute from scratch each call (idempotent refresh).
    nb[2] = INVALID_NODE;
    nb[3] = INVALID_NODE;

    // A zeroed (failed-propagation) position sits inside block_radius; such a
    // satellite gets no cross-plane links this tick.
    if (glm::length(c.position_ecef[i]) < spec.block_radius_km)
    {
      continue;
    }
    if (planes <= 1)
    {
      continue;  // no adjacent plane exists
    }

    const std::size_t p = c.grid_coord[i].plane;
    const std::size_t lo = (p + planes - 1) % planes;  // lower-index neighbor
    const std::size_t hi = (p + 1) % planes;           // higher-index neighbor

    // Slot 3 always takes the hi side. Slot 2 takes the lo side only when it is
    // a distinct plane; with exactly two planes lo == hi, so we seed one slot
    // to avoid storing the same link twice.
    nb[3] = best_in_plane(c, spec, i, hi);
    if (lo != hi)
    {
      nb[2] = best_in_plane(c, spec, i, lo);
    }
  }
}

}  // namespace leo
