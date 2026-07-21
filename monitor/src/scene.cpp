/*****************************************************************************
  filename scene.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator (monitor)
  date 2026-07-14
  Brief Description:
    Implementations of the CPU-side geometry builders declared in scene.hpp.
    Each one turns read-only simulator state into a vertex buffer; none of them
    decides anything the simulator has not already decided.
 *****************************************************************************/

#include "scene.hpp"

#include <cmath>   /* std::cos/sin: the sphere's parametric rows           */
#include <numbers> /* std::numbers::pi_v: the sphere's angular sweep       */

#include <glm/geometric.hpp> /* glm::distance: the edge-length gate        */

namespace monitor
{

namespace
{

/* The route highlight's color: a saturated gold, chosen to be the one thing on
   screen that no link color can be confused with. */
constexpr glm::vec3 kRouteGold(1.0f, 0.9f, 0.15f);

/*---------------------------------------------------------------------------
  Function: world_of
  Description: km -> world space. The scale is applied in DOUBLE and narrowed
               only afterwards: at ~7000 km magnitudes a float has already lost
               meters of precision, so scaling after the narrowing would bake
               that error into the picture.
  Input: ecef_km  -- the position, ECEF kilometers
         km_scale -- km -> world units
  Outputs: The world-space position.
---------------------------------------------------------------------------*/
glm::vec3 world_of(const leo::Vec3& ecef_km, /* double-precision source */
                   float km_scale            /* the shrink factor       */
)
{
  return glm::vec3(static_cast<float>(ecef_km.x * km_scale),
                   static_cast<float>(ecef_km.y * km_scale),
                   static_cast<float>(ecef_km.z * km_scale));
}

/*---------------------------------------------------------------------------
  Function: edge_color
  Description: The color that names each link kind on screen -- blue in-plane,
               orange cross-plane, green uplink. One place, so the legend in the
               user's head stays true across every buffer that draws links.
  Input: k -- the edge kind
  Outputs: Its color.
---------------------------------------------------------------------------*/
glm::vec3 edge_color(leo::Snapshot::Edge::Kind k)
{
  switch (k)
  {
    case leo::Snapshot::Edge::kInPlane:
      return glm::vec3(0.30f, 0.55f, 0.95f);
    case leo::Snapshot::Edge::kCrossPlane:
      return glm::vec3(0.95f, 0.55f, 0.20f);
    case leo::Snapshot::Edge::kUplink:
      return glm::vec3(0.30f, 0.85f, 0.40f);
  }
  return glm::vec3(1.0f); /* unreachable; white would make a new kind obvious */
}

/*---------------------------------------------------------------------------
  Function: drawable
  Description: Is this node's position worth drawing? The snapshot already
               decided (a failed propagation zeroes a position), so the renderer
               only has to ask. The size check tolerates a stale mask rather
               than indexing past its end.
  Input: s -- the snapshot
         n -- the node
  Outputs: True if the node has a usable position this tick.
---------------------------------------------------------------------------*/
bool drawable(const leo::Snapshot& s, leo::NodeId n)
{
  return n < s.position_valid.size() && s.position_valid[n] != 0;
}

/*---------------------------------------------------------------------------
  Function: plane_shown
  Description: Is this node in a visible plane? An empty filter shows all, and
               ground nodes -- which have no meaningful plane -- are never
               hidden by it, so an endpoint cannot vanish when the user
               unchecks its plane.
  Input: c        -- the constellation (grid_coord)
         plane_on -- the mask; empty means "everything"
         n        -- the node
  Outputs: True if the node should be drawn.
---------------------------------------------------------------------------*/
bool plane_shown(const leo::Constellation& c, /* plane of each node */
                 const std::vector<std::uint8_t>& plane_on, /* {} == show all */
                 leo::NodeId n /* the node in question */
)
{
  if (plane_on.empty())
  {
    return true;
  }
  if (c.is_ground(n))
  {
    return true; /* an endpoint belongs to no plane and must never be hidden */
  }
  const std::uint16_t p = c.grid_coord[n].plane;
  return p < plane_on.size() && plane_on[p] != 0;
}

} /* namespace */

/* See scene.hpp for the contract. */
void build_sphere(float radius,                       /* world units       */
                  int stacks,                         /* latitude rows     */
                  int slices,                         /* longitude columns */
                  std::vector<MeshVertex>& verts,     /* out: vertices     */
                  std::vector<std::uint32_t>& indices /* out: triangles    */
)
{
  verts.clear();
  indices.clear();
  const float pi = std::numbers::pi_v<float>;

  /* Rows of vertices from the north pole (stack 0) to the south. For a sphere
     centered on the origin the outward normal IS the normalized position, so
     the two are generated from one unit vector. */
  for (int i = 0; i <= stacks; ++i)
  {
    const float v = static_cast<float>(i) / static_cast<float>(stacks);
    const float phi = v * pi;      /* polar angle, 0..pi          */
    const float y = std::cos(phi); /* height of this row          */
    const float r = std::sin(phi); /* radius of this row's circle */
    for (int j = 0; j <= slices; ++j)
    {
      const float u = static_cast<float>(j) / static_cast<float>(slices);
      const float theta = u * 2.0f * pi; /* azimuth, 0..2pi */
      const glm::vec3 n(r * std::cos(theta), y, r * std::sin(theta));
      verts.push_back({n * radius, n});
    }
  }

  /* Two triangles per quad of the (stack, slice) grid, wound so the OUTWARD
     face is counter-clockwise. This is not a detail: under CULL_BACK a reversed
     winding culls the near hemisphere, the Earth stops writing depth in front
     of the far-side satellites, and the globe collapses into a flat disc. */
  const int row = slices + 1; /* vertices per row, including the seam */
  for (int i = 0; i < stacks; ++i)
  {
    for (int j = 0; j < slices; ++j)
    {
      const std::uint32_t a = static_cast<std::uint32_t>(i * row + j);
      const std::uint32_t b = static_cast<std::uint32_t>(a + row);
      indices.push_back(a);
      indices.push_back(a + 1);
      indices.push_back(b);
      indices.push_back(a + 1);
      indices.push_back(b + 1);
      indices.push_back(b);
    }
  }
}

/* See scene.hpp for the contract. */
void build_satellite_points(
    const leo::Constellation& c,              /* positions           */
    const leo::Snapshot& s,                   /* validity flags      */
    float km_scale,                           /* km -> world units   */
    std::vector<glm::vec3>& out,              /* out: the points     */
    const std::vector<std::uint8_t>& plane_on /* {} == all planes    */
)
{
  out.clear();
  out.reserve(c.num_satellites); /* the usual case is "nearly all of them" */
  for (leo::NodeId n = 0; n < c.num_satellites; ++n)
  {
    if (!drawable(s, n))
    {
      continue; /* failed propagation: cull here so nothing downstream cares */
    }
    if (!plane_shown(c, plane_on, n))
    {
      continue; /* the user unchecked this plane */
    }
    out.push_back(world_of(c.position_ecef[n], km_scale));
  }
}

/* See scene.hpp for the contract. */
void build_edge_lines(
    const leo::Constellation& c,              /* positions            */
    const leo::Snapshot& s,                   /* the typed edge list  */
    float km_scale,                           /* km -> world units    */
    std::vector<LineVertex>& out,             /* out: line vertices   */
    const leo::LinkSpec& spec,                /* the feasibility gate */
    bool only_routable,                       /* feasible/structural  */
    const std::vector<std::uint8_t>& plane_on /* {} == all planes     */
)
{
  out.clear();
  out.reserve(s.edges.size() * 2); /* two vertices per edge */
  for (const leo::Snapshot::Edge& e : s.edges)
  {
    if (e.kind == leo::Snapshot::Edge::kInPlane)
    {
      continue; /* blue comes from build_ingroup_lines: never draw it twice */
    }
    if (!drawable(s, e.a) || !drawable(s, e.b))
    {
      continue; /* an edge is only as drawable as its worse endpoint */
    }
    if (!plane_shown(c, plane_on, e.a) || !plane_shown(c, plane_on, e.b))
    {
      continue; /* an edge touching a hidden plane goes with it */
    }

    /* Feasible view only: the physical gate, identical to Router::build's edge
       test, so a drawn edge is one the router could route over. In the
       structural view every cross-plane edge stays, which is what makes the
       designed lattice (and its over-long chords) visible. Uplinks are never
       gated -- a ground endpoint sits ON the surface, and earth_clear would
       wrongly reject it -- exactly as the router reasons. */
    if (only_routable && !c.is_ground(e.a) && !c.is_ground(e.b))
    {
      const leo::Vec3& pa = c.position_ecef[e.a];
      const leo::Vec3& pb = c.position_ecef[e.b];
      if (glm::distance(pa, pb) > spec.max_range_km)
      {
        continue;
      }
      if (!leo::earth_clear(pa, pb, spec.block_radius_km))
      {
        continue;
      }
    }

    const glm::vec3 col = edge_color(e.kind); /* names the kind on screen */
    out.push_back({world_of(c.position_ecef[e.a], km_scale), col});
    out.push_back({world_of(c.position_ecef[e.b], km_scale), col});
  }
}

/* See scene.hpp for the contract. */
void build_ingroup_lines(
    const leo::Constellation& c,              /* slots + positions   */
    float km_scale,                           /* km -> world units   */
    std::vector<LineVertex>& out,             /* out: line vertices  */
    const std::vector<std::uint8_t>& plane_on /* {} == all planes    */
)
{
  out.clear();
  const glm::vec3 col = edge_color(leo::Snapshot::Edge::kInPlane); /* blue */
  const leo::NodeId n = static_cast<leo::NodeId>(c.num_satellites);

  /* Slots 0/1 are the in-plane terminals, seeded from geometry this tick by
     leo::seed_in_plane_links -- already mutual, in range, and line-of-sight
     clear. So there is nothing left to filter here, only to draw. */
  for (leo::NodeId i = 0; i < n; ++i)
  {
    if (!plane_shown(c, plane_on, i))
    {
      continue; /* group hidden; both endpoints share it, so one test suffices
                 */
    }
    for (int slot = 0; slot < 2; ++slot) /* 0 = fore, 1 = aft */
    {
      const leo::NodeId j = c.isl_neighbors[i][slot];
      if (j == leo::INVALID_NODE)
      {
        continue; /* no reciprocating neighbor on this side this tick */
      }
      if (j <= i)
      {
        continue; /* emit each undirected pair once: the other end draws it */
      }
      out.push_back({world_of(c.position_ecef[i], km_scale), col});
      out.push_back({world_of(c.position_ecef[j], km_scale), col});
    }
  }
}

/* See scene.hpp for the contract. */
void build_path_lines(
    const leo::Constellation& c,  /* live positions of the path nodes */
    const leo::Snapshot& s,       /* the path                         */
    float km_scale,               /* km -> world units                */
    std::vector<RouteVertex>& out /* out: billboard vertices          */
)
{
  out.clear();
  if (!s.found || s.path.size() < 2)
  {
    return; /* nothing routed: an empty buffer draws nothing */
  }

  /* Six vertices (two triangles = one quad) per segment; the shader widens each
     quad into a screen-space band centered on the segment. */
  out.reserve((s.path.size() - 1) * 6);
  for (std::size_t i = 1; i < s.path.size(); ++i)
  {
    const glm::vec3 a = world_of(c.position_ecef[s.path[i - 1]], km_scale);
    const glm::vec3 b = world_of(c.position_ecef[s.path[i]], km_scale);

    /* Each quad corner is A or B, offset by +/- the perpendicular the SHADER
       computes. The shader derives that perpendicular from (other - pos), so at
       the B end the direction reverses: `side` is flipped there to land the
       corner on the same PHYSICAL edge as its A partner, while `across` records
       which edge it truly landed on, for the fragment's soft gradient. Winding
       is irrelevant -- the route pipeline culls nothing. */
    const RouteVertex a_pos{a, b, kRouteGold, +1.0f, +1.0f}; /* A, +perp edge */
    const RouteVertex a_neg{a, b, kRouteGold, -1.0f, -1.0f}; /* A, -perp edge */
    const RouteVertex b_pos{b, a, kRouteGold, -1.0f, +1.0f}; /* B, +perp edge */
    const RouteVertex b_neg{b, a, kRouteGold, +1.0f, -1.0f}; /* B, -perp edge */
    out.push_back(a_pos);
    out.push_back(a_neg);
    out.push_back(b_neg);
    out.push_back(a_pos);
    out.push_back(b_neg);
    out.push_back(b_pos);
  }
}

/* See scene.hpp for the contract. */
std::vector<std::uint8_t> plane_mask_on_route(
    const leo::Constellation& c, /* plane of each satellite */
    const leo::Snapshot& s,      /* the route               */
    std::size_t num_planes       /* mask size               */
)
{
  /* All-zero by default: with no route we want to show NOTHING, not everything.
     A sized-but-zeroed mask means "hide all" downstream, whereas an EMPTY mask
     would mean "show all" -- the opposite of what a missing route should say.
   */
  std::vector<std::uint8_t> on(num_planes, 0);
  if (!s.found)
  {
    return on;
  }
  for (const leo::NodeId n : s.path)
  {
    if (c.is_ground(n))
    {
      continue; /* the two endpoints belong to no plane */
    }
    const std::uint16_t p = c.grid_coord[n].plane;
    if (p < num_planes)
    {
      on[p] = 1;
    }
  }
  return on;
}

} /* namespace monitor */
