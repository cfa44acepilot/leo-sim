/*****************************************************************************
  filename scene.hpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator (monitor)
  date 2026-07-14
  Brief Description:
    CPU-side geometry builders for the renderer: the simulator's state turned
    into vertex buffers, and nothing else.

    ISOLATION IS THE POINT: every function here is a FREE function taking the
    core types by const reference. The renderer needs no new methods on the
    simulator classes -- it reads only what Snapshot and Constellation already
    expose -- so nothing in src/ is touched to make a picture appear.

    All positions come out in WORLD units = kilometers * km_scale, so the caller
    picks a convenient scale (1/1000 puts the Earth's radius at ~6.378 world
    units). Frame discipline: we read position_ecef (Earth-fixed) and draw
    against a fixed Earth. velocity is ECI -- a different frame, and a trap --
    so it is never used here, and gmst_rad is never re-applied.
 *****************************************************************************/

#ifndef LEO_MONITOR_SCENE_HPP
#define LEO_MONITOR_SCENE_HPP

#include <cstddef> /* std::size_t: plane counts                        */
#include <cstdint> /* std::uint8_t/uint32_t: masks and index buffers   */
#include <vector>  /* every builder fills a vector                     */

#include <glm/vec3.hpp> /* the vertex payloads are all vec3            */

#include "constellation.hpp" /* the state being drawn                  */
#include "isl_links.hpp"     /* LinkSpec: the feasibility gate to echo */
#include "simulator.hpp"     /* Snapshot: route, edges, validity flags */

namespace monitor
{

/* Interleaved vertex for the lit Earth/atmosphere spheres. */
struct MeshVertex
{
  glm::vec3 pos;    /* world-space position          */
  glm::vec3 normal; /* outward normal, for the light */
};

/* Interleaved vertex for colored lines (ISL and uplink edges). */
struct LineVertex
{
  glm::vec3 pos;   /* world-space endpoint  */
  glm::vec3 color; /* per-edge-kind color   */
};

/* Vertex for the highlighted route, drawn as THICK screen-space billboard lines
   rather than 1px GL lines: Vulkan's lineWidth > 1 needs the wideLines feature
   and is driver-dependent, so the route's prominence can never rest on it.
   Each path SEGMENT is expanded into two triangles (a camera-facing quad) and
   the vertex shader offsets pos perpendicular to the segment by side * a
   screen-space half-width, giving a consistent pixel thickness on any GPU at
   any zoom. */
struct RouteVertex
{
  glm::vec3 pos;   /* this segment endpoint, world space                     */
  glm::vec3 other; /* the opposite endpoint: gives the on-screen direction   */
  glm::vec3 color; /* per-vertex so the highlight can vary later             */
  float side;      /* +1/-1: which way to offset THIS vertex. The perp flips
                      at the far endpoint, so this is NOT the band side       */
  float across;    /* -1..+1 true position across the band, for the fragment's
                      soft edge -- it tracks the edge a corner actually lands
                      on, which `side` cannot, having cancelled the flip     */
};

/*---------------------------------------------------------------------------
  Function: build_sphere
  Description: A UV sphere of the given world radius, centered on the origin
               (which IS the Earth's center in ECEF). Serves as both the globe
               and the translucent atmosphere shell.

               The winding must come out CCW-outward: with CULL_BACK, a reversed
               winding culls the NEAR hemisphere, the globe stops occluding
               anything behind it, and the whole scene collapses to a flat disc.
               That was a real bug once; it is why this is one shared builder.
  Input: radius  -- world units
         stacks  -- latitude divisions
         slices  -- longitude divisions
         verts   -- receives the vertex list
         indices -- receives the triangle list
  Outputs: None; the two vectors are the result.
---------------------------------------------------------------------------*/
void build_sphere(float radius, /* world units                  */
                  int stacks,   /* latitude divisions           */
                  int slices,   /* longitude divisions          */
                  std::vector<MeshVertex>& verts,     /* out: vertices     */
                  std::vector<std::uint32_t>& indices /* out: triangle list */
);

/*---------------------------------------------------------------------------
  Function: build_satellite_points
  Description: One world-space point per DRAWABLE satellite. Satellites whose
               propagation failed are culled HERE, so no downstream stage has to
               know that a position can be dead.
  Input: c        -- the constellation (positions)
         s        -- the snapshot (position_valid flags)
         km_scale -- km -> world units
         out      -- receives the points
         plane_on -- per-plane visibility; plane_on[p] != 0 draws plane p, and
                     an EMPTY vector means "draw every plane". Ground nodes are
                     never gated by it.
  Outputs: None; `out` is the result.
---------------------------------------------------------------------------*/
void build_satellite_points(const leo::Constellation& c, /* positions */
                            const leo::Snapshot& s, /* validity flags       */
                            float km_scale,         /* km -> world units    */
                            std::vector<glm::vec3>& out, /* out: the points */
                            const std::vector<std::uint8_t>& plane_on = {}
                            /* {} == all planes     */
);

/*---------------------------------------------------------------------------
  Function: build_edge_lines
  Description: The CROSS-PLANE (orange) and UPLINK (green) edges as a line list.
               In-plane (blue) edges do NOT come from here: build_ingroup_lines
               draws those from the ISL slots, so this skips them rather than
               drawing them twice.

               only_routable picks between two honest views:
                 true  (feasible)   -- apply the SAME gate Router::build uses,
                                       so every edge drawn is one the router
                                       can actually use;
                 false (structural) -- draw every cross-plane/uplink edge in the
                                       snapshot, including ones the router would
                                       reject, which is what makes the designed
                                       lattice visible.
               Uplinks are never gated: a ground endpoint sits ON the
               surface, and earth_clear would wrongly reject it -- exactly
               as the router reasons.
  Input: c             -- the constellation
         s             -- the snapshot (its typed edge list)
         km_scale      -- km -> world units
         out           -- receives the line vertices
         spec          -- the gate's thresholds (used only when only_routable)
         only_routable -- feasible view (true) or structural view (false)
         plane_on      -- per-plane visibility; {} == all planes
  Outputs: None; `out` is the result.
---------------------------------------------------------------------------*/
void build_edge_lines(const leo::Constellation& c,  /* positions           */
                      const leo::Snapshot& s,       /* the typed edges     */
                      float km_scale,               /* km -> world units   */
                      std::vector<LineVertex>& out, /* out: line vertices  */
                      const leo::LinkSpec& spec,    /* the feasibility gate*/
                      bool only_routable = true,    /* feasible/structural */
                      const std::vector<std::uint8_t>& plane_on = {}
                      /* {} == all planes    */
);

/*---------------------------------------------------------------------------
  Function: build_ingroup_lines
  Description: The IN-GROUP (blue) links: a straight read of ISL slots 0/1.

               This USED to recompute the links geometrically, here in the
               renderer, because the slots held build_topology's slot-order ring
               -- which bore no relation to which satellites were actually near
               each other. The core now seeds slots 0/1 geometrically every tick
               (leo::seed_in_plane_links), so the recompute is gone and the
               picture is drawn from the very links the router routes on. Every
               blue line on screen is an edge the router can use, and there is
               ONE source of truth instead of two that disagreed.

               Mutual pairing and the in-plane degree cap of 2 are properties of
               the seeding, so nothing is enforced here; each undirected pair is
               simply emitted once.
  Input: c        -- the constellation (slots + positions)
         km_scale -- km -> world units
         out      -- receives the line vertices
         plane_on -- per-plane visibility; {} == all planes
  Outputs: None; `out` is the result.
---------------------------------------------------------------------------*/
void build_ingroup_lines(const leo::Constellation& c,  /* slots + positions   */
                         float km_scale,               /* km -> world units   */
                         std::vector<LineVertex>& out, /* out: line vertices  */
                         const std::vector<std::uint8_t>& plane_on = {}
                         /* {} == all planes    */
);

/*---------------------------------------------------------------------------
  Function: build_path_lines
  Description: The highlighted route as thick billboard geometry -- six
               RouteVertex (two triangles = one camera-facing quad) per path
               segment. Empty when there is no route.

               The offset is applied in the SHADER (screen-space width), so this
               geometry is camera-independent: it need not be rebuilt when the
               camera moves. It IS rebuilt every frame anyway, because the
               satellites move and a cached line drifts off the dots it is
               supposed to run through.
  Input: c        -- the constellation (live positions)
         s        -- the snapshot (the path)
         km_scale -- km -> world units
         out      -- receives the route vertices
  Outputs: None; `out` is the result.
---------------------------------------------------------------------------*/
void build_path_lines(
    const leo::Constellation& c,  /* live positions of the path nodes */
    const leo::Snapshot& s,       /* the path itself                  */
    float km_scale,               /* km -> world units                */
    std::vector<RouteVertex>& out /* out: billboard vertices          */
);

/*---------------------------------------------------------------------------
  Function: plane_mask_on_route
  Description: A plane-visibility mask (same shape as the Planes panel's
               plane_on) selecting exactly the planes that carry at least one
               satellite on the current route. Lets a caller isolate the planes
               the path threads through without disturbing the manual mask.
  Input: c          -- the constellation (grid_coord)
         s          -- the snapshot (the path)
         num_planes -- size of the mask to produce
  Outputs: The mask. All-zero when there is no route, so the caller shows
           NOTHING rather than everything -- the safe reading of "no route".
---------------------------------------------------------------------------*/
std::vector<std::uint8_t> plane_mask_on_route(
    const leo::Constellation& c, /* plane of each satellite */
    const leo::Snapshot& s,      /* the route               */
    std::size_t num_planes       /* mask size               */
);

} /* namespace monitor */

#endif /* LEO_MONITOR_SCENE_HPP */
