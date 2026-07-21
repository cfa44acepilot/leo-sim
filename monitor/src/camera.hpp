/*****************************************************************************
  filename camera.hpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator (monitor)
  date 2026-07-14
  Brief Description:
    The orbit camera, driven by direct manipulation ("grab the surface").
    Header-only; pure math, no Vulkan.

    The camera sits at a FIXED spot in a camera-anchored frame (on +Z, looking
    at the origin, +Y up) and the WORLD is rotated in front of it by rot_.
    Storing the orientation as a quaternion -- rather than a yaw/pitch pair --
    is what makes the tablecloth drag exact: any rotation that carries the
    grabbed surface point onto the cursor can be represented, including the roll
    a yaw/pitch pair cannot express.

    The Earth is rendered FIXED in ECEF (we never spin it by gmst_rad), so ECEF
    +Z is the north pole and the default orientation puts it UP on screen.

    Drag model: on press the cursor ray is intersected with the globe and the
    hit point is anchored; every frame the same is done for the current cursor,
    and the world is rotated so the anchored point lands back under the cursor.
    The rotation is always composed from the PRESS-TIME orientation, never from
    the previous frame's, so a long drag cannot accumulate error -- the grabbed
    point is exactly under the cursor at every instant, at any zoom.
 *****************************************************************************/

#ifndef LEO_MONITOR_CAMERA_HPP
#define LEO_MONITOR_CAMERA_HPP

#include <algorithm> /* std::clamp (distance limits, acos domain guard)      */
#include <cmath>     /* std::acos/cos/sin/pow/sqrt (arc angle, ray, zoom)    */

#include <glm/mat3x3.hpp>                /* basis matrix -> initial quat     */
#include <glm/mat4x4.hpp>                /* view/proj matrices               */
#include <glm/vec3.hpp>                  /* directions, eye position         */
#include <glm/vec4.hpp>                  /* homogeneous unproject            */
#include <glm/geometric.hpp>             /* dot/cross/normalize              */
#include <glm/matrix.hpp>                /* transpose (basis), inverse (ray) */
#include <glm/trigonometric.hpp>         /* glm::radians                     */
#include <glm/gtc/quaternion.hpp>        /* quat, angleAxis, quat/mat4_cast  */
#include <glm/ext/matrix_clip_space.hpp> /* glm::perspective                 */
#include <glm/ext/matrix_transform.hpp>  /* glm::lookAt                      */

namespace monitor
{

/* Where the camera looks on startup: the mid-Atlantic, chosen so both default
   endpoints (Chicago and London) sit on the visible hemisphere. North is up by
   construction of the basis in reset_view, not by choice of these numbers. */
constexpr float kDefaultViewLatDeg = 25.0f;
constexpr float kDefaultViewLonDeg = -45.0f;

/* Half a turn, for the antipodal case of shortest_arc. */
constexpr float kPiF = 3.14159265f;

/* How close two unit vectors must be, in dot product, before the rotation
   between them is treated as degenerate. Below this margin the cross product is
   numerically worthless as an axis, and normalizing it would yield a NaN. */
constexpr float kParallelEps = 0.999999f;

/*---------------------------------------------------------------------------
  Function: shortest_arc
  Description: The minimal rotation carrying unit vector a onto unit vector b --
               the quaternion at the heart of the grab drag. Written out by hand
               rather than pulled from GLM's experimental extensions, so the
               degenerate cases are guarded HERE, where a NaN would silently
               destroy the camera's orientation.
  Input: a -- unit vector to rotate FROM (normalized by the caller)
         b -- unit vector to rotate ONTO (normalized by the caller)
  Outputs: A unit quaternion q with q * a == b. Identity when the two coincide;
           a well-defined half turn about an arbitrary perpendicular when they
           are opposed. Never a NaN.
---------------------------------------------------------------------------*/
inline glm::quat shortest_arc(
    const glm::vec3& a, /* where the grabbed point currently sits */
    const glm::vec3& b  /* where the cursor wants it to sit       */
)
{
  /* Clamp guards acos: the dot of two unit vectors can land a hair outside
     [-1,1] once rounding is done with it. */
  const float d = std::clamp(glm::dot(a, b), -1.0f, 1.0f);

  /* Already aligned: cross() would be the zero vector, and normalizing that is
     the NaN this whole guard exists to prevent. */
  if (d > kParallelEps)
  {
    return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  }

  /* Antipodal: the axis is genuinely undefined, so ANY perpendicular will do.
     Crossing with X fails only when a IS X-ish, in which case crossing with Y
     cannot also fail -- the two candidates cannot be degenerate together. */
  if (d < -kParallelEps)
  {
    glm::vec3 axis = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), a);
    if (glm::dot(axis, axis) < 1e-6f)
    {
      axis = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), a);
    }
    return glm::angleAxis(kPiF, glm::normalize(axis));
  }

  /* The general case: the axis is the plane normal, the angle is the arc. */
  return glm::angleAxis(std::acos(d), glm::normalize(glm::cross(a, b)));
}

class OrbitCamera
{
 public:
  OrbitCamera() { reset_view(); }

  /*-------------------------------------------------------------------------
    Function: reset_view
    Description: Set the startup orientation -- Earth's north pole (ECEF +Z) UP
                 on screen, looking down at the globe from the default lat/lon.
                 Also the "put it back" hook behind the R key, because an exact
                 grab is free to roll and a long chain of drags can leave north
                 tilted.
    Input: None (uses the kDefaultView* constants).
    Outputs: None; rot_ is replaced and the distance is deliberately left alone
             (a reset re-aims, it does not re-zoom).
  -------------------------------------------------------------------------*/
  void reset_view()
  {
    /* The ECEF point we want facing the camera, from the default lat/lon. */
    const float lat = glm::radians(kDefaultViewLatDeg); /* up from equator  */
    const float lon = glm::radians(kDefaultViewLonDeg); /* east of Greenwich*/
    const glm::vec3 fwd(std::cos(lat) * std::cos(lon),  /* ECEF x           */
                        std::cos(lat) * std::sin(lon),  /* ECEF y           */
                        std::sin(lat));                 /* ECEF z (north)   */

    /* Screen-right is the local "east" there: north x forward. Never
       degenerate, because the default latitude is well away from a pole. */
    const glm::vec3 right =
        glm::normalize(glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), fwd));

    /* Screen-up completes the frame, and with fwd/right as above it comes out
       as the local NORTH direction -- which is the whole requirement. */
    const glm::vec3 up = glm::cross(fwd, right);

    /* A basis whose COLUMNS are (right, up, fwd) maps camera-anchored axes back
       into ECEF, so its transpose is the world->anchored rotation we store. */
    const glm::mat3 anchored_to_world(right, up, fwd);
    rot_ = glm::quat_cast(glm::transpose(anchored_to_world));
  }

  /*-------------------------------------------------------------------------
    Function: zoom
    Description: Scroll changes the viewing distance. Multiplying (rather than
                 adding) makes each tick a constant zoom RATIO, so the step
                 shrinks as you close in -- which is what feels linear to the
                 hand.
    Input: scroll_ticks -- wheel delta, positive to move closer
    Outputs: None; distance_ is clamped into its usable range.
  -------------------------------------------------------------------------*/
  void zoom(float scroll_ticks)
  {
    distance_ *= std::pow(0.9f, scroll_ticks);
    distance_ = std::clamp(distance_, min_dist_, max_dist_);
  }

  void set_distance(float d)
  {
    distance_ = std::clamp(d, min_dist_, max_dist_);
  }

  /*-------------------------------------------------------------------------
    Function: surface_dir_at
    Description: Ray-cast the cursor into the scene and return the direction of
                 the globe point under it -- the primitive BOTH halves of the
                 drag are built from (press anchors one, every frame compares
                 against another). Same eye->sphere algebra as the satellite
                 pick's occlusion test, run the other way: cursor -> world.
    Input: mx, my  -- cursor position in window pixels (top-left origin)
           ww, wh  -- window size in pixels (drives aspect and the NDC map)
           radius  -- Earth radius in WORLD units
           out     -- receives the unit direction of the surface point
    Outputs: true with a unit vector in `out` whenever a direction can be formed
             -- either the real hit, or (cursor past the limb) the silhouette
             point nearest the ray. false, `out` untouched, only in the
             degenerate cases that would produce a NaN: no viewport, a singular
             matrix, or a ray straight through the Earth's center.
  -------------------------------------------------------------------------*/
  bool surface_dir_at(double mx, /* cursor x, window pixels                  */
                      double my, /* cursor y, window pixels (top-left origin) */
                      int ww,    /* window width, for aspect + NDC            */
                      int wh,    /* window height                             */
                      float radius,  /* the globe to intersect, world units  */
                      glm::vec3& out /* result: unit direction of the hit */
  ) const
  {
    if (ww <= 0 || wh <= 0)
    {
      return false; /* no viewport -> no ray to cast */
    }
    const float aspect = static_cast<float>(ww) / static_cast<float>(wh);

    /* Cursor pixels -> NDC. proj() already negates Y for Vulkan, so NDC +Y
       points DOWN like the GLFW cursor: map y straight through with NO flip.
       (The satellite picker relies on exactly this convention; an extra flip
       here would select the vertically mirrored point.) */
    const float nx = static_cast<float>(2.0 * mx / ww - 1.0);
    const float ny = static_cast<float>(2.0 * my / wh - 1.0);

    /* Unproject that pixel's ray. Vulkan depth runs 0..1 under
       GLM_FORCE_DEPTH_ZERO_TO_ONE, hence z = 0 at the near plane, 1 at far. */
    const glm::mat4 inv = glm::inverse(proj(aspect) * view());
    glm::vec4 pn = inv * glm::vec4(nx, ny, 0.0f, 1.0f); /* near point       */
    glm::vec4 pf = inv * glm::vec4(nx, ny, 1.0f, 1.0f); /* far point        */
    if (pn.w == 0.0f || pf.w == 0.0f)
    {
      return false; /* a singular transform: refuse rather than divide by 0 */
    }
    const glm::vec3 o = glm::vec3(pn) / pn.w;       /* ray origin, world    */
    const glm::vec3 far_pt = glm::vec3(pf) / pf.w;  /* ray far end, world   */
    const glm::vec3 d = glm::normalize(far_pt - o); /* unit ray direction   */

    /* Sphere at the origin: |o + t*d|^2 = r^2 -> t^2 + 2bt + c = 0, d unit. */
    const float b = glm::dot(o, d);                   /* half the linear */
    const float c = glm::dot(o, o) - radius * radius; /* constant term   */
    const float disc = b * b - c;                     /* < 0 == a miss   */
    if (disc >= 0.0f)
    {
      /* A real hit: take the NEAR root -- the front face the user can see, and
         therefore the only one they could have meant to grab. */
      const glm::vec3 hit = o + (-b - std::sqrt(disc)) * d;
      out = glm::normalize(hit);
      return true;
    }

    /* MISS FALLBACK (the cursor was dragged past the limb): clamp to the
       silhouette by taking the sphere point nearest the ray -- the ray's
       closest approach to the center, pushed out to the surface. This is
       CONTINUOUS with the real hits at the limb (there the two coincide), so
       the globe neither jumps nor freezes as the cursor crosses the edge; it
       keeps tracking, just along the rim. Beyond the limb the mapping
       saturates, so a far-off cursor stops adding rotation instead of spinning
       the world wildly. */
    const glm::vec3 nearest = o - b * d; /* b == dot(o,d), so this is o+t_c*d */
    if (glm::dot(nearest, nearest) < 1e-12f)
    {
      return false; /* the ray runs through the center: no unique silhouette */
    }
    out = glm::normalize(nearest);
    return true;
  }

  /*-------------------------------------------------------------------------
    Function: begin_grab
    Description: Pin the surface point the user just pressed on. Everything the
                 drag needs is captured here: the grabbed direction (expressed
                 in the camera-anchored frame, where it stays constant) and the
                 orientation at press time, which every later frame rotates
                 from. Call it again to RE-anchor -- after a zoom mid-drag, say,
                 which changes the pixel->world mapping under the cursor.
    Input: world_dir -- unit vector of the grabbed point, in WORLD (ECEF) space,
                        as returned by surface_dir_at
    Outputs: None; grab_anchor_ and rot_at_press_ are set, grabbing_ goes true.
  -------------------------------------------------------------------------*/
  void begin_grab(const glm::vec3& world_dir)
  {
    grab_anchor_ = rot_ * world_dir; /* world -> camera-anchored frame       */
    rot_at_press_ = rot_;            /* the orientation all deltas build on  */
    grabbing_ = true;
  }

  /*-------------------------------------------------------------------------
    Function: update_grab
    Description: Drag one frame: rotate the world so the point grabbed at press
                 time sits under the cursor NOW.

                 Because the anchored frame is welded to the camera, world_dir
                 mapped into it depends only on the cursor PIXEL -- not on the
                 orientation we are about to overwrite -- so composing from
                 rot_at_press_ is exact and drift-free, rather than an
                 accumulation of per-frame deltas that would slowly wander.

                 The direction inversion the user asked for falls out for free:
                 we move the grabbed point TO the cursor, so there is no sign to
                 pick.
    Input: world_dir -- unit vector under the cursor this frame, in WORLD space,
                        cast with the CURRENT view (surface_dir_at)
    Outputs: None; rot_ is updated. A no-op when no grab is active.
  -------------------------------------------------------------------------*/
  void update_grab(const glm::vec3& world_dir)
  {
    if (!grabbing_)
    {
      return;
    }
    const glm::vec3 target = rot_ * world_dir; /* cursor dir, anchored frame */

    /* q carries the anchored grab point onto the cursor; applying it on the
       LEFT of the press orientation rotates the world in the CAMERA's frame,
       which is the frame both of these vectors live in. */
    rot_ = glm::normalize(shortest_arc(grab_anchor_, target) * rot_at_press_);
  }

  void end_grab() { grabbing_ = false; }
  bool grabbing() const { return grabbing_; }

  /*-------------------------------------------------------------------------
    Function: eye
    Description: The camera's position in WORLD space. It sits on the anchored
                 +Z axis, so undoing the world rotation says where that lands in
                 ECEF. The satellite picker needs it for its occlusion rays.
    Input: None.
    Outputs: The eye position, world units.
  -------------------------------------------------------------------------*/
  glm::vec3 eye() const
  {
    return glm::inverse(rot_) * glm::vec3(0.0f, 0.0f, distance_);
  }

  /*-------------------------------------------------------------------------
    Function: view
    Description: The fixed anchored camera, times the world rotation the drag
                 manipulates. lookAt's up is the anchored +Y, so "up on screen"
                 is whatever rot_ maps there -- ECEF north, at startup.
    Input: None.
    Outputs: The view matrix.
  -------------------------------------------------------------------------*/
  glm::mat4 view() const
  {
    const glm::mat4 anchored =
        glm::lookAt(glm::vec3(0.0f, 0.0f, distance_), glm::vec3(0.0f),
                    glm::vec3(0.0f, 1.0f, 0.0f));
    return anchored * glm::mat4_cast(rot_);
  }

  /*-------------------------------------------------------------------------
    Function: proj
    Description: The projection. Vulkan clip space has depth 0..1
                 (GLM_FORCE_DEPTH_ZERO_TO_ONE) and Y pointing DOWN, so [1][1] is
                 negated here rather than flipping the viewport -- which keeps
                 the pick math and the GPU sharing one transform.
    Input: aspect -- viewport width / height
    Outputs: The projection matrix.
  -------------------------------------------------------------------------*/
  glm::mat4 proj(float aspect) const
  {
    glm::mat4 p = glm::perspective(glm::radians(50.0f), aspect, near_, far_);
    p[1][1] *= -1.0f;
    return p;
  }

 private:
  glm::quat rot_{1.0f, 0.0f, 0.0f, 0.0f}; /* world -> camera-anchored frame  */
  glm::quat rot_at_press_{1.0f, 0.0f, 0.0f, 0.0f}; /* rot_ when a drag began */
  glm::vec3 grab_anchor_{0.0f, 0.0f, 1.0f}; /* grabbed point, anchored frame */
  bool grabbing_ = false;   /* a left-drag is in progress                    */
  float distance_ = 25.0f;  /* world units; Earth is ~6.378 at 1/1000 scale  */
  float min_dist_ = 7.0f;   /* just above the surface: no clipping inside    */
  float max_dist_ = 120.0f; /* far enough that the globe is still legible    */
  float near_ = 0.1f;       /* tight, to keep depth precision on the mesh    */
  float far_ = 1000.0f;     /* comfortably past the constellation shell      */
};

} /* namespace monitor */

#endif /* LEO_MONITOR_CAMERA_HPP */
