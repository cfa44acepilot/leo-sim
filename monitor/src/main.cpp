/*****************************************************************************
  filename main.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator (monitor)
  date 2026-07-14
  Brief Description:
    The `leo_monitor` front end: drives a leo::Simulator and visualizes one
    Snapshot per frame.

    It obeys the simulator's three tiers, and the ORDER matters: the dataset is
    loaded once (Tier 1 -- the only place NodeIds are assigned), the endpoints
    are placed (Tier 2), and only then is time advanced (Tier 3). Anything that
    re-runs Tier 1 renumbers every NodeId, so every piece of state keyed by one
    (the selection, the route, the plane mask) must be invalidated with it --
    which is why the shell switch is an explicit rebuild rather than a poke.

    This file owns the window, the camera, the input, and the frame loop. All
    Vulkan lives in VulkanRenderer, all geometry conversion in scene.cpp, and
    nothing in src/ is touched: the simulator is consumed strictly read-only.
 *****************************************************************************/

#include <algorithm>   /* std::find/min/max: route lookups, clamps           */
#include <cmath>       /* std::sqrt: the Earth-occlusion ray test in picking */
#include <cstddef>     /* std::size_t                                        */
#include <cstdio>      /* std::printf: the headless reports                  */
#include <cstdlib>     /* std::atoi: flag parsing                            */
#include <cstring>     /* std::strcmp: flag parsing                          */
#include <exception>   /* std::exception: a malformed UTC entry must not kill
                          the frame loop -- it becomes an inline error        */
#include <string>      /* names, paths, messages                             */
#include <vector>      /* the geometry and mask buffers                      */

/* GLFW needs to know Vulkan exists before it is included, so that it declares
   the surface entry points this file's window creation relies on. */
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>  /* window, input callbacks, event loop              */

#include <glm/geometric.hpp>  /* distance/dot/length: picking + link report  */
#include <glm/vec3.hpp>       /* the grab direction handed to the camera     */
#include <glm/vec4.hpp>       /* clip-space vec4 for the satellite picker    */

#include "imgui.h"              /* the panels                                */
#include "imgui_impl_glfw.h"    /* their input                               */
#include "imgui_impl_vulkan.h"  /* their draw pass                           */

#include "camera.hpp"        /* OrbitCamera: the grab-the-surface camera     */
#include "data_update.hpp"   /* DataUpdater: the background "Update data"    */
#include "epoch_health.hpp"  /* now_jd_utc: where the sim clock starts       */
#include "omm_loader.hpp"    /* iso8601_to_jd / jd_to_iso8601: the time UI   */
#include "route_diag.hpp"    /* --diag-route: router graph vs drawn links    */
#include "scene.hpp"         /* the CPU-side geometry builders               */
#include "simulator.hpp"     /* the thing being visualized                   */
#include "vk_renderer.hpp"   /* the thing doing the visualizing              */

namespace
{

// km -> world units. Earth radius ~6378 km becomes ~6.378 world units, a
// comfortable scale for the orbit camera's near/far range.
constexpr float kKmScale = 1.0f / 1000.0f;

// The Earth's radius in those world units -- the sphere both the camera grab
// (cursor -> surface point) and the satellite pick (occlusion test) cast
// against.
constexpr float kEarthRadiusWorld = 6378.137f * kKmScale;

// Sim seconds advanced per real second, as the speed slider's starting point.
// Satellites complete an orbit in ~95 min, so ~60x makes the motion visible
// without being dizzying. The slider spans a creep to a whole orbit per second.
constexpr float kDefaultSpeed = 60.0f;
constexpr float kMinSpeed = 0.1f;
constexpr float kMaxSpeed = 10000.0f;

// Half-width of the settable-time window around the newest element epoch. SGP4
// is a near-epoch model: its accuracy decays in BOTH directions, so the control
// is fenced to +/- this many days rather than letting the operator scrub to an
// arbitrary instant and read positions that quietly stopped meaning anything.
constexpr double kTimeWindowDays = 3.0;

// Exponential moving average of the real frame rate. Fed the wall-clock frame
// delta, so it measures the renderer and is unaffected by the sim speed or by
// pause -- a paused sim still draws frames, and the readout should say so.
struct FpsMeter
{
  double fps = 0.0;

  void sample(double dt_real_s)
  {
    if (dt_real_s <= 0.0)
    {
      return;  // a zero/backwards delta would divide by 0
    }
    const double instant = 1.0 / dt_real_s;
    // Low alpha: a single slow frame nudges the readout instead of spiking it.
    constexpr double kAlpha = 0.1;
    fps = fps > 0.0 ? fps + kAlpha * (instant - fps) : instant;
  }
};

// The ONE place simulated time moves. Pause gates advance(), speed scales it,
// and jump_to() sets it outright. Every control writes t_jd through this
// struct, so pause / speed / set-time can never disagree about what "now" is,
// and the frame loop keeps a single `sim.step(clock.t_jd)` call.
//
// jump_to is a TIER-3 operation: it changes only the propagation target. The
// next step() re-propagates, re-seeds links and re-routes to that instant. It
// must never trigger load()/build_topology(), which would renumber every NodeId
// and desync the selection, the route and the highlight.
struct SimClock
{
  double t_jd = 0.0;            // current simulated instant (Julian Date, UTC)
  bool paused = false;          // freezes advancement; jumps still apply
  float speed = kDefaultSpeed;  // real-time multiplier (1.0 = wall clock)

  void advance(double dt_real_s)
  {
    if (paused)
    {
      return;  // pause gates the clock, not the renderer
    }
    t_jd += dt_real_s * static_cast<double>(speed) / 86400.0;  // s -> days
  }

  void jump_to(double jd) { t_jd = jd; }
};

// Scratch state for the UTC entry field: the text the operator is typing and
// the parse error (if any) to show beneath it. Kept out of SimClock, which is
// the time itself and should carry no UI state.
struct TimeUi
{
  char iso[40] = "";  // "YYYY-MM-DDTHH:MM:SS" being typed
  std::string error;  // last parse failure, shown inline; empty = fine
};

// Mouse/scroll state threaded through GLFW's user pointer so the static
// callbacks can reach the camera.
struct InputState
{
  monitor::OrbitCamera camera;
  bool dragging = false;  // the cursor has moved past kClickSlopPx
  double last_x = 0.0;
  double last_y = 0.0;
  double press_x = 0.0;  // cursor at the last left-press (click vs drag)
  double press_y = 0.0;
  bool press_in_view = false;  // press started over the viewport, not the UI
  bool click_pending = false;  // a click (not a drag) awaits pick handling
};

// A left press+release within this many pixels is a CLICK (select a satellite);
// past it the same gesture is a camera grab. One threshold decides both, so the
// two can never fire for one gesture: `dragging` latches on when it is crossed,
// and a latched drag suppresses the click on release.
constexpr double kClickSlopPx = 5.0;

// True when ImGui owns the mouse (pointer over a UI window), so the camera
// should ignore it. These callbacks are chained after ImGui's own (installed by
// ImGui_ImplGlfw_InitForVulkan), so ImGui has already updated its state.
bool ui_wants_mouse()
{
  return ImGui::GetCurrentContext() && ImGui::GetIO().WantCaptureMouse;
}

// -----------------------------------------------------------------------------
// Function:    grab_dir_at_cursor
// Description: Ray-cast the cursor onto the globe for the drag, feeding the
//              camera the window size it cannot know on its own.
// Input:       w       -- the window (queried for its pixel size)
//              in      -- input state (camera + cursor position)
//              out     -- receives the unit direction of the grabbed point
// Outputs:     true when a direction was produced (a hit, or the silhouette
//              clamp past the limb); false in the degenerate cases, where the
//              caller must simply not rotate rather than apply a NaN.
// -----------------------------------------------------------------------------
bool grab_dir_at_cursor(GLFWwindow* w, InputState* in, glm::vec3& out)
{
  int ww = 0, wh = 0; /* window size drives the NDC map   */
  glfwGetWindowSize(w, &ww, &wh);
  return in->camera.surface_dir_at(in->last_x, in->last_y, ww, wh,
                                   kEarthRadiusWorld, out);
}

void cursor_cb(GLFWwindow* w, double x, double y)
{
  auto* in = static_cast<InputState*>(glfwGetWindowUserPointer(w));
  in->last_x = x;
  in->last_y = y;
  if (!in->press_in_view)
  {
    return;  // no button down in the viewport -> no drag
  }
  // Latch into a drag the moment the gesture outgrows the click threshold. The
  // grab was anchored back at the PRESS point, so the rotation still carries
  // the originally-pressed surface point to the cursor -- the slop is only a
  // gate on WHEN we start rotating, never a shift of what was grabbed.
  const double dx = x - in->press_x;
  const double dy = y - in->press_y;
  if (!in->dragging && dx * dx + dy * dy >= kClickSlopPx * kClickSlopPx)
  {
    in->dragging = true;
  }
  glm::vec3 dir(0.0f); /* globe point under the cursor now */
  if (in->dragging && in->camera.grabbing() && grab_dir_at_cursor(w, in, dir))
  {
    in->camera.update_grab(dir);
  }
}

void button_cb(GLFWwindow* w, int button, int action, int /*mods*/)
{
  if (button != GLFW_MOUSE_BUTTON_LEFT)
  {
    return;
  }
  auto* in = static_cast<InputState*>(glfwGetWindowUserPointer(w));
  if (action == GLFW_PRESS)
  {
    in->press_in_view = !ui_wants_mouse();  // ignore presses that start on UI
    in->dragging = false;                   // a press is a click until it moves
    in->press_x = in->last_x;
    in->press_y = in->last_y;
    // Anchor the grab AT THE PRESS, before we know whether this becomes a drag:
    // by the time the threshold is crossed the cursor has already left the
    // pressed point, and anchoring there instead would silently grab the wrong
    // spot. A press over empty sky still anchors (on the silhouette), so the
    // globe follows the cursor rather than ignoring the drag.
    glm::vec3 dir(0.0f); /* the point being grabbed       */
    if (in->press_in_view && grab_dir_at_cursor(w, in, dir))
    {
      in->camera.begin_grab(dir);
    }
  }
  else if (action == GLFW_RELEASE)
  {
    // Only a gesture that never became a drag selects. `dragging` is the same
    // latch the cursor callback set, so releasing after a drag cannot also
    // select the satellite that happens to sit under the cursor at the end.
    if (in->press_in_view && !in->dragging)
    {
      in->click_pending = true;
    }
    in->camera.end_grab();
    in->dragging = false;
    in->press_in_view = false;
  }
}

void scroll_cb(GLFWwindow* w, double /*dx*/, double dy)
{
  if (ui_wants_mouse())
  {
    return;
  }
  auto* in = static_cast<InputState*>(glfwGetWindowUserPointer(w));
  in->camera.zoom(static_cast<float>(dy));
  // Zooming mid-drag moves the world under the cursor, so the anchored grab
  // point no longer matches the pixel the user is holding. Re-anchor on the
  // point now under the cursor: the grab stays exact from here on instead of
  // snapping back to a stale one on the next mouse move.
  glm::vec3 dir(0.0f); /* the re-grabbed point           */
  if (in->camera.grabbing() && grab_dir_at_cursor(w, in, dir))
  {
    in->camera.begin_grab(dir);
  }
}

// The "RAAN groups" window: one checkbox per RAAN bin. These bins are clustered
// by fixed RAAN band, NOT guaranteed to be a single co-orbital ring (RAAN
// precesses across epochs on live data), so the UI deliberately avoids the word
// "plane" -- see the future-work note in topology.hpp. Toggling a box shows or
// hides that group's satellites and edges (the filter is applied when the scene
// geometry is rebuilt each frame). plane_on[p] != 0 means group p is visible.
//
// `route_only` is a mode toggle: when on, the caller drives visibility from the
// current route (only groups carrying a path satellite) instead of plane_on, so
// the manual controls are disabled but plane_on is left intact -- unchecking
// restores the manual selection exactly. `have_route` just annotates the empty
// case so the operator knows a blank globe means "no route", not a stuck
// filter.
void draw_planes_panel(std::vector<std::uint8_t>& plane_on, bool& route_only,
                       bool have_route)
{
  ImGui::Begin("RAAN groups");
  ImGui::Text("%zu RAAN groups", plane_on.size());

  ImGui::Checkbox("Only groups on route", &route_only);
  if (route_only && !have_route)
  {
    ImGui::TextDisabled("(no route now -> nothing shown)");
  }
  ImGui::Separator();

  // Manual controls are inert while the route drives the filter.
  ImGui::BeginDisabled(route_only);
  if (ImGui::Button("All"))
  {
    std::fill(plane_on.begin(), plane_on.end(), std::uint8_t{1});
  }
  ImGui::SameLine();
  if (ImGui::Button("None"))
  {
    std::fill(plane_on.begin(), plane_on.end(), std::uint8_t{0});
  }

  // A compact grid of checkboxes inside a scroll region (72 groups fit easily).
  const int cols = 4;
  if (ImGui::BeginTable("group_grid", cols))
  {
    for (std::size_t p = 0; p < plane_on.size(); ++p)
    {
      ImGui::TableNextColumn();
      bool on = plane_on[p] != 0;
      char label[16];
      std::snprintf(label, sizeof(label), "G%zu", p);
      if (ImGui::Checkbox(label, &on))
      {
        plane_on[p] = on ? std::uint8_t{1} : std::uint8_t{0};
      }
    }
    ImGui::EndTable();
  }
  ImGui::EndDisabled();
  ImGui::End();
}

// The "Links" window. `only_routable` toggles the CROSS-PLANE/uplink view
// (feasible vs the full designed lattice from Snapshot.edges). The blue
// in-group links have no such toggle any more: they are read straight from ISL
// slots 0/1, which the core seeds geometrically each tick, so every blue line
// IS a router edge -- there is no infeasible version of them left to show. The
// yellow route is never filtered.
void draw_links_panel(bool& only_routable, bool& show_ingroup, bool& show_route)
{
  ImGui::Begin("Links");
  ImGui::Checkbox("Show only routable links", &only_routable);
  ImGui::TextDisabled(only_routable ? "cross-plane: feasible (router-usable)"
                                    : "cross-plane: full designed lattice");
  ImGui::Separator();
  ImGui::Checkbox("Show in-group (blue) links", &show_ingroup);
  ImGui::TextDisabled("blue = in-plane ISL slots 0/1 (router-usable)");
  ImGui::Separator();
  // Visibility of the gold route line only: the route is still computed and
  // still reported by the Route status panel while this is off, so unchecking
  // it is a way to SEE the mesh the route is drawn over, not a way to stop
  // routing.
  ImGui::Checkbox("Show optimal route", &show_route);
  ImGui::TextDisabled("route is still computed while hidden");
  ImGui::End();
}

// The "Endpoints" window: two dropdowns choosing the source and destination
// ground stations from the loaded catalog. `sel_src`/`sel_dst` are indices into
// `stations`; the caller seeds them from the startup endpoints so the default
// view is unchanged, and re-places the endpoints (Tier 2) only when this
// returns true -- i.e. the user actually picked a different station in either
// combo.
bool draw_endpoints_panel(const std::vector<leo::GroundStation>& stations,
                          int& sel_src, int& sel_dst)
{
  bool changed = false;
  ImGui::Begin("Endpoints");
  if (stations.empty())
  {  // no catalog loaded -> nothing to pick
    ImGui::TextDisabled("(no stations loaded)");
    ImGui::End();
    return false;
  }

  // One combo implementation shared by Source and Destination: the preview is
  // the current pick; clicking a different row moves the selection and flags
  // the change so the caller re-routes. Names come straight from the catalog.
  auto combo = [&](const char* label, int& sel)
  {
    if (ImGui::BeginCombo(label, stations[sel].name.c_str()))
    {
      for (int i = 0; i < static_cast<int>(stations.size()); ++i)
      {
        const bool is_sel = (i == sel);
        if (ImGui::Selectable(stations[i].name.c_str(), is_sel) && i != sel)
        {
          sel = i;
          changed = true;
        }
        if (is_sel)
        {
          ImGui::SetItemDefaultFocus();  // keyboard opens on the pick
        }
      }
      ImGui::EndCombo();
    }
  };
  combo("Source", sel_src);
  combo("Destination", sel_dst);

  ImGui::End();
  return changed;
}

// The "Shell" window: a dropdown selecting which single inclination shell the
// simulator models. `sel` is the current preset index; on a change this returns
// the NEW index (the caller runs the Tier-1 rebuild) and otherwise -1. Showing
// the live satellite count makes a sparse shell (polar, 70-deg) obviously
// smaller than the 53-deg main -- exactly the point of the selector. Only the
// SELECTION is decided here; switching shells is far too heavy (a full reload +
// re-cluster + renumber) to do anywhere but a deliberate rebuild path.
int draw_shell_panel(const std::vector<leo::ShellPreset>& shells, int sel,
                     std::size_t sat_count, std::size_t group_count)
{
  int changed = -1;  // -1 = no change; otherwise the newly chosen index
  ImGui::Begin("Shell");
  if (ImGui::BeginCombo("Shell", std::string(shells[sel].name).c_str()))
  {
    for (int i = 0; i < static_cast<int>(shells.size()); ++i)
    {
      const bool is_sel = (i == sel);
      if (ImGui::Selectable(std::string(shells[i].name).c_str(), is_sel) &&
          i != sel)
      {
        changed = i;
      }
      if (is_sel)
      {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  // The resulting size of the CURRENT shell, so switching to the polar shell
  // visibly collapses the count (and the operator knows it is expected).
  ImGui::Text("%zu satellites, %zu RAAN groups", sat_count, group_count);
  ImGui::TextDisabled("switching reloads + re-clusters (Tier 1)");
  ImGui::End();
  return changed;
}

// Blue in-group link diagnostic (instruction33). For one satellite, print its
// identity/group/velocity-direction, EVERY same-group satellite within
// max_range_km sorted by distance (with earth_clear, same-direction, and
// whether this satellite chose it fore/aft), the two it actually mutually
// linked, and its total blue-link count. This makes every "missing" blue link
// explainable: a close candidate that did not link shows exactly why (failed
// earth_clear, wrong direction, a closer pick took the terminal, or the pick
// was not reciprocated).
void print_sat_link_report(const leo::Constellation& c,
                           const leo::LinkSpec& spec, leo::NodeId node,
                           std::FILE* out)
{
  if (node >= c.num_satellites)
  {  // ground node or out of range: nothing to say
    std::fprintf(out, "[sat] node %u is not a satellite\n",
                 static_cast<unsigned>(node));
    return;
  }
  // The picks the CORE seeding made, from the core itself -- so the report, the
  // drawn links, and the router's edges are all the same decision, not three
  // re-derivations of it that can drift apart (they did: see instructions41).
  std::vector<leo::NodeId> pick_fore; /* per-sat fore pick */
  std::vector<leo::NodeId> pick_aft;  /* per-sat aft pick  */
  leo::compute_in_plane_picks(c, spec, pick_fore, pick_aft);
  const auto picked = [&](leo::NodeId a, leo::NodeId b)
  {
    return pick_fore[a] == b || pick_aft[a] == b;  // a aimed a terminal at b
  };

  const leo::Vec3& pi = c.position_ecef[node];  // this satellite's position
  const leo::Vec3& vi = c.velocity_eci[node];   // its velocity (TEME direction)
  const double vlen = glm::length(vi);          // guard a zero (dead) velocity
  const glm::dvec3 vdir = vlen > 0.0 ? glm::dvec3(vi / vlen) : glm::dvec3(0.0);
  const std::uint16_t grp = c.grid_coord[node].plane;  // RAAN group index
  std::fprintf(out, "[sat] node=%u cat=%u group=%u vel_dir=(%.3f,%.3f,%.3f)\n",
               static_cast<unsigned>(node),
               static_cast<unsigned>(c.catalog_id[node]),
               static_cast<unsigned>(grp), vdir.x, vdir.y, vdir.z);

  // Every same-group satellite within range, so the listing is exactly the set
  // the picker considered (candidates farther than max_range never link).
  struct Cand
  {
    leo::NodeId j; /* candidate node id      */
    double dist;   /* straight-line km       */
    bool clear;    /* passes earth_clear     */
    bool same_dir; /* velocity dot > min_vel */
  };
  std::vector<Cand> cands;
  const leo::NodeId begin = c.plane_offsets[grp];
  const leo::NodeId end = c.plane_offsets[grp + 1];
  for (leo::NodeId j = begin; j < end; ++j)
  {
    if (j == node)
    {
      continue;  // not its own candidate
    }
    const leo::Vec3& pj = c.position_ecef[j];
    const double dist = glm::distance(pi, pj);
    if (dist > spec.max_range_km)
    {
      continue;  // out of range -> never a candidate
    }
    cands.push_back({j, dist, leo::earth_clear(pi, pj, spec.block_radius_km),
                     glm::dot(vi, c.velocity_eci[j]) > spec.min_vel_dot});
  }
  std::sort(cands.begin(), cands.end(),
            [](const Cand& a, const Cand& b) { return a.dist < b.dist; });

  std::fprintf(out,
               "[sat] same-group candidates <= %.0f km "
               "(node dist clear same_dir chosen):\n",
               spec.max_range_km);
  for (const Cand& cd : cands)
  {
    // "chosen" says which of this satellite's two terminals (if any) went to
    // cd.
    const char* chosen = pick_fore[node] == cd.j  ? "fore"
                         : pick_aft[node] == cd.j ? "aft"
                                                  : "-";
    std::fprintf(out, "[sat]   %u %.0fkm clear=%s same_dir=%s chosen=%s\n",
                 static_cast<unsigned>(cd.j), cd.dist, cd.clear ? "y" : "n",
                 cd.same_dir ? "y" : "n", chosen);
  }

  // A pick becomes a drawn blue link only if the neighbor picked back (mutual);
  // count those to confirm the degree cap on this exact satellite.
  int blue = 0; /* mutual links = drawn blue lines for this satellite */
  const auto show_pick = [&](const char* label, leo::NodeId j)
  {
    if (j == leo::INVALID_NODE)
    {
      std::fprintf(out, "[sat] %s pick: none\n", label);
      return;
    }
    const bool mutual = picked(j, node);  // did the neighbor pick back?
    const double dist = glm::distance(pi, c.position_ecef[j]);
    std::fprintf(out, "[sat] %s pick: node=%u %.0fkm mutual=%s\n", label,
                 static_cast<unsigned>(j), dist, mutual ? "y -> LINK" : "no");
    if (mutual)
    {
      ++blue;
    }
  };
  show_pick("fore", pick_fore[node]);
  show_pick("aft", pick_aft[node]);
  std::fprintf(out, "[sat] blue-link count = %d (cap 2)\n", blue);
}

// The "Data" section of the Simulation panel: one button that refreshes the
// on-disk snapshot in the background. It updates the FILE ONLY -- reloading it
// into the running sim would be Tier 1 (load + build_topology), which reorders
// the dense arrays and renumbers every NodeId beneath the selection, the route
// and the highlight. So the scene deliberately keeps showing the old data and
// the success message says, plainly, to restart.
//
// The button is disabled while a fetch is in flight and the result is polled
// from the frame loop, so a multi-second network call never freezes the UI.
void draw_data_section(monitor::DataUpdater& updater, const std::string& omm,
                       const std::string& fetcher)
{
  using State = monitor::DataUpdater::State;
  const ImVec4 good(0.45f, 0.85f, 0.5f, 1.0f);  // green: the file was rewritten
  const ImVec4 bad(0.95f, 0.35f, 0.3f, 1.0f);   // red: fetch tool said no

  ImGui::SeparatorText("Data");
  ImGui::BeginDisabled(updater.busy());  // no second download over the first
  if (ImGui::Button("Update data"))
  {
    updater.start(omm, fetcher);
  }
  ImGui::EndDisabled();
  if (updater.busy())
  {
    ImGui::SameLine();
    ImGui::TextDisabled("fetching...");
  }

  // One status line, colored by outcome. Success stays on screen (the restart
  // instruction is the point); failure explains itself and the button is live
  // again for a retry.
  if (updater.state() == State::kIdle)
  {
    return;
  }
  ImGui::PushTextWrapPos(0.0f);  // long sentences; TextColored will not wrap
  switch (updater.state())
  {
    case State::kFetching:
      ImGui::TextDisabled("%s", updater.message().c_str());
      break;
    case State::kDone:
      ImGui::TextColored(good, "%s", updater.message().c_str());
      break;
    case State::kFailed:
      ImGui::TextColored(bad, "%s", updater.message().c_str());
      break;
    case State::kIdle:
      break;  // returned above; listed so a new state cannot compile silently
  }
  ImGui::PopTextWrapPos();
}

// The "Simulation" window: frame rate, the pause/speed controls, and the
// set-time control. Every mutation goes through `clock` (the single time
// source); this panel never advances time itself. A jump made here lands on the
// NEXT step(), which re-propagates and re-routes to the chosen instant -- the
// dots, links, route line and selection ring all read live state per frame, so
// they follow with no extra bookkeeping.
//
// Given an explicit first-use spot (top-left) so the window can't open hidden
// behind the other panels; the user can move it and imgui.ini remembers.
void draw_sim_panel(SimClock& clock, TimeUi& ui, double fps,
                    const leo::Snapshot& s, monitor::DataUpdater& updater,
                    const std::string& omm, const std::string& fetcher)
{
  const ImVec4 warn(0.95f, 0.6f, 0.2f, 1.0f);  // amber: degraded but usable
  const ImVec4 bad(0.95f, 0.35f, 0.3f, 1.0f);  // red: outside the valid window

  ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(340.0f, 0.0f), ImGuiCond_FirstUseEver);
  ImGui::Begin("Simulation");

  // Frame rate first: it is the one number that describes the app rather than
  // the model, and it stays live while paused.
  ImGui::Text("%.1f fps  (%.1f ms/frame)", fps, fps > 0.0 ? 1000.0 / fps : 0.0);
  ImGui::Separator();

  ImGui::Checkbox("Pause", &clock.paused);
  ImGui::SameLine();
  ImGui::TextDisabled(clock.paused ? "(time frozen)" : "(running)");

  // Logarithmic so the low decades (0.1x .. 10x) get as much slider travel as
  // the high ones -- a linear 0.1..10000 slider would make everything under
  // ~100x unreachable in a pixel or two of grab.
  ImGui::SliderFloat("##speed", &clock.speed, kMinSpeed, kMaxSpeed, "%.2fx",
                     ImGuiSliderFlags_Logarithmic);
  ImGui::SameLine();
  if (ImGui::Button("Reset"))
  {
    clock.speed = kDefaultSpeed;
  }
  // Read out the multiplier in words; while paused it is inert, and saying so
  // is cheaper than the operator wondering why the globe is still.
  if (clock.paused)
  {
    ImGui::TextDisabled("speed: %.2gx real time (paused -- no effect)",
                        static_cast<double>(clock.speed));
  }
  else if (clock.speed >= 1.0f)
  {
    ImGui::Text("speed: %.0fx real time", static_cast<double>(clock.speed));
  }
  else
  {
    ImGui::Text("speed: %.2fx real time", static_cast<double>(clock.speed));
  }

  ImGui::Separator();
  ImGui::Text("UTC %s", s.utc_iso.c_str());

  // With no epochs there is no window to bound against, so the time control has
  // nothing meaningful to clamp to. The Data section still shows: an empty
  // snapshot is exactly when a fetch is most warranted.
  if (s.epoch_health.empty)
  {
    ImGui::TextDisabled("(no element epochs -> time not settable)");
    draw_data_section(updater, omm, fetcher);
    ImGui::End();
    return;
  }

  // The window is anchored on the NEWEST epoch: that is the element set whose
  // accuracy decays slowest away from `now`, and it is what the staleness check
  // already measures against.
  const double newest = s.epoch_health.max_jd;
  const double lo = newest - kTimeWindowDays;  // earliest settable instant
  const double hi = newest + kTimeWindowDays;  // latest settable instant
  const auto jump_clamped = [&](double jd)
  {
    clock.jump_to(std::clamp(jd, lo, hi));  // the fence, applied on every path
  };

  // Offset form rather than absolute JD: "how far from the epoch" is the number
  // that governs whether the positions can be trusted. Re-read from the clock
  // every frame, so the grab tracks free-running time and a jump made by the
  // buttons below moves the slider too. ImGui writes back only on a real edit,
  // so a t_jd outside the window pins the grab at an edge without jumping.
  float offset = static_cast<float>(clock.t_jd - newest);
  if (ImGui::SliderFloat("days from epoch", &offset,
                         static_cast<float>(-kTimeWindowDays),
                         static_cast<float>(kTimeWindowDays), "%+.3f d"))
  {
    jump_clamped(newest + static_cast<double>(offset));
  }

  if (ImGui::Button("Now"))
  {
    jump_clamped(leo::now_jd_utc());
  }
  ImGui::SameLine();
  if (ImGui::Button("Newest epoch"))
  {
    jump_clamped(newest);
  }
  // Spell the fence out: "days from epoch" only means something once the
  // operator can see which calendar instants the two ends stand for. Trim the
  // fractional seconds jd_to_iso8601 appends -- they are noise at this scale.
  ImGui::TextDisabled("window %s .. %s",
                      leo::jd_to_iso8601(lo).substr(0, 19).c_str(),
                      leo::jd_to_iso8601(hi).substr(0, 19).c_str());

  // Exact entry, for reproducing a specific instant. The parser throws on
  // malformed input (the same one behind the CLI's --time), so a typo yields an
  // inline message instead of an exception escaping into the frame loop.
  ImGui::InputTextWithHint("##utc", "YYYY-MM-DDTHH:MM:SS", ui.iso,
                           sizeof(ui.iso));
  ImGui::SameLine();
  if (ImGui::Button("Go"))
  {
    try
    {
      jump_clamped(leo::iso8601_to_jd(ui.iso));
      ui.error.clear();
    }
    catch (const std::exception& e)
    {
      ui.error = e.what();
    }
  }
  if (!ui.error.empty())
  {
    ImGui::TextColored(bad, "%s", ui.error.c_str());
  }

  // Trust line. Distance from the newest epoch is symmetric -- scrubbing three
  // days BACKWARD degrades SGP4 exactly as three days forward does, even though
  // the core's newest_age_days goes negative there and its propagation_ok flag
  // (a one-sided staleness test) stays true. So gate the loudest warning on the
  // absolute offset, and still surface the core's own staleness verdict.
  const double offset_days = std::fabs(clock.t_jd - newest);
  ImGui::PushTextWrapPos(0.0f);  // TextColored does not wrap on its own
  if (offset_days > kTimeWindowDays)
  {
    ImGui::TextColored(bad,
                       "%.2f days from the newest epoch -- OUTSIDE the +/-%.0f "
                       "day window; these positions are not trustworthy.",
                       offset_days, kTimeWindowDays);
  }
  else if (!s.epoch_health.propagation_ok)
  {
    ImGui::TextColored(warn,
                       "stale snapshot: the newest element is %.1f days old; "
                       "use Update data below to fetch a fresh one.",
                       s.epoch_health.newest_age_days);
  }
  else if (offset_days > kTimeWindowDays * 0.8)
  {
    ImGui::TextColored(warn,
                       "%.2f days from the newest epoch -- near the edge of "
                       "the valid window; positions are degrading.",
                       offset_days);
  }
  ImGui::PopTextWrapPos();

  draw_data_section(updater, omm, fetcher);
  ImGui::End();
}

// The selected-satellite info window. Reads the constellation/Snapshot live so
// the numbers track the running sim; the highlight and this window key off the
// same NodeId. Closing the window (the [x]) deselects. All reads are const.
void draw_selected_info_panel(const leo::Constellation& c,
                              const leo::Snapshot& s, const leo::LinkSpec& spec,
                              leo::NodeId& selected)
{
  if (selected == leo::INVALID_NODE)
  {
    return;  // nothing selected -> no window
  }
  bool open = true;  // ImGui clears it on the [x]
  ImGui::Begin("Satellite", &open);
  if (selected < c.num_satellites)
  {
    const leo::CatalogId cat = c.catalog_id[selected];
    const leo::GridCoord gc = c.grid_coord[selected];
    const leo::Vec3& p = c.position_ecef[selected];
    const double altitude = glm::length(p) - 6378.137;  // |r| - Re, km

    ImGui::Text("catalog %u   node %u", static_cast<unsigned>(cat),
                static_cast<unsigned>(selected));
    ImGui::Text("RAAN group %u   slot %u", static_cast<unsigned>(gc.plane),
                static_cast<unsigned>(gc.slot));
    if (selected < s.position_valid.size() && s.position_valid[selected] == 0)
    {
      ImGui::TextColored(ImVec4(0.95f, 0.4f, 0.3f, 1.0f),
                         "position invalid (failed propagation)");
    }
    ImGui::Separator();
    ImGui::Text("ecef  x %.1f  y %.1f  z %.1f km", p.x, p.y, p.z);
    ImGui::Text("altitude %.1f km", altitude);

    // Cold orbital elements via the public by_catalog map (read-only accessor).
    const leo::Satellite& sat = c.by_catalog.at(cat);
    ImGui::Separator();
    ImGui::Text("incl %.2f deg   RAAN %.2f deg", sat.inclination, sat.raan);
    ImGui::Text("ecc %.5f   mean motion %.4f rev/day", sat.eccentricity,
                sat.mean_motion);

    const bool on_route =
        std::find(s.path.begin(), s.path.end(), selected) != s.path.end();
    ImGui::Separator();
    ImGui::Text("on current route: %s", on_route ? "YES" : "no");

    // Blue (in-group) partners = ISL slots 0/1 and orange (cross-plane) = slots
    // 2/3: both are now seeded from geometry each tick, so the panel reads the
    // links themselves rather than re-deriving the blue ones.
    ImGui::Text("in-plane (blue) partners:");
    bool any_blue = false;
    for (int slot = 0; slot < 2; ++slot)
    {
      const leo::NodeId j = c.isl_neighbors[selected][slot];
      if (j == leo::INVALID_NODE)
      {
        continue;
      }
      ImGui::BulletText("cat %u   %.0f km",
                        static_cast<unsigned>(c.catalog_id[j]),
                        glm::distance(p, c.position_ecef[j]));
      any_blue = true;
    }
    if (!any_blue)
    {
      ImGui::BulletText("(none)");
    }

    ImGui::Text("cross-plane (orange) partners:");
    bool any_orange = false;
    for (int slot = 2; slot <= 3; ++slot)
    {
      const leo::NodeId j = c.isl_neighbors[selected][slot];
      if (j == leo::INVALID_NODE || j >= c.num_satellites)
      {
        continue;
      }
      ImGui::BulletText("cat %u   %.0f km",
                        static_cast<unsigned>(c.catalog_id[j]),
                        glm::distance(p, c.position_ecef[j]));
      any_orange = true;
    }
    if (!any_orange)
    {
      ImGui::BulletText("(none)");
    }
  }
  else
  {
    ImGui::Text("(no satellite)");
  }
  ImGui::End();
  if (!open)
  {
    selected = leo::INVALID_NODE;  // closing the window deselects
  }
}

// Geocentric latitude (deg) of a node from its ECEF position: asin(z / |r|).
// Used to explain why a high-latitude endpoint is blind under the 53-deg shell.
double node_latitude_deg(const leo::Constellation& c, leo::NodeId n)
{
  constexpr double kRadToDeg = 57.29577951308232;  // 180/pi
  const leo::Vec3& p = c.position_ecef[n];         // ground endpoint (ECEF km)
  const double r = glm::length(p);  // radius; 0 if failed propagation
  if (r <= 0.0)
  {
    return 0.0;  // no meaningful latitude
  }
  return std::asin(p.z / r) * kRadToDeg;
}

// World positions to ring as blind endpoints for the current Snapshot: the
// source or destination ground node named by no_path_reason. Empty when the
// route is found or the failure is kDisconnected (both endpoints have coverage,
// just no path). Shared by the live loop and the headless screenshot path.
std::vector<glm::vec3> blind_marker_points(const leo::Constellation& c,
                                           const leo::Snapshot& s,
                                           float km_scale)
{
  std::vector<glm::vec3> out;
  if (s.found)
  {
    return out;  // a routed pair has no blind endpoint to mark
  }
  leo::NodeId b = leo::INVALID_NODE;  // the blind ground node, if any
  if (s.no_path_reason == leo::Snapshot::NoPath::kSourceBlind)
  {
    b = s.src_node;
  }
  else if (s.no_path_reason == leo::Snapshot::NoPath::kDestBlind)
  {
    b = s.dst_node;
  }
  if (b != leo::INVALID_NODE && b < c.node_count())
  {
    const leo::Vec3& e = c.position_ecef[b];  // ground endpoint ECEF (km)
    out.push_back(glm::vec3(static_cast<float>(e.x * km_scale),
                            static_cast<float>(e.y * km_scale),
                            static_cast<float>(e.z * km_scale)));
  }
  return out;
}

// The "Route status" window: plain-language state for the current Snapshot.
// When found, a brief hops/ms line (the globe already shows the yellow path).
// When not, the no_path_reason in words, plus -- for a blind endpoint poleward
// of the shell's reach -- a coverage explanation. Read-only.
void draw_route_status_panel(const leo::Constellation& c,
                             const leo::Snapshot& s, double shell_incl_deg)
{
  const ImVec4 warn(0.95f, 0.6f, 0.2f, 1.0f);  // amber: an explained non-route
  ImGui::Begin("Route status");
  if (s.found)
  {
    ImGui::Text("route: %zu hops, %.1f ms", s.hops, s.cost_ms);
  }
  else
  {
    using NoPath = leo::Snapshot::NoPath;
    leo::NodeId blind = leo::INVALID_NODE;  // the blind endpoint, if any
    switch (s.no_path_reason)
    {
      case NoPath::kSourceBlind:
        ImGui::TextColored(warn,
                           "Source has no satellite in view -- no coverage "
                           "here.");
        blind = s.src_node;
        break;
      case NoPath::kDestBlind:
        ImGui::TextColored(
            warn,
            "Destination has no satellite in view -- no coverage "
            "here.");
        blind = s.dst_node;
        break;
      case NoPath::kDisconnected:
        ImGui::TextColored(warn,
                           "Endpoints are not connected through the network "
                           "right now.");
        break;
      case NoPath::kReachable:
        ImGui::TextDisabled("no route (reason unavailable)");
        break;
    }
    // Coverage hint: a blind endpoint whose latitude is poleward of the shell's
    // inclination genuinely cannot be reached by a 53-deg shell.
    if (blind != leo::INVALID_NODE && blind < c.node_count())
    {
      const double lat = node_latitude_deg(c, blind);
      if (std::fabs(lat) > shell_incl_deg)
      {
        ImGui::TextWrapped(
            "This point is above the %.0f-deg shell's coverage (lat %.0f > "
            "%.0f); the modeled shell does not reach the poles.",
            shell_incl_deg, std::fabs(lat), shell_incl_deg);
      }
    }
  }
  ImGui::End();
}

// Per-shell population diagnostic (instruction39 STEP 0). Loads the RAW,
// unfiltered catalog (build_topology would reduce it to a single shell) and
// reports, for each preset, how many satellites fall in its inclination band.
// This tells the operator which shells are populated enough to be worth
// selecting -- the polar/70-deg shells are far smaller than the 53-deg main.
// Returns false if the file cannot be loaded so the caller can exit non-zero.
bool print_shell_diagnostic(const std::string& omm, std::FILE* out)
{
  leo::Constellation raw;  // a throwaway catalog: load only, no build_topology
  leo::load_omm_json(omm, raw);
  if (raw.by_catalog.empty())
  {
    std::fprintf(out, "[shells] no satellites loaded from '%s'\n", omm.c_str());
    return false;
  }
  std::fprintf(out, "[shells] %zu satellites in '%s':\n", raw.by_catalog.size(),
               omm.c_str());
  for (const leo::ShellPreset& p : leo::shell_presets())
  {
    std::size_t n = 0;  // satellites within this shell's inclination band
    for (const auto& [cat, sat] : raw.by_catalog)
    {
      if (std::fabs(sat.inclination - p.spec.inclination_deg) <=
          p.spec.inclination_tol_deg)
      {
        ++n;
      }
    }
    std::fprintf(out, "[shells]   %-18s (incl %.1f +/- %.1f deg): %zu\n",
                 std::string(p.name).c_str(), p.spec.inclination_deg,
                 p.spec.inclination_tol_deg, n);
  }
  return true;
}

// Parse a "--planes 0,3,5" list into a plane_on mask of the given size (only
// the listed planes visible). Used by the headless screenshot path to verify
// the same filter the live panel drives.
std::vector<std::uint8_t> parse_planes(const std::string& list,
                                       std::size_t num_planes)
{
  std::vector<std::uint8_t> on(num_planes, 0);
  std::size_t i = 0;
  while (i < list.size())
  {
    std::size_t j = list.find(',', i);
    if (j == std::string::npos)
    {
      j = list.size();
    }
    const long p = std::strtol(list.substr(i, j - i).c_str(), nullptr, 10);
    if (p >= 0 && static_cast<std::size_t>(p) < num_planes)
    {
      on[p] = 1;
    }
    i = j + 1;
  }
  return on;
}

}  // namespace

int main(int argc, char** argv)
{
  // Minimal arg handling: [src] [dst] [--omm path] [--stations path]. Defaults
  // mirror the `leo` CLI so `leo_monitor` works from the repo root out of the
  // box.
  std::string src = "Chicago";
  std::string dst = "London";
  std::string omm = "data/starlink.json";
  std::string stations = "data/ground_stations.json";
  std::string screenshot;     // if set: headless, render one PNG and exit
  int warmup_frames = 1;      // sim steps before the screenshot is taken
  bool capture_ui = false;    // --ui: overlay the ImGui demo on the screenshot
  std::string planes_arg;     // --planes 0,3,5: screenshot only these planes
  bool route_planes = false;  // --route-planes: only planes on the route (as
                              // the "Only planes on route" checkbox does live)
  bool all_links = false;  // --all-links: structural view (draw every edge),
                           // mirrors unchecking "Show only routable links"
  int diag_sat = -1;       // --diag-sat N: headless print of node N's blue
                           // in-group link report (mirrors the live 'D' pick)
  int select_sat = -1;     // --select N: pre-select node N so a screenshot
                           // shows the stencil highlight (live: left-click)
  std::string start_time;  // --time ISO: start the sim clock at this UTC
                           // instant instead of now (mirrors the `leo` CLI,
                           // and lets a screenshot pin an exact instant)
  std::string fetcher =
      "fetch_starlink.exe";  // --fetcher: the external tool
                             // the "Update data" button shells out to, as the
                             // `leo update` subcommand does. Never linked in.
  bool update_data = false;  // --update-data: run the "Update data" button's
                             // fetch headlessly and exit (the button itself
                             // cannot be screenshotted), then report the result
  int shell_index = 0;       // --shell N: start on shell preset N (0 = 53-deg
                             // main default); mirrors the live Shell dropdown
                             // so each shell is screenshot-verifiable headless
  bool shell_diag = false;   // --shell-diag: print per-shell band counts of
                             // the raw catalog and exit (STEP 0 diagnostic)
  bool hide_route = false;   // --no-route: start with the yellow route line
                             // hidden (mirrors the "Show optimal route"
                             // checkbox, which a screenshot cannot click)
  bool diag_route = false;   // --diag-route: print the chosen route and compare
                             // the router's in-plane edges against the blue
                             // links the renderer draws, then exit (text only)
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i)
  {
    const std::string a = argv[i];
    if (a == "--omm" && i + 1 < argc)
    {
      omm = argv[++i];
    }
    else if (a == "--stations" && i + 1 < argc)
    {
      stations = argv[++i];
    }
    else if (a == "--screenshot" && i + 1 < argc)
    {
      screenshot = argv[++i];
    }
    else if (a == "--frames" && i + 1 < argc)
    {
      warmup_frames = std::atoi(argv[++i]);
    }
    else if (a == "--ui")
    {
      capture_ui = true;
    }
    else if (a == "--planes" && i + 1 < argc)
    {
      planes_arg = argv[++i];
    }
    else if (a == "--route-planes")
    {
      route_planes = true;
    }
    else if (a == "--all-links")
    {
      all_links = true;
    }
    else if (a == "--diag-sat" && i + 1 < argc)
    {
      diag_sat = std::atoi(argv[++i]);
    }
    else if (a == "--no-route")
    {
      hide_route = true;
    }
    else if (a == "--diag-route")
    {
      diag_route = true;
    }
    else if (a == "--select" && i + 1 < argc)
    {
      select_sat = std::atoi(argv[++i]);
    }
    else if (a == "--time" && i + 1 < argc)
    {
      start_time = argv[++i];
    }
    else if (a == "--fetcher" && i + 1 < argc)
    {
      fetcher = argv[++i];
    }
    else if (a == "--update-data")
    {
      update_data = true;
    }
    else if (a == "--shell" && i + 1 < argc)
    {
      shell_index = std::atoi(argv[++i]);
    }
    else if (a == "--shell-diag")
    {
      shell_diag = true;
    }
    else
    {
      positional.push_back(a);
    }
  }
  if (positional.size() >= 1)
  {
    src = positional[0];
  }
  if (positional.size() >= 2)
  {
    dst = positional[1];
  }

  // The instant the sim clock starts at. Parsed before the dataset loads so a
  // malformed --time fails immediately with a clear message (iso8601_to_jd
  // throws) instead of after a multi-second OMM parse.
  double start_jd = leo::now_jd_utc();  // default: wall-clock UTC, as before
  if (!start_time.empty())
  {
    try
    {
      start_jd = leo::iso8601_to_jd(start_time);
    }
    catch (const std::exception& e)
    {
      std::fprintf(stderr, "[monitor] bad --time '%s': %s\n",
                   start_time.c_str(), e.what());
      return 1;
    }
  }

  // --- Headless data refresh ----------------------------------------------
  // Drives the SAME DataUpdater the "Update data" button drives, polled exactly
  // as the frame loop polls it. The button cannot be screenshotted (--ui shows
  // the ImGui demo, not our panels), so this is how the fetch path -- success,
  // failure, and a missing tool -- is verified. Runs before the OMM load, which
  // would only read the file we are about to replace.
  if (update_data)
  {
    using State = monitor::DataUpdater::State;
    monitor::DataUpdater u;
    u.start(omm, fetcher);
    long polls = 0;  // each iteration is a frame the UI would have drawn
    while (u.busy())
    {
      u.poll();  // returns immediately whether or not the worker has finished
      ++polls;
    }
    std::printf("[monitor] polled %ld times while fetching (never blocked)\n",
                polls);
    const bool ok = u.state() == State::kDone;
    std::printf("[monitor] %s: %s\n", ok ? "ok" : "FAILED",
                u.message().c_str());
    return ok ? 0 : 1;
  }

  // STEP 0 diagnostic: report per-shell populations and exit, before any load
  // of the running sim. Independent of the chosen shell.
  if (shell_diag)
  {
    return print_shell_diagnostic(omm, stdout) ? 0 : 1;
  }

  // Resolve the starting shell preset. Out-of-range --shell falls back to the
  // 53-deg default (index 0) rather than indexing past the end.
  const std::vector<leo::ShellPreset>& shells = leo::shell_presets();
  if (shell_index < 0 || shell_index >= static_cast<int>(shells.size()))
  {
    shell_index = 0;
  }

  // Tier 1 + Tier 2: load the dataset and place the endpoints. If this fails
  // there is nothing to render -- exit cleanly (and the simulator/CLI are of
  // course unaffected; this is a separate process).
  leo::Simulator sim;
  if (!sim.load(omm, stations, shells[shell_index].spec))
  {
    std::fprintf(stderr,
                 "[monitor] failed to load '%s' / '%s' (run from the repo root "
                 "or pass --omm/--stations)\n",
                 omm.c_str(), stations.c_str());
    return 1;
  }
  if (!sim.set_endpoints(src, dst))
  {
    std::fprintf(stderr, "[monitor] unknown station name in '%s' / '%s'\n",
                 src.c_str(), dst.c_str());
    return 1;
  }

  // --- Headless route diagnostic -------------------------------------------
  // Compares the router's usable graph against the blue in-group links the
  // renderer draws, for the route between the two endpoints. Text only: it
  // needs no Vulkan, so it runs anywhere and exits before any renderer is
  // created. Warms up exactly as the screenshot path does, so the constellation
  // is propagated and the links are seeded before anything is compared.
  if (diag_route)
  {
    double t_jd = start_jd; /* the instant being diagnosed */
    leo::Snapshot snap = sim.step(t_jd);
    for (int i = 1; i < warmup_frames; ++i)
    {
      t_jd += 30.0 / 86400.0; /* +30 sim seconds per step    */
      snap = sim.step(t_jd);
    }
    std::printf("[monitor] %s -> %s | shell %s | %s\n", src.c_str(),
                dst.c_str(), std::string(shells[shell_index].name).c_str(),
                snap.utc_iso.c_str());
    monitor::print_route_diag(sim.constellation(), snap, sim.link_spec(),
                              stdout);
    return 0;
  }

  // --- Headless screenshot mode -------------------------------------------
  // Render a single frame to a PNG with no window/swapchain. This works in a
  // non-interactive session (no visible desktop needed) and never touches the
  // screen. Shares all the scene-building and the renderer with the live path.
  if (!screenshot.empty())
  {
    const int w = 1280, h = 800;
    monitor::VulkanRenderer renderer(w, h);
    if (!renderer.ok())
    {
      std::fprintf(stderr, "[monitor] headless renderer init failed\n");
      return 1;
    }
    renderer.set_capture_ui(capture_ui);  // --ui overlays the ImGui demo

    double t_jd = start_jd;
    leo::Snapshot snap = sim.step(t_jd);
    // A few warm-up steps so the geometry is past t0 and links are seeded.
    for (int i = 1; i < warmup_frames; ++i)
    {
      t_jd += 30.0 / 86400.0;  // +30 sim seconds per warm-up step
      snap = sim.step(t_jd);
    }
    const leo::Constellation& c = sim.constellation();

    // Headless blue-link report for one satellite (mirrors the live 'D' pick).
    if (diag_sat >= 0)
    {
      print_sat_link_report(c, sim.link_spec(),
                            static_cast<leo::NodeId>(diag_sat), stdout);
    }

    std::vector<monitor::MeshVertex> sv;
    std::vector<std::uint32_t> si;
    monitor::build_sphere(6378.137f * kKmScale, 48, 96, sv, si);
    renderer.set_earth_mesh(sv, si);
    const float atmo_km = snap.atmosphere_radius_km > 0.0
                              ? static_cast<float>(snap.atmosphere_radius_km)
                              : 6458.0f;
    monitor::build_sphere(atmo_km * kKmScale, 48, 96, sv, si);
    renderer.set_atmosphere_mesh(sv, si);

    // Plane filter for the screenshot: --route-planes isolates the planes on
    // the optimal path (mirrors the live checkbox), else --planes picks
    // specific planes, else empty = all.
    std::vector<std::uint8_t> plane_on;
    if (route_planes)
    {
      plane_on = monitor::plane_mask_on_route(c, snap, snap.num_planes);
    }
    else if (!planes_arg.empty())
    {
      plane_on = parse_planes(planes_arg, snap.num_planes);
    }

    std::vector<glm::vec3> pts;
    std::vector<monitor::LineVertex> edge_lines;  // cross-plane + uplink
    std::vector<monitor::LineVertex> blue_lines;  // geometric in-group (blue)
    std::vector<monitor::RouteVertex> path_lines;
    monitor::build_satellite_points(c, snap, kKmScale, pts, plane_on);
    renderer.set_points(pts);
    monitor::build_edge_lines(c, snap, kKmScale, edge_lines, sim.link_spec(),
                              !all_links, plane_on);
    // Blue in-group links are geometric now: append them to the same buffer.
    monitor::build_ingroup_lines(c, kKmScale, blue_lines, plane_on);
    edge_lines.insert(edge_lines.end(), blue_lines.begin(), blue_lines.end());
    renderer.set_edges(edge_lines);
    // Same gate as the live path: --no-route mirrors the "Show optimal route"
    // checkbox, so the hidden state is verifiable in a screenshot.
    if (!hide_route)
    {
      monitor::build_path_lines(c, snap, kKmScale, path_lines);
    }
    renderer.set_path(path_lines);

    // --select N highlights one satellite so the stencil ring is verifiable
    // headless (the live path selects by left-click instead).
    std::vector<glm::vec3> sel;
    if (select_sat >= 0 &&
        static_cast<std::size_t>(select_sat) < c.num_satellites)
    {
      const leo::Vec3& e = c.position_ecef[select_sat];
      sel.push_back(glm::vec3(static_cast<float>(e.x * kKmScale),
                              static_cast<float>(e.y * kKmScale),
                              static_cast<float>(e.z * kKmScale)));
    }
    renderer.set_selection(sel);
    // Amber ring on a blind endpoint (e.g. a high-latitude station past the
    // shell's reach), so a coverage gap is a located, visible fact.
    renderer.set_markers(blind_marker_points(c, snap, kKmScale));

    monitor::OrbitCamera cam;
    const float aspect = static_cast<float>(w) / static_cast<float>(h);
    const bool wrote =
        renderer.capture(cam.view(), cam.proj(aspect), screenshot);
    renderer.wait_idle();
    if (!wrote)
    {
      std::fprintf(stderr, "[monitor] failed to write %s\n",
                   screenshot.c_str());
      return 1;
    }
    std::printf(
        "[monitor] wrote %s (%dx%d) | shell %s | %s -> %s, %zu sats, "
        "%zu planes\n",
        screenshot.c_str(), w, h, std::string(shells[shell_index].name).c_str(),
        src.c_str(), dst.c_str(), snap.usable_sats, snap.num_planes);
    return 0;
  }

  if (!glfwInit())
  {
    std::fprintf(stderr, "[monitor] glfwInit failed\n");
    return 1;
  }
  if (!glfwVulkanSupported())
  {
    std::fprintf(stderr, "[monitor] Vulkan not supported on this system\n");
    glfwTerminate();
    return 1;
  }
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);  // Vulkan, not GL
  GLFWwindow* window =
      glfwCreateWindow(1280, 800, "leo monitor", nullptr, nullptr);
  if (!window)
  {
    std::fprintf(stderr, "[monitor] glfwCreateWindow failed\n");
    glfwTerminate();
    return 1;
  }

  InputState input;
  glfwSetWindowUserPointer(window, &input);
  glfwSetCursorPosCallback(window, cursor_cb);
  glfwSetMouseButtonCallback(window, button_cb);
  glfwSetScrollCallback(window, scroll_cb);

  monitor::VulkanRenderer renderer(window);
  if (!renderer.ok())
  {
    std::fprintf(stderr, "[monitor] renderer init failed; exiting\n");
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  // First step gives us the atmosphere radius (= LinkSpec block radius) and the
  // initial geometry. Earth radius is the WGS84 equatorial value. `clock` owns
  // simulated time from here on -- nothing else writes t_jd.
  SimClock clock;
  clock.t_jd = start_jd;
  leo::Snapshot snap = sim.step(clock.t_jd);
  const leo::Constellation& c = sim.constellation();

  // Static shells (built once). Earth at 6378 km, atmosphere from the snapshot.
  std::vector<monitor::MeshVertex> sphere_v;
  std::vector<std::uint32_t> sphere_i;
  monitor::build_sphere(6378.137f * kKmScale, 48, 96, sphere_v, sphere_i);
  renderer.set_earth_mesh(sphere_v, sphere_i);
  const float atmo_km = snap.atmosphere_radius_km > 0.0
                            ? static_cast<float>(snap.atmosphere_radius_km)
                            : 6458.0f;
  monitor::build_sphere(atmo_km * kKmScale, 48, 96, sphere_v, sphere_i);
  renderer.set_atmosphere_mesh(sphere_v, sphere_i);

  // Per-plane visibility, driven by the "Planes" panel. All planes on at start.
  // `route_only` is the "Only planes on route" mode: while set, the effective
  // filter is derived from the route each frame and plane_on is left untouched.
  std::vector<std::uint8_t> plane_on(snap.num_planes, 1);
  bool route_only = false;

  // Edge view: true = draw only routable cross-plane links, false = the full
  // designed lattice. Driven by the "Links" panel below. `show_ingroup` toggles
  // the blue in-plane links, which need no feasibility view of their own -- the
  // core seeds slots 0/1 feasible, so what is drawn is what the router uses.
  bool show_only_routable = true;
  bool show_ingroup = true;
  // The yellow route line. Visibility only -- the route itself is always
  // computed, so hiding the line costs nothing and hides nothing else.
  bool show_route = !hide_route;  // --no-route starts with it hidden

  // Endpoint dropdown state. Seed the two selections from the startup endpoints
  // so the initial view matches the CLI args exactly. A named station selects
  // its catalog row; a lat,lon startup arg (no name match) falls back to the
  // first two rows but does NOT re-route -- set_endpoints is only called when
  // the user picks from a dropdown, so the running route keeps its startup
  // endpoints until then. `catalog` aliases the Simulator's station list
  // (stable across set_endpoints, which never touches stations_), the same list
  // the CLI reads.
  const std::vector<leo::GroundStation>& catalog = sim.stations();
  auto index_of = [&](const std::string& name, int fallback)
  {
    for (int i = 0; i < static_cast<int>(catalog.size()); ++i)
    {
      if (catalog[i].name == name)
      {
        return i;
      }
    }
    return fallback;
  };
  int sel_src = index_of(src, 0);
  int sel_dst = index_of(dst, catalog.size() > 1 ? 1 : 0);

  // The route highlight is never plane-filtered -- it is the answer, always
  // shown. Its geometry is rebuilt EVERY frame from live positions (below), not
  // cached, so the yellow line always passes through the moving satellite dots.
  std::vector<glm::vec3> pts;
  std::vector<monitor::LineVertex> edge_lines;  // cross-plane + uplink
  std::vector<monitor::LineVertex> blue_lines;  // geometric in-group (blue)
  std::vector<monitor::RouteVertex> path_lines;

  // `mask` is the effective plane filter for this frame (manual plane_on, or
  // the route-derived mask when route_only is set) -- passed in rather than
  // captured so the caller decides which without mutating plane_on.
  auto upload_dynamic =
      [&](const leo::Snapshot& s, const std::vector<std::uint8_t>& mask)
  {
    monitor::build_satellite_points(c, s, kKmScale, pts, mask);
    renderer.set_points(pts);
    monitor::build_edge_lines(c, s, kKmScale, edge_lines, sim.link_spec(),
                              show_only_routable, mask);
    // Geometric blue in-group links, appended to the same edge buffer so the
    // renderer draws one line list. Rebuilt each frame like the other edges;
    // skipped entirely when the "Show in-group (blue) links" toggle is off.
    if (show_ingroup)
    {
      monitor::build_ingroup_lines(c, kKmScale, blue_lines, mask);
      edge_lines.insert(edge_lines.end(), blue_lines.begin(), blue_lines.end());
    }
    renderer.set_edges(edge_lines);
    // Rebuild the route line from the CURRENT position_ecef of each path NodeId
    // every frame -- the SAME array and frame the satellite dots use. Caching
    // only the NodeId list (as before) drifted the line off the moving dots,
    // because the satellites advance each tick while the route stays the same.
    // The geometry is a handful of segments, so an unconditional rebuild is
    // free.
    //
    // "Show optimal route" gates only THIS -- the geometry upload. The route is
    // still routed every tick, so the Route status panel and the satellite
    // panel's "on current route" field keep working while the line is hidden;
    // an empty vertex list simply means the route pass draws nothing.
    if (show_route)
    {
      monitor::build_path_lines(c, s, kKmScale, path_lines);
    }
    else
    {
      path_lines.clear();
    }
    renderer.set_path(path_lines);
  };
  upload_dynamic(snap, plane_on);

  // Pick the satellite nearest the cursor: project each to screen, keep the
  // closest within a small pixel radius, but REJECT any dot occluded by the
  // Earth (a ray from the eye that hits the globe before reaching the
  // satellite). So a click selects only a visible dot, never one on the far
  // hemisphere. Returns INVALID_NODE over empty space. `eye` is the camera
  // position in world units.
  const float earth_r = kEarthRadiusWorld;  // the same globe the grab casts at
  const auto pick_sat = [&](const glm::mat4& mvp, const glm::vec3& eye,
                            double mx, double my, int ww, int wh) -> leo::NodeId
  {
    leo::NodeId best = leo::INVALID_NODE; /* nearest satellite so far      */
    double best_px2 = 20.0 * 20.0;        /* accept only within ~20 px     */
    for (leo::NodeId i = 0; i < c.num_satellites; ++i)
    {
      const leo::Vec3& e = c.position_ecef[i];  // ECEF km -> world (km_scale)
      const glm::vec3 wp(static_cast<float>(e.x * kKmScale),
                         static_cast<float>(e.y * kKmScale),
                         static_cast<float>(e.z * kKmScale));
      const glm::vec4 clip = mvp * glm::vec4(wp, 1.0f);
      if (clip.w <= 0.0f)
      {
        continue;  // behind the camera -> not clickable
      }
      const glm::vec3 ndc = glm::vec3(clip) / clip.w;  // perspective divide
      // NDC -> window pixels. The camera projection already negates Y for
      // Vulkan (NDC +Y points DOWN), so NDC and the GLFW cursor share a
      // top-left origin -- map y straight through, with NO extra flip.
      const double sx = (ndc.x * 0.5 + 0.5) * ww;
      const double sy = (ndc.y * 0.5 + 0.5) * wh;
      const double dx = sx - mx, dy = sy - my;  // pixel offset from cursor
      const double d2 = dx * dx + dy * dy;
      if (d2 >= best_px2)
      {
        continue;  // not the closest to the cursor
      }
      // Earth-occlusion test: ray eye->sat vs the Earth sphere at the origin. A
      // real hit (t in (0, dist)) means the globe is in front -> not
      // selectable.
      const glm::vec3 dir = wp - eye;
      const float dist = glm::length(dir);
      const glm::vec3 dn = dir / dist;
      const float b = glm::dot(eye, dn);
      const float cc = glm::dot(eye, eye) - earth_r * earth_r;
      const float disc = b * b - cc;
      if (disc > 0.0f)
      {
        const float t = -b - std::sqrt(disc);  // near intersection
        if (t > 0.0f && t < dist - 1e-3f)
        {
          continue;  // occluded by the Earth
        }
      }
      best_px2 = d2;
      best = i;
    }
    return best;
  };
  bool d_prev = false;  // edge-detect the 'D' key so one press = one report
  bool r_prev = false;  // edge-detect 'R' so a held key does not re-reset

  // Selection state. `selected` is the clicked satellite (INVALID_NODE = none);
  // it drives the stencil highlight and the info window. --select pre-selects
  // for parity. Pause now lives on `clock` with the rest of the time controls.
  leo::NodeId selected = (select_sat >= 0 && static_cast<std::size_t>(
                                                 select_sat) < c.num_satellites)
                             ? static_cast<leo::NodeId>(select_sat)
                             : leo::INVALID_NODE;
  std::vector<glm::vec3> sel_pt;  // reused each frame: 0 or 1 world position

  // Frame-rate meter, the UTC entry buffer, and the background data fetcher --
  // all owned by the loop and driven by the Simulation panel.
  FpsMeter fps;
  TimeUi time_ui;
  monitor::DataUpdater updater;

  // The active shell preset index, driven by the "Shell" dropdown.
  int sel_shell = shell_index;

  // Rebuild the whole scene for a different shell. This is a TIER-1 switch:
  // sim.load re-filters, re-clusters and re-inits the propagator, which changes
  // the satellite SET and RENUMBERS every NodeId. So every piece of state keyed
  // to the old numbering MUST be invalidated first -- a stale NodeId indexing
  // the new (smaller) constellation is a crash, not a glitch. Order matters:
  // load (new numbering) -> re-place endpoints BY NAME -> drop the selection ->
  // step to populate positions -> resize the plane mask -> re-upload buffers.
  // Deliberately a one-shot path, never a per-frame poke.
  const auto rebuild_for_shell = [&](int idx) -> bool
  {
    if (!sim.load(omm, stations, shells[idx].spec))
    {
      return false;  // empty shell: keep old scene, caller reverts the combo
    }
    // Endpoints are ground nodes wiped by the reload; re-place them by the
    // names still selected in the dropdowns (indices into the unchanged station
    // list).
    sim.set_endpoints(catalog[sel_src].name, catalog[sel_dst].name);
    selected = leo::INVALID_NODE;         // old NodeId is meaningless now
    snap = sim.step(clock.t_jd);          // populate new shell at current time
    plane_on.assign(snap.num_planes, 1);  // group count changed with the shell
    upload_dynamic(snap, plane_on);       // rebuild count-sized buffers
    std::printf("[monitor] shell -> %s | %zu sats, %zu groups\n",
                std::string(shells[idx].name).c_str(), snap.usable_sats,
                snap.num_planes);
    return true;
  };

  std::printf("[monitor] shell: %s\n",
              std::string(shells[shell_index].name).c_str());
  std::printf(
      "[monitor] %s -> %s | %zu sats, %zu planes | drag the globe to "
      "turn it, scroll to zoom, left-click to select, D = link report, "
      "R = reset view\n",
      src.c_str(), dst.c_str(), snap.usable_sats, snap.num_planes);

  double last_time = glfwGetTime();
  while (!glfwWindowShouldClose(window))
  {
    glfwPollEvents();
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
      glfwSetWindowShouldClose(window, GLFW_TRUE);
    }

    // Shared pick setup: window size, camera MVP, and eye for both the click
    // selection and the 'D' link report, so both use the same occlusion-correct
    // pick.
    int ww = 0, wh = 0;
    glfwGetWindowSize(window, &ww, &wh);
    const float pick_aspect = wh > 0 ? static_cast<float>(ww) / wh : 1.0f;
    const glm::mat4 pick_mvp =
        input.camera.proj(pick_aspect) * input.camera.view();
    const glm::vec3 eye = input.camera.eye();

    // Left-click selects the satellite under the cursor; clicking empty space
    // clears the selection. The callback set click_pending on a non-drag
    // release.
    if (input.click_pending)
    {
      input.click_pending = false;
      selected = pick_sat(pick_mvp, eye, input.last_x, input.last_y, ww, wh);
    }

    // 'D' (on the press edge) dumps the blue-link diagnostic for the satellite
    // under the cursor. Ignored while ImGui owns the keyboard (e.g. typing).
    const bool d_now = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;
    if (d_now && !d_prev && !ImGui::GetIO().WantCaptureKeyboard)
    {
      const leo::NodeId hit =
          pick_sat(pick_mvp, eye, input.last_x, input.last_y, ww, wh);
      if (hit != leo::INVALID_NODE)
      {
        print_sat_link_report(c, sim.link_spec(), hit, stdout);
      }
      else
      {
        std::printf("[sat] no satellite under cursor\n");
      }
    }
    d_prev = d_now;

    // 'R' (on the press edge) restores the north-up startup view. Exact grab
    // manipulation can only be exact if it is free to ROLL -- a chain of drags
    // may leave north tilted, which is correct but disorienting -- so this is
    // the way back. Distance is kept: a reset re-aims, it does not re-zoom.
    const bool r_now = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS;
    if (r_now && !r_prev && !ImGui::GetIO().WantCaptureKeyboard)
    {
      input.camera.reset_view();
    }
    r_prev = r_now;

    // Advance simulated time, then re-run Tier 3 from the new instant. Keep
    // last_time current every frame so that unpausing resumes from the current
    // t_jd (one frame's dt), never jumping by the whole paused duration.
    // clock.advance() applies the pause gate and the speed multiplier; a jump
    // requested by last frame's panel is already sitting in clock.t_jd.
    const double now = glfwGetTime();
    const double dt_real = now - last_time;
    last_time = now;
    fps.sample(dt_real);  // real frame rate, independent of pause and speed
    clock.advance(dt_real);
    snap = sim.step(clock.t_jd);

    // Planes could change count only if the dataset reloaded (it doesn't here),
    // but keep the mask sized defensively.
    if (plane_on.size() != snap.num_planes)
    {
      plane_on.assign(snap.num_planes, 1);
    }

    // Build the ImGui frame FIRST: the panel's checkboxes mutate plane_on, and
    // the geometry rebuilt just below must reflect this frame's toggles.
    // Render() must run before renderer.draw() records the draw data.
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    // Collect a finished fetch, if any, before the panel reads its state. Never
    // blocks: an in-flight download just leaves the button disabled this frame.
    updater.poll();
    // May jump clock.t_jd; the jump lands on the NEXT step() (Tier 3 only), so
    // this frame still shows the instant that was just propagated.
    draw_sim_panel(clock, time_ui, fps.fps, snap, updater, omm, fetcher);
    // Route status: found -> hops/ms; else the reason + coverage hint. The
    // shell inclination is the one the sim was built with (default ShellSpec
    // here).
    draw_route_status_panel(c, snap, leo::ShellSpec{}.inclination_deg);
    draw_planes_panel(plane_on, route_only, snap.found);
    draw_links_panel(show_only_routable, show_ingroup, show_route);
    // Info window for the selected satellite; its [x] deselects (sets INVALID).
    draw_selected_info_panel(c, snap, sim.link_spec(), selected);

    // Endpoint pickers. On a change, re-place the two ground nodes with Tier 2
    // ONLY (set_endpoints) -- never load()/build_topology(), which would
    // renumber every NodeId under the renderer. The next step() emits a
    // Snapshot with the new route; the highlight rebuilds every frame anyway,
    // so nothing else is needed. An unknown name (can't happen -- names come
    // from the catalog) leaves the previous pair in place rather than
    // half-applying.
    const int prev_src = sel_src, prev_dst = sel_dst;
    if (draw_endpoints_panel(catalog, sel_src, sel_dst))
    {
      if (!sim.set_endpoints(catalog[sel_src].name, catalog[sel_dst].name))
      {
        sel_src = prev_src;
        sel_dst = prev_dst;
      }
    }

    // Shell selector. On a change, run the deliberate Tier-1 rebuild (reload +
    // re-cluster + renumber + scene reset) rather than poking a parameter. This
    // MUST happen before ImGui::Render(): the rebuild resets `selected`, `snap`
    // and `plane_on`, and the post-render scene upload below reads all three.
    // The earlier panels drew from the old shell this frame; next frame they
    // reflect the new one. A failed rebuild (unreadable file) reverts the
    // combo.
    const int new_shell =
        draw_shell_panel(shells, sel_shell, snap.usable_sats, snap.num_planes);
    if (new_shell >= 0)
    {
      if (rebuild_for_shell(new_shell))
      {
        sel_shell = new_shell;
      }
    }
    ImGui::Render();

    // Effective filter: the route-derived mask while "Only planes on route" is
    // set (recomputed each frame so it tracks the moving constellation), else
    // the manual plane_on. plane_on stays intact so unchecking restores it.
    const std::vector<std::uint8_t> route_mask =
        route_only ? monitor::plane_mask_on_route(c, snap, snap.num_planes)
                   : std::vector<std::uint8_t>{};
    upload_dynamic(snap, route_only ? route_mask : plane_on);

    // Highlight the selected satellite at its LIVE position (empty clears it).
    // Guard a stale index if the selection somehow outran num_satellites.
    sel_pt.clear();
    if (selected != leo::INVALID_NODE && selected < c.num_satellites)
    {
      const leo::Vec3& e = c.position_ecef[selected];
      sel_pt.push_back(glm::vec3(static_cast<float>(e.x * kKmScale),
                                 static_cast<float>(e.y * kKmScale),
                                 static_cast<float>(e.z * kKmScale)));
    }
    renderer.set_selection(sel_pt);
    // Amber ring(s) on any blind endpoint, tracking the live ground position.
    renderer.set_markers(blind_marker_points(c, snap, kKmScale));

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    const float aspect =
        fbh > 0 ? static_cast<float>(fbw) / static_cast<float>(fbh) : 1.0f;
    renderer.draw(input.camera.view(), input.camera.proj(aspect));
  }

  renderer.wait_idle();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
