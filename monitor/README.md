# leo_monitor — optional Vulkan 1.4 renderer

A separate, **optional** frontend that visualizes the LEO simulator in 3D. It is
disabled by default and completely isolated from the simulator library and the
`leo` CLI: a normal build produces exactly the same simulator + CLI as before,
with no Vulkan or GLFW dependency. The renderer only ever *reads* the simulator
(via `const leo::Constellation&` / `const leo::Snapshot&`), so enabling or
breaking it can never affect the core.

## Enabling it

```sh
cmake -S . -B build -DLEO_BUILD_MONITOR=ON
cmake --build build --target leo_monitor
```

With the option **OFF** (the default) none of the Vulkan/GLFW machinery below is
even configured.

## Dependencies

- **Vulkan SDK ≥ 1.3** (developed against 1.4). Located via
  `find_package(Vulkan)`, which reads the `VULKAN_SDK` environment variable set
  by the official SDK installer on both Windows and Linux. No SDK path is ever
  hardcoded. The SDK also provides the `glslc` / `glslangValidator` used to
  compile the shaders to SPIR-V at build time.
- **GLFW 3.4** — fetched and built from source automatically via
  `FetchContent`, so no system package is required on either OS.

If `find_package(Vulkan)` fails, install the LunarG Vulkan SDK and make sure
`VULKAN_SDK` is exported (the installer does this; on Linux, source the SDK's
`setup-env.sh`).

## Running

```sh
# from the repo root (so the default data/ paths resolve)
./build/<config>/leo_monitor                 # defaults to Chicago -> London
./build/<config>/leo_monitor Seattle Tokyo
./build/<config>/leo_monitor --omm data/starlink.json --stations data/ground_stations.json
```

Controls: **left-drag the globe** to turn it (the surface point you grab stays
under the cursor), **scroll** to zoom, **left-click** a satellite to select it,
**D** for its link report, **R** to reset the view, **Esc** to quit. The view
starts north-up over the Atlantic; because an exact grab is free to roll, a long
chain of drags can leave north tilted — **R** puts it back.

### Headless screenshot

To render a single frame to a PNG with **no window** (works over SSH / in CI /
in any session without a visible desktop):

```sh
./build/<config>/leo_monitor Chicago London --screenshot frame.png --frames 30
```

`--frames N` advances the simulation N steps before capturing (so links are
seeded and the geometry has moved off t0). The PNG is written by a small
self-contained encoder, so this path needs Vulkan but no image library.

The scene shows the Earth and a translucent atmosphere shell, the satellite
cloud (one point per valid satellite), the ISL/uplink edges colored by type
(blue = in-plane, orange = cross-plane, green = uplink), and the current route
highlighted in yellow. Simulated time advances ~60× real time.

## Frame discipline (the one real trap)

The renderer uses the **Earth-locked** view: satellites are drawn at their
`position_ecef` (Earth-fixed) against a **fixed** Earth, so we do *not* also
rotate the Earth by `gmst_rad` — doing both would double-count Earth rotation.
Satellite orientation is never derived from velocity (velocity is in the ECI
frame; mixing it with an ECEF position is a frame error). An inertial view
(`position_eci` + Earth spun by `gmst_rad`) would be a valid alternative mode,
but this build commits to the single Earth-locked mode.

## Layout

```
monitor/
  CMakeLists.txt        # all Vulkan/GLFW config; only runs under the option
  src/
    main.cpp            # window, camera, sim driver, frame loop
    vk_renderer.{hpp,cpp}  # Vulkan 1.4: dynamic rendering + synchronization2
    scene.{hpp,cpp}     # const-ref free functions: Snapshot/Constellation -> vertices
    camera.hpp          # header-only orbit camera
  shaders/              # GLSL sources, compiled to SPIR-V at build time
```
