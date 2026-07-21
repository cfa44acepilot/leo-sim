# leo — LEO Satellite Constellation Routing Simulator

A headless C++20 simulator for a low-Earth-orbit satellite network (modeled on
Starlink). It loads real published orbital elements (OMM/GP JSON from CelesTrak),
propagates every satellite to a requested instant with SGP4, reconstructs the
orbital-plane lattice and the inter-satellite laser-link mesh, attaches
ground-station uplinks, and computes the lowest-latency route between two ground
points — reporting that latency against an ideal-fiber baseline. It is the
simulation core for a planned Vulkan renderer; the geometry and routing are built
and validated first so the renderer consumes a pipeline that is already proven
correct.

---

## Quick start

Requires CMake ≥ 3.20 and a C++20 compiler (developed with MSVC). The first
configure uses `FetchContent` to pull the dependencies — `unordered_dense`,
`glm`, `nlohmann/json`, and GoogleTest — so it needs network access and takes
~50 s.

```sh
# Configure (downloads dependencies on first run)
cmake -S . -B build

# Build everything: the `leo` CLI, the `constellation` library, and the tests
cmake --build build

# ...or just the CLI front end
cmake --build build --target leo
```

With an MSVC multi-config generator the binary lands at `build/Debug/leo.exe`.
The headline query routes between two named ground stations:

```
$ ./build/Debug/leo.exe Chicago London
Chicago -> London  at 2026-06-29T16:20:00 UTC
route: Chicago -> SAT-44932 (p31,s12) =in-plane=> SAT-45044 (p31,s13) =cross=> SAT-46157 (p32,s14) =in-plane=> SAT-46203 (p32,s15) =cross=> SAT-45210 (p33,s11) -> London
latency: 27.9 ms over 6 hops
27.9 ms vs 31.1 ms fiber (1.1x faster than fiber)
10634 satellites usable, 72 planes
```

The route line tags every hop so its structure is visible: ` -> ` is a
ground↔satellite uplink/downlink, ` =in-plane=> ` is a fore/aft link within one
orbital plane, ` =cross=> ` is a cross-plane link. The last two lines are the
"trust" report — how many satellites actually propagated and how many planes were
resolved. When the snapshot's newest element epoch is far from the requested
time, a staleness warning is appended and the answer is flagged as extrapolated;
run `leo update` to pull a fresh snapshot first.

Endpoints may be station names (from `data/ground_stations.json`, which ships
with eight cities including Chicago and London) or raw `lat,lon` pairs:

```sh
leo Chicago London
leo "41.88,-87.63" "51.51,-0.13"
leo Chicago London --time 2026-06-29T18:00:00 --hop-cost 0.0005 --diag
```

Exit codes: `0` a route was found, `1` no route at that instant (a normal outcome
over a sparse one-second snapshot, printed with the reason), `2` a usage or I/O
error.

---

## How it works

The pipeline lives in `src/` as a chain of small, independently tested modules.
A query runs them in order:

1. **OMM load** (`omm_loader.cpp`) — parses the CelesTrak GP/OMM JSON with
   `nlohmann/json` into cold `Satellite` records. File I/O only; no networking in
   the library.
2. **Topology / plane clustering** (`topology.cpp`) — filters one shell by
   inclination, bins satellites into orbital planes by RAAN, sorts them by
   (plane, slot), and wires the in-plane fore/aft ISL terminals. Reorders the
   dense arrays into plane-contiguous order, so it runs exactly once.
3. **SGP4 propagation** (`propagator.cpp`) — initializes the vendored Vallado
   SGP4 model directly from the mean elements (no TLE string round-trip) and
   propagates to the requested time, writing TEME (≈ECI) positions/velocities and
   the Earth-fixed ECEF positions derived by rotating about the spin axis by GMST.
4. **Cross-plane ISL seeding** (`isl_links.cpp`) — for each satellite, picks the
   nearest satellite in each adjacent plane that is in laser range, in line of
   sight (`earth_clear`), and travelling the same direction.
5. **Ground uplinks** (`ground.cpp`) — connects each ground station to every
   satellite above its minimum elevation.
6. **Routing** (`router.cpp`) — builds a symmetric CSR graph from the ISL slots
   and uplinks, then runs **A\*** with edge weights of light-time
   (`distance / c`) plus an optional per-hop switching cost. Dijkstra is kept as
   the zero-heuristic reference. The router applies the physical-feasibility gate
   at build time, so an edge over range or occluded by Earth is never traversed.
7. **Snapshot assembly** (`simulator.cpp`) — packages the answer, a typed/deduped
   edge list for rendering, and the trust data (epoch health, usable-satellite
   count) into one `Snapshot`. The CLI (`main.cpp` / `cli.cpp`) only formats it.

**Frame discipline.** Inertial TEME/ECI is the integrator state and the frame for
the velocity-direction link filter (it is free of Earth rotation); Earth-fixed
ECEF, obtained as `R_z(GMST) · ECI`, is used for everything tied to the ground —
distances, line-of-sight, and ground-station geometry. **Units** are kilometers
and km/s throughout (SGP4's native output), including routing weights.

---

## Validation

Every layer is pinned against an **independent reference**, not against values the
code generated itself. The suite is **56 tests across 11 GoogleTest binaries**,
all passing.

- **SGP4 propagation vs Vallado's published SGP4-VER vectors.** Satellite 00005 is
  initialized from the `SGP4-VER.TLE` mean elements and propagated; its TEME
  position is required to match the official `00005.e` ephemeris to within **1 km**
  at both *t* = 0 and *t* = 360 min — the latter exercising the time-dependent
  terms that *t* = 0 cannot. (`propagator_test.cpp`, `Sgp4ReferenceSat5`)
- **GMST vs the J2000 reference.** The sidereal-time formula is checked against the
  known value at 2000-01-01T12:00 UTC, ≈ 4.894961 rad (280.4606°), independent of
  SGP4. (`propagator_test.cpp`, `GmstAtJ2000`)
- **`geodetic_to_ecef` vs hand-computed axis points.** The equator maps to the
  semi-major axis (x = 6378.137 km) and the pole to the semi-minor axis
  (z = 6356.752 km), validating the WGS84 ellipsoid math.
  (`ground_test.cpp`, `GeodeticToEcefAxisPoints`)
- **A\* proven equal to Dijkstra.** Over a 3×3 graph and several source/destination
  pairs (with and without a per-hop cost), A\* returns the identical optimal cost
  and never expands more nodes than Dijkstra — i.e. the straight-line light-time
  heuristic is admissible and consistent. (`router_test.cpp`, `AstarEqualsDijkstra`)
- **Link feasibility and ISL gating.** `earth_clear` is tested directly at the
  grazing boundary (a sight line just above the block radius clears, just below is
  blocked); seeding rejects candidates that are out of range, occluded by Earth, or
  counter-moving; and the router's gate drops a structurally-wired in-plane link
  that is over the operational range while keeping one within range.
  (`isl_links_test.cpp`, `router_test.cpp`)

Run them all with CTest (or run the individual `*_test` executables in
`build/Debug/` directly):

```sh
ctest --test-dir build -C Debug
```

---

## Design decisions worth knowing

**A physically-grounded link budget.** `max_range_km` and `block_radius_km` are
not free parameters — they are coupled by geometry. Two satellites at orbital
radius *R* whose sight line just grazes a shell of radius *r* can be at most
`2·√(R² − r²)` apart before the line dips into that shell. For shell 1
(*R* ≈ 6921 km, *r* = `block_radius_km` = 6458 km) that grazing ceiling is
≈ 4980 km — the hard physical limit. The default `max_range_km` is set to a
shorter **operational** value of **3000 km** (published Starlink ISL figures
suggest ~1500–3000 km, bounded by laser pointing/power/acquisition), which sits
safely under the geometric ceiling. Changing one without the other is unphysical.

**The constellation is a 3D geometric object, not a graph with fixed topology.**
The router minimizes light-time over the satellites' actual propagated positions,
so an optimal path is free to weave through *adjacent planes* (cross-plane hops)
rather than following the in-plane lattice. The plane/slot labels in the route
output make this visible — you can watch the path trade an in-plane hop for a
shorter cross-plane chord when the geometry favors it.

---

## Known limitations / future work

- **Fixed-plane-count clustering.** Topology assumes a known Walker plane count
  (72 for the modeled Starlink shell). This is correct for modeling Starlink
  specifically, but a general constellation detector would need
  precession-corrected RAAN clustering — RAAN drifts ~5°/day from the J2 bulge,
  so a snapshot spanning several days smears the planes together.
- **`leo update` is Windows-only.** It shells out to a separate `fetch_starlink`
  tool to download a fresh snapshot; networking is deliberately kept out of the
  simulator library. The simulator itself only reads local `data/*.json` and is
  portable.
- **The Vulkan renderer is not yet built.** The `Snapshot` already carries the
  edge list, GMST, and validity flags it will need; the renderer is the next
  milestone.
