// constellation.cpp -- container mutators for the hot/cold layout.

#include "constellation.hpp"

namespace leo {

NodeId Constellation::add_satellite(const Satellite& sat) {
  // Duplicate guard: re-adding a catalog id must not push a second row, which
  // would leave the map smaller than the arrays and break the bridge. Hand back
  // the existing row instead so callers stay valid.
  if (const auto it = by_catalog.find(sat.catalog_id); it != by_catalog.end()) {
    return it->second.index;
  }

  // The new row is the current array length (nodes are appended, never reused).
  const NodeId index = static_cast<NodeId>(node_count());

  // Grow every hot array in lockstep; a NodeId must index all of them validly.
  // Vectors are zero-initialized explicitly because glm leaves dvec3
  // uninitialized by default, and a tick would otherwise read garbage.
  position_eci.emplace_back(0.0);
  velocity_eci.emplace_back(0.0);
  position_ecef.emplace_back(0.0);

  // An unlinked terminal must read as INVALID_NODE, not 0 (which is a real
  // node), so fill rather than rely on default construction.
  auto& neighbors = isl_neighbors.emplace_back();
  neighbors.fill(INVALID_NODE);

  grid_coord.emplace_back(GridCoord{0, 0});
  catalog_id.push_back(sat.catalog_id);

  // File the cold record with its bridge index pointing at the row above.
  Satellite stored = sat;
  stored.index = index;
  by_catalog[sat.catalog_id] = std::move(stored);

  ++num_satellites;
  return index;
}

NodeId Constellation::add_ground_node(const Vec3& ecef) {
  // Ground nodes live in the array tail, past the satellite prefix. The new row
  // is the current array length; this does NOT bump num_satellites.
  const NodeId index = static_cast<NodeId>(node_count());

  // Ground carries only an ECEF position -- it does not orbit, so its inertial
  // state stays zero and is never read (the propagator touches satellites only).
  position_ecef.push_back(ecef);
  position_eci.emplace_back(0.0);
  velocity_eci.emplace_back(0.0);

  // No laser terminals on the ground; all ISL slots stay empty.
  auto& neighbors = isl_neighbors.emplace_back();
  neighbors.fill(INVALID_NODE);

  grid_coord.emplace_back(GridCoord{0, 0});  // placeholder; ground has no plane
  // catalog_id 0 is the sentinel: 0 is never a real NORAD id, so it marks a
  // non-satellite. No by_catalog entry is created.
  catalog_id.push_back(0);

  return index;
}

void Constellation::clear_ground() {
  // Truncate the ground tail back to the satellite prefix. Resizing down keeps
  // every hot array node_count()-long and leaves the post-topology state intact
  // (num_satellites, num_planes, plane_offsets, by_catalog untouched), so
  // verify() still passes and a re-query can re-append fresh endpoints.
  position_eci.resize(num_satellites);
  velocity_eci.resize(num_satellites);
  position_ecef.resize(num_satellites);
  isl_neighbors.resize(num_satellites);
  grid_coord.resize(num_satellites);
  catalog_id.resize(num_satellites);
  ground_uplinks.clear();
}

void Constellation::reserve(std::size_t n) {
  by_catalog.reserve(n);
  position_eci.reserve(n);
  velocity_eci.reserve(n);
  position_ecef.reserve(n);
  isl_neighbors.reserve(n);
  grid_coord.reserve(n);
  catalog_id.reserve(n);
}

const Satellite* Constellation::find(CatalogId cid) const {
  const auto it = by_catalog.find(cid);
  return it == by_catalog.end() ? nullptr : &it->second;
}

Satellite* Constellation::find(CatalogId cid) {
  const auto it = by_catalog.find(cid);
  return it == by_catalog.end() ? nullptr : &it->second;
}

bool Constellation::verify() const {
  const std::size_t n = node_count();  // defined by catalog_id.size()

  // Every hot array must be exactly one entry per node; a short array means a
  // NodeId could index out of bounds during a tick.
  if (position_eci.size() != n) return false;
  if (velocity_eci.size() != n) return false;
  if (position_ecef.size() != n) return false;
  if (isl_neighbors.size() != n) return false;
  if (grid_coord.size() != n) return false;

  // One cold record per satellite, no orphans.
  if (by_catalog.size() != num_satellites) return false;

  // Bridge round-trips over SATELLITES only: ground nodes occupy the tail with
  // catalog_id 0 and no by_catalog entry, so including them would falsely fail.
  for (std::size_t i = 0; i < num_satellites; ++i) {
    const auto it = by_catalog.find(catalog_id[i]);
    if (it == by_catalog.end()) return false;
    if (it->second.index != static_cast<NodeId>(i)) return false;
  }

  // Ground tail [num_satellites, n): no laser terminals and no cold record.
  // Cheap, and catches an append that landed in the wrong region.
  for (std::size_t i = num_satellites; i < n; ++i) {
    for (int t = 0; t < NUM_ISL_TERMINALS; ++t) {
      if (isl_neighbors[i][t] != INVALID_NODE) return false;
    }
    if (by_catalog.contains(catalog_id[i])) return false;
  }

  // Two valid states share this method. Before build_topology there are no
  // planes and nothing is linked; after it, the lattice scaffold must hold.
  if (num_planes == 0) {
    if (!plane_offsets.empty()) return false;  // no scaffold yet
    for (std::size_t i = 0; i < n; ++i) {
      for (int t = 0; t < NUM_ISL_TERMINALS; ++t) {
        if (isl_neighbors[i][t] != INVALID_NODE) return false;
      }
    }
    return true;
  }

  // Post-topology: plane_offsets is a non-decreasing prefix sum that brackets
  // exactly the satellite prefix.
  if (plane_offsets.size() != num_planes + 1) return false;
  if (plane_offsets.front() != 0) return false;
  if (plane_offsets.back() != num_satellites) return false;
  for (std::size_t p = 0; p < num_planes; ++p) {
    if (plane_offsets[p] > plane_offsets[p + 1]) return false;
  }

  for (std::size_t i = 0; i < num_satellites; ++i) {
    const std::size_t p = grid_coord[i].plane;
    if (p >= num_planes) return false;

    // i must fall inside its own plane's range, and slot is the offset into it.
    const NodeId lo = plane_offsets[p];
    const NodeId hi = plane_offsets[p + 1] - 1;
    if (i < lo || i > hi) return false;
    if (grid_coord[i].slot != static_cast<std::uint16_t>(i - lo)) return false;

    const auto& nb = isl_neighbors[i];

    // In-plane fore (slot 0) / aft (slot 1). Like the cross-plane slots, these
    // are now seeded per tick from geometry (seed_in_plane_links), which
    // OVERWRITES build_topology's slot-order ring -- so the invariant can no
    // longer be "nb[0] is the next slot in the ring". What survives both states
    // is membership: a linked in-plane terminal points at a satellite in this
    // satellite's OWN plane, and empty is always allowed (no reciprocating
    // neighbor within range this tick). The ring itself is still asserted, on
    // the freshly-built lattice, by topology_test.
    for (int t = 0; t < 2; ++t) {
      const NodeId x = nb[t];
      if (x == INVALID_NODE) continue;  // unlinked is fine
      if (x < lo || x > hi) return false;
      if (x == static_cast<NodeId>(i)) return false;  // never itself
    }

    // Cross-plane terminals (slots 2,3) are seeded after propagation. Accept
    // both states without a phase flag: a slot is valid if empty, or if it
    // points at a satellite in an ADJACENT plane (never i's own plane).
    const std::size_t prev = (p + num_planes - 1) % num_planes;
    const std::size_t next = (p + 1) % num_planes;
    for (int t = 2; t < NUM_ISL_TERMINALS; ++t) {
      const NodeId x = nb[t];
      if (x == INVALID_NODE) continue;  // unseeded is fine
      if (x >= num_satellites) return false;
      const std::size_t xp = grid_coord[x].plane;
      if (xp == p || (xp != prev && xp != next)) return false;
    }
  }
  return true;
}

}  // namespace leo
