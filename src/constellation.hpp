/*****************************************************************************
  filename constellation.hpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-22
  Brief Description:
    Declares the core in-memory container: the hot/cold data-oriented layout.
    Per-tick state (positions, velocities, ISL links) lives in parallel dense
    arrays indexed by a packed NodeId, while cold identity (catalog id, orbital
    elements) lives in a hashmap. This header defines the types every other
    module is built on.
 *****************************************************************************/
// constellation.hpp
//
// Core in-memory data layout for the LEO constellation simulator.
//
// Design: hot/cold split (data-oriented). The per-tick simulation reads and
// writes positions, velocities, and ISL neighbor links for every node, every
// step -- that data lives in parallel dense arrays (struct-of-arrays) so a tick
// streams it linearly and cache-friendly. The rarely-touched identity of a
// satellite (names, orbital elements, catalog id) is "cold" and lives in a
// hashmap keyed by NORAD catalog id. A single dense NodeId bridges the two:
// it indexes every hot array and is stored back inside the cold record.

#ifndef LEO_CONSTELLATION_HPP
#define LEO_CONSTELLATION_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <ankerl/unordered_dense.h>
#include <glm/vec3.hpp>

namespace leo {

// Dense, contiguous index into the hot arrays. Assigned by the container, not
// taken from the data, so it stays packed regardless of catalog id gaps.
using NodeId = std::uint32_t;

// NORAD catalog number. Already exceeds 5 digits and is heading past 6, so it
// needs a full 32-bit range -- it is NOT interchangeable with a NodeId.
using CatalogId = std::uint32_t;

// Sentinel for an ISL terminal that is not currently linked to anything.
constexpr NodeId INVALID_NODE = std::numeric_limits<NodeId>::max();

// Laser inter-satellite-link terminals per satellite (typically 4: fore, aft,
// and two cross-plane). Configurable default; fixed at compile time so the
// neighbor slot lives inline in the dense array with no extra indirection.
constexpr int NUM_ISL_TERMINALS = 4;

// Double precision throughout. UNITS: kilometers and km/s everywhere (SGP4's
// native output; the propagator writes km). Orbital radii are ~7e3 km and we
// integrate over many orbits, so float's ~7 significant digits would lose
// meaningful precision per step. Routing weights and the renderer must use km.
using Vec3 = glm::dvec3;

// A satellite's address within its shell: which orbital plane, and which slot
// (phase position) along that plane. Set once at load from the geometry.
struct GridCoord {
  std::uint16_t plane;
  std::uint16_t slot;
};

// Cold identity record: the parsed OMM fields plus the bridge index. This is
// the hashmap value, touched on lookup/handover, NOT every tick. It must never
// hold position, velocity, or neighbor data -- that is hot and lives in the
// dense arrays.
struct Satellite {
  std::string object_name;  // e.g. "STARLINK-1008"
  std::string object_id;    // international designator, e.g. "2019-074B"
  CatalogId catalog_id = 0;  // NORAD_CAT_ID

  // Time the elements are valid for, as a Julian Date. The JSON parser fills
  // this later; accept a plain double for now.
  double epoch_jd = 0.0;

  // OMM mean orbital elements (degrees / rev-per-day, as published).
  double mean_motion = 0.0;
  double eccentricity = 0.0;
  double inclination = 0.0;
  double raan = 0.0;            // right ascension of the ascending node
  double arg_pericenter = 0.0;  // argument of perigee
  double mean_anomaly = 0.0;
  double bstar = 0.0;             // drag-like term
  double mean_motion_dot = 0.0;  // first derivative of mean motion
  double mean_motion_ddot = 0.0;  // second derivative

  char classification = 'U';  // 'U' unclassified
  int element_set_no = 0;
  int rev_at_epoch = 0;
  int ephemeris_type = 0;

  // Bridge to the hot arrays: this record's row. Set by add_satellite.
  NodeId index = INVALID_NODE;
};

// The constellation: a cold identity map plus parallel dense hot arrays. Every
// dense array is sized to the node count and indexed by NodeId. Satellites
// occupy [0, num_satellites); ground nodes will occupy the tail later, which is
// why node_count() (array length) is distinct from satellite_count().
struct Constellation {
  // Cold: identity + elements, keyed by catalog id.
  ankerl::unordered_dense::map<CatalogId, Satellite> by_catalog;

  // Hot: one entry per node, all kept the same length as catalog_id.
  std::vector<Vec3> position_eci;   // integrator state (inertial)
  std::vector<Vec3> velocity_eci;   // integrator state (inertial)
  std::vector<Vec3> position_ecef;  // derived each tick (Earth-fixed)
  std::vector<std::array<NodeId, NUM_ISL_TERMINALS>> isl_neighbors;
  std::vector<GridCoord> grid_coord;  // shell address, set at load

  // Reverse of by_catalog's index: NodeId -> catalog id, so a hot-loop result
  // can recover identity without scanning the map.
  std::vector<CatalogId> catalog_id;

  // Satellites occupy the dense prefix [0, num_satellites). Distinct from the
  // total array length once ground nodes are appended to the tail.
  std::size_t num_satellites = 0;

  // Static lattice scaffold, filled by build_topology after load. Satellites
  // are sorted by (plane, slot); plane p occupies the contiguous dense range
  // [plane_offsets[p], plane_offsets[p+1]). plane_offsets is a prefix sum of
  // length num_planes + 1 ([0] == 0, [num_planes] == num_satellites). Both stay
  // empty/zero until build_topology runs.
  std::vector<NodeId> plane_offsets;
  std::size_t num_planes = 0;

  // Visible satellites per ground node, recomputed each tick by
  // connect_ground_uplinks. ground_uplinks[g] is the uplink list for the ground
  // node at NodeId (num_satellites + g). Kept here, not in isl_neighbors,
  // because one station can see many satellites (no fixed terminal count).
  std::vector<std::vector<NodeId>> ground_uplinks;

  // Appends one satellite: assigns the next dense index, grows EVERY hot array
  // by one default entry (so they stay equal length), points the new node's ISL
  // terminals at INVALID_NODE, files the record in by_catalog with its index
  // set, bumps num_satellites, and returns the new NodeId.
  NodeId add_satellite(const Satellite& sat);

  // Appends a ground station to the array tail: grows every hot array by one
  // (ECEF set, inertial state zero, ISL slots empty, catalog_id 0 sentinel).
  // Does NOT bump num_satellites or touch by_catalog. Returns the new NodeId
  // (>= num_satellites). Call after build_topology so it lands in the tail.
  NodeId add_ground_node(const Vec3& ecef);

  // Drop all ground nodes: truncate every hot array back to num_satellites and
  // clear ground_uplinks, leaving the post-topology state. Lets a re-query swap
  // endpoints (add_ground_node only appends, so old ones must be removed first).
  void clear_ground();

  // Reserve capacity across every array and the map. Pure capacity hint: it
  // does not change logical size.
  void reserve(std::size_t n);

  // Satellites only -- the dense prefix.
  std::size_t satellite_count() const { return num_satellites; }

  // Total nodes = length of the hot arrays (satellites now, + ground later).
  std::size_t node_count() const { return catalog_id.size(); }

  // Ground nodes occupy the tail [num_satellites, node_count()).
  std::size_t ground_count() const { return node_count() - num_satellites; }
  bool is_ground(NodeId n) const { return n >= num_satellites; }

  // Cold lookup by catalog id; nullptr if absent.
  const Satellite* find(CatalogId cid) const;
  Satellite* find(CatalogId cid);

  // Reverse lookup: dense index -> catalog id.
  CatalogId catalog_at(NodeId index) const { return catalog_id[index]; }

  // Full structural audit of the hot/cold layout. Returns false unless every
  // dense array length matches node_count(), the map size matches the satellite
  // count, the NodeId<->CatalogId bridge round-trips both ways for every row,
  // and no ISL link has been set yet. Called at the end of a load (asserted in
  // debug) and exercised directly by tests.
  bool verify() const;
};

}  // namespace leo

#endif  // LEO_CONSTELLATION_HPP
