// constellation_test.cpp -- structural invariants of the hot/cold layout.

#include "constellation.hpp"

#include <array>

#include <gtest/gtest.h>

namespace {

using leo::CatalogId;
using leo::Constellation;
using leo::INVALID_NODE;
using leo::NodeId;
using leo::NUM_ISL_TERMINALS;
using leo::Satellite;

// Minimal cold record; only the catalog id matters for layout tests.
Satellite make_sat(CatalogId cid, const char* name) {
  Satellite s;
  s.catalog_id = cid;
  s.object_name = name;
  return s;
}

// Every dense array must stay exactly node_count() long, the map must hold one
// entry per added satellite, and the NodeId<->CatalogId bridge must round-trip
// in both directions.
TEST(Constellation, AddKeepsArraysAndBridgeConsistent) {
  Constellation c;
  const std::array<CatalogId, 3> cids = {44714, 44715, 100200};  // 6-digit ok

  for (std::size_t i = 0; i < cids.size(); ++i) {
    const NodeId got = c.add_satellite(make_sat(cids[i], "STARLINK-X"));
    EXPECT_EQ(got, static_cast<NodeId>(i));  // dense, sequential
  }

  const std::size_t n = c.node_count();
  EXPECT_EQ(n, cids.size());
  EXPECT_EQ(c.satellite_count(), cids.size());
  EXPECT_EQ(c.by_catalog.size(), cids.size());

  // All hot arrays share the node count.
  EXPECT_EQ(c.position_eci.size(), n);
  EXPECT_EQ(c.velocity_eci.size(), n);
  EXPECT_EQ(c.position_ecef.size(), n);
  EXPECT_EQ(c.isl_neighbors.size(), n);
  EXPECT_EQ(c.grid_coord.size(), n);
  EXPECT_EQ(c.catalog_id.size(), n);

  // index round-trips: catalog -> record.index -> catalog.
  for (std::size_t i = 0; i < cids.size(); ++i) {
    const CatalogId cid = cids[i];
    ASSERT_TRUE(c.by_catalog.contains(cid));
    EXPECT_EQ(c.by_catalog[cid].index, static_cast<NodeId>(i));
    EXPECT_EQ(c.catalog_at(static_cast<NodeId>(i)), cid);

    const Satellite* found = c.find(cid);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->index, static_cast<NodeId>(i));
  }

  EXPECT_EQ(c.find(999999), nullptr);  // absent
}

// Freshly added nodes have all ISL terminals empty; the slots are mutable and
// hold dense NodeIds (or the sentinel), never catalog ids.
TEST(Constellation, NeighborSlotsDefaultToSentinelAndAreWritable) {
  Constellation c;
  const NodeId a = c.add_satellite(make_sat(1, "A"));
  const NodeId b = c.add_satellite(make_sat(2, "B"));

  for (int t = 0; t < NUM_ISL_TERMINALS; ++t) {
    EXPECT_EQ(c.isl_neighbors[a][t], INVALID_NODE);
    EXPECT_EQ(c.isl_neighbors[b][t], INVALID_NODE);
  }

  // Link two terminals, leave the rest as sentinels.
  c.isl_neighbors[a][0] = b;
  c.isl_neighbors[b][1] = a;

  EXPECT_EQ(c.isl_neighbors[a][0], b);
  EXPECT_EQ(c.isl_neighbors[a][1], INVALID_NODE);
  EXPECT_EQ(c.isl_neighbors[b][1], a);
  EXPECT_EQ(c.isl_neighbors[b][0], INVALID_NODE);
}

// Re-adding an existing catalog id must not grow the structure: the guard hands
// back the existing row instead of pushing a desynced duplicate. (The loader
// skips dupes before reaching add_satellite, so this is the only coverage.)
TEST(Constellation, DuplicateCatalogIdReturnsSameNode) {
  Constellation c;
  const NodeId first = c.add_satellite(make_sat(44714, "STARLINK-A"));

  const std::size_t nodes_before = c.node_count();
  const std::size_t map_before = c.by_catalog.size();

  const NodeId again = c.add_satellite(make_sat(44714, "STARLINK-A-DUP"));
  EXPECT_EQ(again, first);  // same dense row, not a new one
  EXPECT_EQ(c.node_count(), nodes_before);
  EXPECT_EQ(c.by_catalog.size(), map_before);
}

// reserve() is a capacity hint only: logical size must be untouched.
TEST(Constellation, ReserveDoesNotChangeLogicalSize) {
  Constellation c;
  c.add_satellite(make_sat(42, "Solo"));

  const std::size_t before = c.node_count();
  c.reserve(10'000);

  EXPECT_EQ(c.node_count(), before);
  EXPECT_EQ(c.satellite_count(), before);
  EXPECT_EQ(c.by_catalog.size(), before);
  EXPECT_GE(c.position_eci.capacity(), 10'000u);
}

}  // namespace
