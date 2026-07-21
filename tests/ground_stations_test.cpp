// ground_stations_test.cpp -- catalog loading and validation.

#include "ground_stations.hpp"

#include <filesystem>

#include <gtest/gtest.h>

namespace {

using leo::GroundStation;
using leo::load_ground_stations;
using leo::StationLoadReport;

std::filesystem::path fixture(const char* name) {
  return std::filesystem::path(LEO_TEST_DATA_DIR) / name;
}

// The fixture has 2 valid stations plus a duplicate name, an out-of-range lat,
// and an out-of-range lon -> 2 loaded, 3 skipped, and names resolve.
TEST(GroundStations, LoadsValidAndRejectsBad) {
  std::vector<GroundStation> out;
  const StationLoadReport rep = load_ground_stations(fixture("test_stations.json"), out);

  EXPECT_EQ(rep.loaded, 2u);
  EXPECT_EQ(rep.skipped, 3u);
  ASSERT_EQ(out.size(), 2u);

  EXPECT_EQ(out[0].name, "Alpha");
  EXPECT_NEAR(out[0].lat_deg, 40.0, 1e-9);
  EXPECT_NEAR(out[0].lon_deg, -74.0, 1e-9);
  EXPECT_NEAR(out[0].alt_km, 0.01, 1e-9);
  EXPECT_EQ(out[1].name, "Bravo");
  EXPECT_NEAR(out[1].alt_km, 0.0, 1e-9);  // alt defaults to 0
}

}  // namespace
