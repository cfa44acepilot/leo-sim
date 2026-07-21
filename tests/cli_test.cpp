// cli_test.cpp -- argument resolution glue (routing is covered elsewhere).

#include "cli.hpp"

#include <filesystem>

#include <gtest/gtest.h>

#include "constellation.hpp"
#include "epoch_health.hpp"  // now_jd_utc
#include "ground.hpp"        // geodetic_to_ecef
#include "simulator.hpp"

namespace {

using leo::CliOptions;
using leo::Constellation;
using leo::now_jd_utc;
using leo::omm_data_age_days;
using leo::parse_args;
using leo::parse_lat_lon;
using leo::resolve_endpoints;
using leo::Simulator;
using leo::Vec3;

std::filesystem::path fixture(const char* name) {
  return std::filesystem::path(LEO_TEST_DATA_DIR) / name;
}

// "lat,lon" parses to the right numbers; a bare name does not parse.
TEST(Cli, ParseLatLon) {
  const auto ll = parse_lat_lon("41.88,-87.63");
  ASSERT_TRUE(ll.has_value());
  EXPECT_NEAR(ll->first, 41.88, 1e-9);
  EXPECT_NEAR(ll->second, -87.63, 1e-9);

  EXPECT_FALSE(parse_lat_lon("Chicago").has_value());      // name
  EXPECT_FALSE(parse_lat_lon("1,2,3").has_value());        // too many parts
  EXPECT_FALSE(parse_lat_lon("12.5").has_value());         // no comma
  EXPECT_FALSE(parse_lat_lon("1.0,abc").has_value());      // junk component
}

// parse_args pulls out positionals and a valued flag.
TEST(Cli, ParseArgs) {
  const char* argv[] = {"leo", "Chicago", "London", "--hop-cost", "0.003"};
  CliOptions opts;
  std::string err;
  ASSERT_TRUE(parse_args(5, const_cast<char**>(argv), opts, err)) << err;
  EXPECT_EQ(opts.src, "Chicago");
  EXPECT_EQ(opts.dst, "London");
  EXPECT_NEAR(opts.hop_cost_s, 0.003, 1e-12);

  // Missing both endpoints is a usage error.
  const char* bad[] = {"leo", "Chicago"};
  CliOptions o2;
  std::string e2;
  EXPECT_FALSE(parse_args(2, const_cast<char**>(bad), o2, e2));
}

// A "lat,lon" endpoint is placed at the matching geodetic ECEF position.
TEST(Cli, ResolveCoordinatePlacesGroundNode) {
  Simulator sim;
  ASSERT_TRUE(sim.load(fixture("test_omm.json"), fixture("test_stations.json")));

  std::string err;
  // src is a coordinate, dst is a known station ("Alpha" in the fixture).
  ASSERT_TRUE(resolve_endpoints(sim, "1.5,2.5", "Alpha", err)) << err;

  const Constellation& c = sim.constellation();
  ASSERT_EQ(c.ground_count(), 2u);
  const Vec3 expected = leo::geodetic_to_ecef(1.5, 2.5, 0.0);
  const Vec3 got = c.position_ecef[c.num_satellites];  // first ground node = src
  EXPECT_NEAR(got.x, expected.x, 1e-6);
  EXPECT_NEAR(got.y, expected.y, 1e-6);
  EXPECT_NEAR(got.z, expected.z, 1e-6);
}

// omm_data_age_days reads the newest epoch's age; a missing file is negative.
TEST(Cli, OmmDataAgeDays) {
  // test_omm.json's newest record (STARLINK-1008) has epoch JD 2461202.6201488.
  const double expected = now_jd_utc() - 2461202.6201488;
  const double age = omm_data_age_days(fixture("test_omm.json"));
  EXPECT_NEAR(age, expected, 0.01);  // within a fraction of a day

  EXPECT_LT(omm_data_age_days(fixture("does_not_exist.json")), 0.0);
}

// An unknown station name takes the error path and leaves the sim unchanged.
TEST(Cli, ResolveUnknownNameErrors) {
  Simulator sim;
  ASSERT_TRUE(sim.load(fixture("test_omm.json"), fixture("test_stations.json")));

  std::string err;
  EXPECT_FALSE(resolve_endpoints(sim, "Alpha", "Nowhereville", err));
  EXPECT_FALSE(err.empty());
  EXPECT_EQ(sim.constellation().ground_count(), 0u);  // nothing placed
}

}  // namespace
