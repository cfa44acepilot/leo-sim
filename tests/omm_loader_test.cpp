/*****************************************************************************
  filename omm_loader_test.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-22
  Brief Description:
    Unit tests for the OMM JSON loader: field mapping, ISO-8601 epoch to Julian
    Date conversion, whole-record rejection of malformed/duplicate entries, and
    the post-load structural guarantee. Additive to constellation_test.cpp.
 *****************************************************************************/
// omm_loader_test.cpp -- JSON load, field mapping, epoch conversion, and the
// post-load structural guarantee. Additive to constellation_test.cpp.

#include "omm_loader.hpp"

#include <filesystem>

#include <gtest/gtest.h>

#include "constellation.hpp"

namespace {

using leo::Constellation;
using leo::iso8601_to_jd;
using leo::LoadReport;
using leo::Satellite;

// Path to the committed fixtures; CMake injects the source-tree data dir so the
// test runs from any build directory on either platform.
std::filesystem::path fixture(const char* name) {
  return std::filesystem::path(LEO_TEST_DATA_DIR) / name;
}

// The fixture has 4 records: a 6-digit catalog id, a catalog id given as a JSON
// string, a fully-valid spot-check record, and one missing INCLINATION. Three
// must load, the malformed one must be skipped, and the structure must verify.
TEST(OmmLoader, LoadsFixtureAndSkipsInvalid) {
  Constellation c;
  const LoadReport rep = leo::load_omm_json(fixture("test_omm.json"), c);

  EXPECT_EQ(rep.inserted, 3u);
  EXPECT_EQ(rep.skipped, 1u);
  EXPECT_EQ(rep.errors.size(), 1u);  // the skip carries one message

  EXPECT_EQ(c.node_count(), 3u);
  EXPECT_EQ(c.satellite_count(), 3u);
  EXPECT_TRUE(c.verify());
}

// The string-encoded NORAD_CAT_ID and the 6-digit id must both round-trip; an
// id is never assumed to fit in 5 digits.
TEST(OmmLoader, ParsesCatalogIdFromNumberAndString) {
  Constellation c;
  leo::load_omm_json(fixture("test_omm.json"), c);

  ASSERT_NE(c.find(44715), nullptr);   // arrived as the string "44715"
  ASSERT_NE(c.find(270001), nullptr);  // 6-digit number
  EXPECT_EQ(c.find(44716), nullptr);   // the skipped (missing-field) record
}

// Spot-check the known record against hand-verified published values.
TEST(OmmLoader, SpotChecksKnownRecord) {
  Constellation c;
  leo::load_omm_json(fixture("test_omm.json"), c);

  const Satellite* s = c.find(44714);  // STARLINK-1008
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->object_name, "STARLINK-1008");
  EXPECT_EQ(s->catalog_id, 44714u);
  EXPECT_DOUBLE_EQ(s->inclination, 53.1508);
  EXPECT_DOUBLE_EQ(s->raan, 97.7628);

  // EPOCH "2026-06-11T02:53:00.859776" -> JD, computed by hand from the
  // Gregorian JDN (2461203 at noon) minus the morning offset.
  EXPECT_NEAR(s->epoch_jd, 2461202.6201488413, 1e-6);
}

// J2000 is the algorithm's anchor; midnight that same day pins the fraction.
TEST(OmmLoader, Iso8601ToJulianDate) {
  EXPECT_NEAR(iso8601_to_jd("2000-01-01T12:00:00"), 2451545.0, 1e-6);  // J2000
  EXPECT_NEAR(iso8601_to_jd("2000-01-01T00:00:00"), 2451544.5, 1e-6);
}

// jd_to_iso8601 inverts the parser: J2000 formats exactly, and a microsecond
// timestamp round-trips back to the same JD.
TEST(OmmLoader, JdToIso8601RoundTrips) {
  EXPECT_EQ(leo::jd_to_iso8601(2451545.0).substr(0, 19), "2000-01-01T12:00:00");

  const double jd = iso8601_to_jd("2026-06-11T02:53:00.859776");
  const double back = iso8601_to_jd(leo::jd_to_iso8601(jd));
  EXPECT_NEAR(back, jd, 1e-6);
}

}  // namespace
