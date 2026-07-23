/*****************************************************************************
  filename omm_loader.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator
  date 2026-07-22
  Brief Description:
    Implements the OMM JSON parse: maps each record's fields to a Satellite,
    converts ISO-8601 epochs to Julian Date, and skips any malformed or
    duplicate record whole (never a half-populated row). Populates a
    Constellation and returns a LoadReport tallying inserts and rejections.
 *****************************************************************************/
// omm_loader.cpp -- parse the cached OMM JSON and populate a Constellation.

// MSVC flags std::sscanf as "unsafe"; the suggested sscanf_s is non-portable,
// so opt out of the warning rather than fork the parse per platform.
#define _CRT_SECURE_NO_WARNINGS

#include "omm_loader.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace leo {

namespace {

using json = nlohmann::json;

// NORAD catalog ids are published sometimes as JSON numbers, sometimes as
// quoted strings, and are already past 5 digits -- so go through 64-bit and
// narrow, and reject anything non-numeric instead of letting it default to 0.
CatalogId parse_catalog_id(const json& j) {
  if (j.is_string()) {
    const std::string s = j.get<std::string>();
    std::size_t consumed = 0;
    const unsigned long long v = std::stoull(s, &consumed);
    if (consumed != s.size()) {
      throw std::runtime_error("NORAD_CAT_ID string not numeric: " + s);
    }
    return static_cast<CatalogId>(v);
  }
  // get<uint64_t> throws json::type_error if it is neither number nor string.
  return static_cast<CatalogId>(j.get<std::uint64_t>());
}

// Build a fully-populated Satellite from one record, or throw. Every field uses
// at() (throws if the key is missing) and get<T> (throws on wrong type); the
// caller turns any throw into a skip, so a half-parsed record never reaches
// add_satellite.
Satellite parse_record(const json& r) {
  Satellite s;
  s.object_name = r.at("OBJECT_NAME").get<std::string>();
  s.object_id = r.at("OBJECT_ID").get<std::string>();
  s.catalog_id = parse_catalog_id(r.at("NORAD_CAT_ID"));
  s.epoch_jd = iso8601_to_jd(r.at("EPOCH").get<std::string>());

  // Angles stay in degrees and mean motion in rev/day, exactly as published;
  // unit conversion is propagation's job, not the loader's.
  s.mean_motion = r.at("MEAN_MOTION").get<double>();
  s.eccentricity = r.at("ECCENTRICITY").get<double>();
  s.inclination = r.at("INCLINATION").get<double>();
  s.raan = r.at("RA_OF_ASC_NODE").get<double>();
  s.arg_pericenter = r.at("ARG_OF_PERICENTER").get<double>();
  s.mean_anomaly = r.at("MEAN_ANOMALY").get<double>();
  s.bstar = r.at("BSTAR").get<double>();
  s.mean_motion_dot = r.at("MEAN_MOTION_DOT").get<double>();
  s.mean_motion_ddot = r.at("MEAN_MOTION_DDOT").get<double>();

  const std::string cls = r.at("CLASSIFICATION_TYPE").get<std::string>();
  if (cls.empty()) throw std::runtime_error("empty CLASSIFICATION_TYPE");
  s.classification = cls[0];  // 'U' / 'C' / 'S'

  s.element_set_no = r.at("ELEMENT_SET_NO").get<int>();
  s.rev_at_epoch = r.at("REV_AT_EPOCH").get<int>();
  s.ephemeris_type = r.at("EPHEMERIS_TYPE").get<int>();
  return s;
}

}  // namespace

double iso8601_to_jd(std::string_view iso) {
  int year = 0, month = 0, day = 0, hour = 0, minute = 0;
  double second = 0.0;

  // string_view is not guaranteed null-terminated; sscanf needs a C string.
  // %lf on the seconds field absorbs the optional fractional ".ffffff".
  const std::string s(iso);
  const int matched = std::sscanf(s.c_str(), "%d-%d-%dT%d:%d:%lf", &year,
                                  &month, &day, &hour, &minute, &second);
  if (matched != 6) {
    throw std::runtime_error("malformed ISO-8601 epoch: " + s);
  }

  // Standard Gregorian -> Julian Day Number (integer day count). The /12, /5,
  // etc. are deliberate integer divisions from the published algorithm; the
  // result counts whole days from noon, which is why the clock subtracts 12h.
  const long long a = (14 - month) / 12;
  const long long y = static_cast<long long>(year) + 4800 - a;
  const long long m = month + 12 * a - 3;
  const long long jdn = day + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100
                        + y / 400 - 32045;

  // Add the time-of-day fraction; seconds carry the sub-second part.
  return static_cast<double>(jdn) + (hour - 12) / 24.0 + minute / 1440.0
         + second / 86400.0;
}

std::string jd_to_iso8601(double jd) {
  // Fliegel/Van Flandern style JD -> Gregorian. The +0.5 shifts the noon-based
  // JD so that the integer part z lands on a civil-day boundary, and f is the
  // elapsed fraction of that day.
  const double shifted = jd + 0.5;
  long long z = static_cast<long long>(std::floor(shifted));
  double f = shifted - static_cast<double>(z);

  // Gregorian correction (always applies for the modern dates we handle).
  const long long alpha =
      static_cast<long long>(std::floor((z - 1867216.25) / 36524.25));
  const long long a = z + 1 + alpha - alpha / 4;
  const long long b = a + 1524;
  const long long c = static_cast<long long>(std::floor((b - 122.1) / 365.25));
  const long long d = static_cast<long long>(std::floor(365.25 * c));
  const long long e =
      static_cast<long long>(std::floor((b - d) / 30.6001));

  const int day = static_cast<int>(b - d - static_cast<long long>(30.6001 * e));
  const int month = static_cast<int>((e < 14) ? e - 1 : e - 13);
  const int year = static_cast<int>((month > 2) ? c - 4716 : c - 4715);

  // Time of day from the day fraction. Carry a rounded-up 60.000000s back into
  // the minutes so formatting never prints ":60".
  double rem = f * 86400.0;  // seconds into the day
  int hour = static_cast<int>(rem / 3600.0);
  rem -= hour * 3600.0;
  int minute = static_cast<int>(rem / 60.0);
  double second = rem - minute * 60.0;
  if (second >= 59.9999995) {  // guard the 6-decimal print rounding to 60
    second = 0.0;
    if (++minute == 60) { minute = 0; ++hour; }
  }

  char buf[40];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%09.6f", year, month,
                day, hour, minute, second);
  return std::string(buf);
}

LoadReport load_omm_json(const std::filesystem::path& file, Constellation& out) {
  LoadReport report;

  // Binary so no platform newline translation alters byte offsets in errors.
  std::ifstream in(file, std::ios::binary);
  if (!in) {
    report.errors.push_back("cannot open file: " + file.string());
    return report;
  }

  json doc;
  try {
    in >> doc;
  } catch (const std::exception& e) {
    report.errors.push_back(std::string("JSON parse error: ") + e.what());
    return report;
  }

  if (!doc.is_array()) {
    report.errors.push_back("top-level JSON is not an array");
    return report;
  }

  // One growth up front; the array size is a tight upper bound on insertions.
  out.reserve(out.node_count() + doc.size());

  for (std::size_t i = 0; i < doc.size(); ++i) {
    try {
      Satellite sat = parse_record(doc[i]);

      // Skip duplicates rather than let add_satellite hand back an existing
      // row -- counting it as inserted would overstate the population.
      if (out.find(sat.catalog_id) != nullptr) {
        ++report.skipped;
        report.errors.push_back("record " + std::to_string(i)
                                + ": duplicate NORAD_CAT_ID "
                                + std::to_string(sat.catalog_id));
        continue;
      }

      out.add_satellite(sat);
      ++report.inserted;
    } catch (const std::exception& e) {
      // Any missing/wrong-typed field throws here: skip the whole record.
      ++report.skipped;
      report.errors.push_back("record " + std::to_string(i) + ": " + e.what());
    }
  }

  // The whole point of the loader is to leave the structure consistent.
  assert(out.verify());
  return report;
}

}  // namespace leo
