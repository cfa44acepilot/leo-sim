// ground.cpp -- geodetic placement and uplink visibility for ground stations.

#include "ground.hpp"

#include <cmath>
#include <cstddef>
#include <numbers>

#include <glm/geometric.hpp>  // glm::dot, length, normalize

namespace leo {

namespace {

constexpr double kDeg2Rad = std::numbers::pi / 180.0;

// WGS84 ellipsoid. ~2 m off the propagator's wgs72 radius, irrelevant here.
constexpr double kA = 6378.137;             // semi-major axis (km)
constexpr double kF = 1.0 / 298.257223563;  // flattening
constexpr double kE2 = kF * (2.0 - kF);     // first eccentricity squared

// A satellite with a zeroed position (failed propagation step) sits at the
// origin; any real satellite is well above the Earth's radius. Use that to skip
// the dead ones.
constexpr double kEarthRadiusKm = kA;

}  // namespace

Vec3 geodetic_to_ecef(double lat_deg, double lon_deg, double alt_km) {
  const double lat = lat_deg * kDeg2Rad;
  const double lon = lon_deg * kDeg2Rad;
  const double slat = std::sin(lat);
  const double clat = std::cos(lat);

  // N: radius of curvature in the prime vertical at this latitude.
  const double n = kA / std::sqrt(1.0 - kE2 * slat * slat);

  return Vec3((n + alt_km) * clat * std::cos(lon),
              (n + alt_km) * clat * std::sin(lon),
              (n * (1.0 - kE2) + alt_km) * slat);
}

void connect_ground_uplinks(Constellation& c, const UplinkSpec& spec) {
  // Resize (and clear) so the list count tracks stations added between calls.
  const std::size_t g_count = c.ground_count();
  c.ground_uplinks.assign(g_count, {});

  // Compare sin(elevation) directly to avoid a per-satellite asin/acos.
  const double sin_min = std::sin(spec.min_elevation_deg * kDeg2Rad);

  for (std::size_t g = 0; g < g_count; ++g) {
    const NodeId gnode = static_cast<NodeId>(c.num_satellites + g);
    const Vec3& gpos = c.position_ecef[gnode];
    // Geocentric local vertical (a sub-0.2-deg approximation to the true
    // geodetic normal -- fine for v1).
    const Vec3 up = glm::normalize(gpos);

    std::vector<NodeId>& list = c.ground_uplinks[g];
    for (std::size_t s = 0; s < c.num_satellites; ++s) {
      const Vec3& sp = c.position_ecef[s];
      if (glm::length(sp) < kEarthRadiusKm) continue;  // dead/zeroed position

      // Elevation angle of the satellite above the station's horizon. A
      // positive minimum already keeps the sight line above the limb, so no
      // separate Earth-occlusion test is needed (unlike the ISL case).
      const double sin_elev = glm::dot(up, glm::normalize(sp - gpos));
      if (sin_elev >= sin_min) list.push_back(static_cast<NodeId>(s));
    }
  }
}

}  // namespace leo
