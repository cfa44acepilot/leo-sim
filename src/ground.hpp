// ground.hpp
//
// Ground stations (source/dest endpoints) as nodes in the array tail. A ground
// node has a fixed ECEF position derived from geodetic coordinates; its uplinks
// (the satellites it can currently see) are recomputed each tick from satellite
// positions. Runs AFTER Propagator::propagate(). Like ISL links, uplink weights
// are NOT stored here -- routing (step 8) derives them and treats uplinks as
// undirected.

#ifndef LEO_GROUND_HPP
#define LEO_GROUND_HPP

#include "constellation.hpp"

namespace leo {

// Geodetic (WGS84) -> ECEF, in kilometers. lat/lon in degrees, altitude in km
// above the ellipsoid. The returned frame matches the propagator's
// position_ecef: x points through the Greenwich meridian, because the
// propagator rotates TEME by GMST (measured to Greenwich), so longitude lines
// up directly with no extra offset.
Vec3 geodetic_to_ecef(double lat_deg, double lon_deg, double alt_km);

// Uplink visibility threshold.
struct UplinkSpec {
  double min_elevation_deg = 25.0;
};

// Recompute every ground node's uplink list from the current satellite
// positions. Idempotent: clears and refills ground_uplinks (and resizes it to
// ground_count(), so it stays correct if stations are added between calls). A
// satellite is visible iff its elevation above the station's local horizon is
// >= min_elevation_deg.
void connect_ground_uplinks(Constellation& c, const UplinkSpec& spec = {});

}  // namespace leo

#endif  // LEO_GROUND_HPP
