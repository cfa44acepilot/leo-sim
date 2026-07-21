#version 450
// Sample an equirectangular Earth day-map. The frame is ECEF: +z is the polar
// axis and +x points at the prime meridian (lat 0, lon 0), so latitude comes
// from N.z and longitude from atan2(N.y, N.x). A fixed key light gives a simple
// day/night terminator (the Earth is drawn fixed, so the lit side is static --
// this is the Earth-locked view mode).

layout(location = 0) in vec3 vNormal;

layout(set = 0, binding = 0) uniform sampler2D uEarth;

layout(location = 0) out vec4 outColor;

const float PI = 3.14159265359;

void main() {
  vec3 N = normalize(vNormal);
  float u = 0.5 + atan(N.y, N.x) / (2.0 * PI);
  float v = 0.5 - asin(clamp(N.z, -1.0, 1.0)) / PI;
  vec3 albedo = texture(uEarth, vec2(u, v)).rgb;

  vec3 L = normalize(vec3(0.6, 0.7, 0.5));
  float lambert = max(dot(N, L), 0.0) * 0.9 + 0.15;  // 0.15 ambient (night side)
  outColor = vec4(albedo * lambert, 1.0);
}
