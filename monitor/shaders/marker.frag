#version 450
// Ground-endpoint "no coverage" marker: an amber RING (annulus) so it reads as a
// deliberate marker -- not a satellite dot -- and does not fully obscure the
// point it flags. gl_PointCoord is 0..1 across the square point sprite.

layout(location = 0) in vec3 vColor;
layout(location = 0) out vec4 outColor;

void main() {
  float r = length(gl_PointCoord - vec2(0.5));
  if (r > 0.5 || r < 0.34) discard;  // keep only the outer ring band
  outColor = vec4(vColor, 1.0);
}
