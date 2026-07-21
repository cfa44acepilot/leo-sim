#version 450
// Discard fragments outside the point sprite's inscribed disc (gl_PointCoord is
// 0..1 across the square point), so the stencil mask and the outline both read as
// clean circles rather than squares -> a circular selection ring.

layout(location = 0) in vec3 vColor;
layout(location = 0) out vec4 outColor;

void main() {
  vec2 d = gl_PointCoord - vec2(0.5);
  if (dot(d, d) > 0.25) discard;  // radius 0.5 -> r^2 = 0.25
  outColor = vec4(vColor, 1.0);
}
