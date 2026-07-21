#version 450
// Round off the square point sprite: discard fragments outside the inscribed
// circle so satellites read as dots, not squares.

layout(location = 0) in vec4 vColor;
layout(location = 0) out vec4 outColor;

void main() {
  vec2 d = gl_PointCoord - vec2(0.5);
  if (dot(d, d) > 0.25) discard;
  outColor = vColor;
}
