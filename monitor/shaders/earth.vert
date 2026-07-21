#version 450
// Textured Earth. Same transform as mesh.vert, but the fragment shader samples
// an equirectangular day-map, so we only need to pass the (world-space) normal
// -- which for a sphere centered at the origin is the surface direction used to
// derive latitude/longitude UVs.

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;

layout(push_constant) uniform PC {
  mat4 mvp;
  vec4 color;
} pc;

layout(location = 0) out vec3 vNormal;

void main() {
  gl_Position = pc.mvp * vec4(inPos, 1.0);
  vNormal = inNormal;
}
