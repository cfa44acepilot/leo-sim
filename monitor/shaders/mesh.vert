#version 450
// Lit sphere (Earth + atmosphere). Geometry is already in world space (the
// Earth is fixed at the ECEF origin), so the push-constant matrix is just
// proj*view and the normal passes through unchanged.

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
