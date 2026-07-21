#version 450
// One satellite per vertex (POINT_LIST). Positions arrive already in world
// space; we set a fixed screen-space point size so the cloud is visible at any
// zoom. Color is uniform for the whole cloud via the push constant.

layout(location = 0) in vec3 inPos;

layout(push_constant) uniform PC {
  mat4 mvp;
  vec4 color;
} pc;

layout(location = 0) out vec4 vColor;

void main() {
  gl_Position = pc.mvp * vec4(inPos, 1.0);
  gl_PointSize = 2.5;
  vColor = pc.color;
}
