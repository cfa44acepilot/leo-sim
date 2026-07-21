#version 450
// Selection marker as a screen-space point. gl_PointSize comes from the push-
// constant alpha, so the stencil mask pass (inner disc) and the outline pass
// (outer disc) share one shader at two sizes. Position is a bare world-space
// vec3, the same vertex layout as the satellite point cloud.

layout(location = 0) in vec3 inPos;

layout(push_constant) uniform PC {
  mat4 mvp;
  vec4 color;  // rgb = ring color, a = point size in pixels
} pc;

layout(location = 0) out vec3 vColor;

void main() {
  gl_Position = pc.mvp * vec4(inPos, 1.0);
  gl_PointSize = pc.color.a;
  vColor = pc.color.rgb;
}
