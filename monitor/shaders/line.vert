#version 450
// Colored lines for the ISL/uplink edges and the route path (LINE_LIST). Color
// is per-vertex (baked on the CPU from Edge::Kind / path highlight), so the
// push-constant color is unused here -- only the matrix matters.

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inColor;

layout(push_constant) uniform PC {
  mat4 mvp;
  vec4 color;
} pc;

layout(location = 0) out vec3 vColor;

void main() {
  gl_Position = pc.mvp * vec4(inPos, 1.0);
  vColor = inColor;
}
