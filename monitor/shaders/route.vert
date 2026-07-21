#version 450
// Thick route highlight drawn as a screen-space billboard line. The CPU expands
// each path segment into a quad (six vertices); here we push each corner
// perpendicular to the segment's ON-SCREEN direction by a fixed pixel half-width,
// so the route keeps a constant thickness at any zoom and on any GPU.
//
// Why not gl line width: Vulkan lineWidth > 1.0 requires the wideLines device
// feature and is driver-dependent (often clamped to 1.0), so relying on it would
// make the route thin or invisible on many GPUs. Expanding to a quad is
// guaranteed to work everywhere.

layout(location = 0) in vec3 inPos;     // this endpoint (world)
layout(location = 1) in vec3 inOther;   // opposite endpoint (world)
layout(location = 2) in vec3 inColor;   // route color
layout(location = 3) in float inSide;   // +1 / -1: offset direction for this vtx
layout(location = 4) in float inAcross; // -1..+1: physical band position

layout(push_constant) uniform PC {
  mat4 mvp;
  vec4 color;  // repurposed for the route: xy = viewport px, z = half-width px
} pc;

layout(location = 0) out vec3 vColor;
layout(location = 1) out float vAcross;

void main() {
  vec2 viewport = pc.color.xy;
  float halfPx = pc.color.z;

  // Project both endpoints; the offset must be perpendicular to the segment as
  // it appears on screen, so we work in pixel space after the perspective divide.
  vec4 clipP = pc.mvp * vec4(inPos, 1.0);
  vec4 clipQ = pc.mvp * vec4(inOther, 1.0);
  vec2 pxP = (clipP.xy / clipP.w) * 0.5 * viewport;
  vec2 pxQ = (clipQ.xy / clipQ.w) * 0.5 * viewport;

  // On-screen perpendicular. Guard a segment that projects to ~zero length.
  vec2 dir = pxQ - pxP;
  float len = length(dir);
  vec2 nrm = (len > 1e-4) ? vec2(-dir.y, dir.x) / len : vec2(0.0, 1.0);

  // Offset in pixels, converted back to clip space (multiply by w so it survives
  // the perspective divide the rasterizer applies to gl_Position).
  vec2 offsetPx = nrm * (inSide * halfPx);
  vec2 offsetNdc = offsetPx / (0.5 * viewport);
  clipP.xy += offsetNdc * clipP.w;

  gl_Position = clipP;
  vColor = inColor;
  vAcross = inAcross;
}
