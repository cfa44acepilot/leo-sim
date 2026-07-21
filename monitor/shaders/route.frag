#version 450
// Route fragment: a solid bright core with a faint soft outer edge (cheap glow /
// anti-alias). vAcross is -1..+1 across the band; keep the middle fully opaque
// and fade only the outer sliver so the thick line reads crisp, not stair-stepped.

layout(location = 0) in vec3 vColor;
layout(location = 1) in float vAcross;
layout(location = 0) out vec4 outColor;

void main() {
  float edge = 1.0 - smoothstep(0.65, 1.0, abs(vAcross));
  outColor = vec4(vColor, edge);
}
