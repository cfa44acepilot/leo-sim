#version 450
// Simple hemispheric Lambert shade for the Earth/atmosphere shells. Alpha comes
// from the push constant so the same shader serves the opaque Earth (a = 1) and
// the translucent atmosphere (a < 1, drawn with blending by a separate pipeline
// state).

layout(location = 0) in vec3 vNormal;

layout(push_constant) uniform PC {
  mat4 mvp;
  vec4 color;
} pc;

layout(location = 0) out vec4 outColor;

void main() {
  vec3 N = normalize(vNormal);
  vec3 L = normalize(vec3(0.6, 0.7, 0.5));  // fixed key light
  float lambert = max(dot(N, L), 0.0) * 0.8 + 0.2;  // 0.2 ambient floor
  outColor = vec4(pc.color.rgb * lambert, pc.color.a);
}
