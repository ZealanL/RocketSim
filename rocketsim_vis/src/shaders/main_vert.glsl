#version 330 core

// Naming scheme I came up with:
// - Everything is snake_case
// - Vertex/fragment shader input variables prefixed with "vs_"/"fs_" respectively
// - Uniforms prefixed with "u_"

layout (location = 0) in vec3 vs_vert;
layout (location = 1) in vec3 vs_vert_normal;
layout (location = 2) in vec2 vs_vert_uv;

out vec2 fs_uv;
out vec3 fs_normal;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;

void main() {
    fs_uv = vs_vert_uv;
    fs_normal = normalize(mat3(u_model) * vs_vert_normal);
    gl_Position = u_proj * u_view * u_model * vec4(vs_vert, 1.0);
}
