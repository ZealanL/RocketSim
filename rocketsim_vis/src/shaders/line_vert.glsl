#version 330 core

// CPU-expanded quad corners (see lines::expand_segment): the GPU only
// transforms and varies them through. Attribute locations are pinned in
// the shader, so the VAO binds them by index.
layout (location = 0) in vec3 vs_world;
layout (location = 1) in vec3 vs_pa;
layout (location = 2) in vec3 vs_pb;
layout (location = 3) in vec4 vs_color;
layout (location = 4) in float vs_width;

out vec3 fs_pos;
out vec3 fs_pa;
out vec3 fs_pb;
out vec4 fs_color;
out float fs_width;

uniform mat4 u_view;
uniform mat4 u_proj;

void main() {
    gl_Position = u_proj * u_view * vec4(vs_world, 1.0);
    fs_pos = vs_world;
    fs_pa = vs_pa;
    fs_pb = vs_pb;
    fs_color = vs_color;
    fs_width = vs_width;
}
