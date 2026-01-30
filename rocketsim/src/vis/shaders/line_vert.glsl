#version 330 core

in vec3 vs_vert;
in vec3 vs_line_pos_a;
in vec3 vs_line_pos_b;
in vec4 vs_line_color_a;
in vec4 vs_line_color_b;
in float vs_line_width_a;
in float vs_line_width_b;

out vec3 fs_pos;
out vec3 fs_line_pos_a;
out vec3 fs_line_pos_b;
out vec4 fs_line_color;
out float fs_line_width;

uniform mat4 u_proj;
uniform mat4 u_view;
uniform vec2 u_screen_size;

void main() {
    fs_line_pos_a = vs_line_pos_a;
    fs_line_pos_b = vs_line_pos_b;

    vec3 camera_pos = inverse(u_view)[3].xyz;

    vec3 line_dir = normalize(vs_line_pos_b - vs_line_pos_a);

    float side = vs_vert.x;
    float along = vs_vert.y;
    float line_frac = (along + 1.0) / 2; // Map from [-1, 1] to [0, 1]
    vec3 axis_pos = mix(vs_line_pos_a, vs_line_pos_b, line_frac);

    // Calculate minimal world width
    vec4 view_pos = u_view * vec4(axis_pos, 1.0);
    float dist = abs(view_pos.z); // Distance from camera plane
    float min_world_width = (2.0 * dist) / (u_proj[1][1] * u_screen_size.y);

    float clamped_line_width = max(
        mix(vs_line_width_a, vs_line_width_b, line_frac), min_world_width
    );

    vec3 view_dir = normalize(axis_pos - camera_pos);

    vec3 world_perp_dir = cross(line_dir, view_dir);
    if (length(world_perp_dir) < 1e-3) {
        // Fix rare edge-case
        world_perp_dir = vec3(0, 0, 1);
    } else {
        world_perp_dir = normalize(world_perp_dir);
    }

    // Add additional padding for the end-caps if this is a 0-length line
    vec3 pad_end_cap = line_dir * along * clamped_line_width;

    // NOTE: Wider than it would normally be by the "min_world_width", for antialiasing
    vec3 world_pos = axis_pos + pad_end_cap + world_perp_dir * (side * (clamped_line_width + min_world_width) * 0.5);

    gl_Position = (u_proj * u_view * vec4(world_pos, 1.0));
    fs_line_width = clamped_line_width;
    fs_pos = world_pos;
    fs_line_color = mix(vs_line_color_a, vs_line_color_b, line_frac);
}