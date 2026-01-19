#version 330 core

in vec3 fs_pos;
in vec2 fs_uv;
in vec3 fs_normal;

in vec3 fs_sky_dir;

uniform sampler2D u_texture;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;

out vec4 out_frag_color;

bool is_pos_in_goal(vec3 pos) {
    float EPSILON = 5.0;
    return (abs(pos.y) > (5120.0 + EPSILON)) && (abs(pos.y) < 6500) && (pos.z < 1000);
}

vec3 team_color_from_pos(vec3 pos) {
    vec3 BLUE = vec3(0.0, 0.3, 1.0);
    vec3 ORANGE = vec3(1.0, 0.3, 0.0);
    vec3 GRAY = vec3(0.5);
    float blue_scale = clamp(pos.y / 4000.0, 0, 1);
    float orange_scale = clamp(-pos.y / 4000.0, 0, 1); // TODO: Why are the signs flipped???

    if (!is_pos_in_goal(pos)) {
        // Not inside goal
        blue_scale *= 0.7;
        orange_scale *= 0.7;
    }

    float gray_scale = 1.0 - (blue_scale + orange_scale);
    vec3 team_color = BLUE * blue_scale + ORANGE * orange_scale + GRAY * gray_scale;
    return team_color;
}

float get_grid_line_val(float grid_size, float grid_line_width, vec3 pos, vec3 norm, vec3 pixel_widths) {
    vec3 grid_pixel_width = pixel_widths / grid_size;
    vec3 grid_pos = pos / grid_size;
    vec3 dist_to_line = abs(mod(grid_pos + 0.5, 1.0) - 0.5);
    float half_width = (grid_line_width / 2.0) / grid_size;
    vec3 line_frac3 = smoothstep(half_width + grid_pixel_width, half_width - grid_pixel_width, dist_to_line);

    // Mask with normal
    line_frac3 *= clamp(1.0 - abs(norm), 0.0, 1.0);

    return max(line_frac3.x, max(line_frac3.y, line_frac3.z));
}

void main() {
    vec3 pixel_widths = fwidth(fs_pos);

    float vertical_frac = abs(fs_normal.z);

    float grid_size = 1600;
    float grid_line_width = 10;

    if (is_pos_in_goal(fs_pos)) {
        grid_size = 100;
        grid_line_width = 10;
    } else if (fs_pos.z < 5) {
        grid_size /= 2; // Smaller grid on the ground
    }

    float grid_val = get_grid_line_val(grid_size, grid_line_width, fs_pos, fs_normal, pixel_widths);
    grid_val += get_grid_line_val(grid_size / 2, grid_line_width, fs_pos, fs_normal, pixel_widths) * 0.25;
    grid_val += get_grid_line_val(grid_size / 4, grid_line_width / 2, fs_pos, fs_normal, pixel_widths) * 0.125;

    grid_val = min(grid_val, 1);

    float min_grid_light = 0.05 + (abs(fs_normal.z) * 0.1);
    float grid_light = grid_val * (0.8 - min_grid_light) + min_grid_light;

    vec3 texture_color = texture(u_texture, fs_uv).xyz;
    float light_factor = (dot(fs_normal, normalize(vec3(0.3, -0.2, 0.6))) + 1) / 2;
    float brightness = (light_factor * 0.7) + 0.3;

    float VERTICAL_GRID_COLOR_SCALE = 0.3;
    float team_color_frac = grid_val * (((1.0 - vertical_frac) * VERTICAL_GRID_COLOR_SCALE) + (1.0 - VERTICAL_GRID_COLOR_SCALE));
    vec3 team_color = team_color_from_pos(fs_pos * team_color_frac);
    out_frag_color = vec4(grid_light * team_color * brightness, 1.0);
}