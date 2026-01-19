#version 330 core

in vec2 fs_uv;
in vec3 fs_normal;

uniform sampler2D u_texture;

out vec4 out_frag_color;

void main() {
    vec3 light_ambient_color = vec3(0.7, 0.85, 1.0);
    vec3 light_sun_color = vec3(1.0, 0.85, 0.7);
    vec3 light_dir = normalize(vec3(0.1, 0.2, 1.0));
    float light_align = max(dot(light_dir, fs_normal), 0);
    vec3 light_color = light_ambient_color*0.3 + (light_sun_color * light_align);

    vec4 texture_color = texture(u_texture, fs_uv);

    if (texture_color.w >= 1.0) {
        out_frag_color = texture_color * vec4(light_color, 1.0);
    } else {
        // Partially-transparent textures are unaffected by lighting
        out_frag_color = texture_color;
    }
}