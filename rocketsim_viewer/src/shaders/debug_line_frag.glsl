#version 330 core
noperspective in vec2 fs_local;
noperspective in float fs_radius;
flat in float fs_half_len;
flat in vec4 fs_color;
uniform float u_alpha;
out vec4 out_color;
void main() {
    vec2 d=vec2(max(abs(fs_local.x)-fs_half_len,0.0),fs_local.y);
    float distance_to_edge=length(d)-fs_radius;
    float coverage=1.0-smoothstep(-0.75,0.75,distance_to_edge);
    float alpha=fs_color.a*u_alpha*coverage;
    if(alpha<=0.001 || fs_radius<=0.0) discard;
    out_color=vec4(fs_color.rgb,alpha);
}
