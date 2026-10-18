#version 330 core
layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_normal;
layout(location=2) in vec4 a_color;
layout(location=3) in vec2 a_uv;
layout(location=4) in vec4 i_m0;
layout(location=5) in vec4 i_m1;
layout(location=6) in vec4 i_m2;
layout(location=7) in vec4 i_m3;
layout(location=8) in vec4 i_tint;
uniform mat4 u_group;
uniform mat4 u_view;
uniform mat4 u_proj;
out vec3 v_normal;
out vec4 v_color;
out vec2 v_uv;
void main() {
    mat4 world=u_group*mat4(i_m0,i_m1,i_m2,i_m3);
    mat3 basis=mat3(world);
    vec3 n=abs(determinant(basis))>0.000001 ? transpose(inverse(basis))*a_normal : basis*a_normal;
    v_normal=dot(n,n)>0.000001 ? normalize(n) : vec3(0,0,1);
    v_color=a_color*i_tint;
    v_uv=a_uv;
    gl_Position=u_proj*u_view*world*vec4(a_pos,1);
}
