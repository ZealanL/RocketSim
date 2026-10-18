#version 330 core
in vec3 v_normal;
in vec4 v_color;
in vec2 v_uv;
uniform sampler2D u_texture;
uniform bool u_textured;
uniform float u_alpha;
out vec4 out_color;
void main(){ vec4 c=v_color; if(u_textured)c*=texture(u_texture,v_uv); vec3 n=gl_FrontFacing?normalize(v_normal):-normalize(v_normal); float light=0.32+0.68*max(dot(normalize(vec3(0.2,0.4,1.0)),n),0.0); out_color=vec4(c.rgb*light,c.a*u_alpha); }
