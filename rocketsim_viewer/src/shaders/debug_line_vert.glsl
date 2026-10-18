#version 330 core
layout(location=0) in vec3 vs_start;
layout(location=1) in vec3 vs_end;
layout(location=2) in vec4 vs_color;
layout(location=3) in float vs_width;
layout(location=4) in float vs_pixels;
layout(location=5) in float vs_point;
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;
uniform vec2 u_viewport;
noperspective out vec2 fs_local;
noperspective out float fs_radius;
flat out float fs_half_len;
flat out vec4 fs_color;
const vec2 QUAD[6]=vec2[6](vec2(0,-1),vec2(1,-1),vec2(0,1),vec2(0,1),vec2(1,-1),vec2(1,1));
void main() {
    vec2 corner=QUAD[gl_VertexID];
    vec4 a=u_proj*u_view*u_model*vec4(vs_start,1);
    vec4 b=u_proj*u_view*u_model*vec4(vs_end,1);
    float da=a.z+a.w, db=b.z+b.w;
    fs_color=vs_color;
    fs_local=vec2(0); fs_radius=0; fs_half_len=0;
    if (da<0 && db<0) { gl_Position=vec4(2,2,2,1); return; }
    // Clip before perspective division. A near-plane crossing must not expand behind the camera.
    if (da<0) a=mix(a,b,-da/(db-da));
    else if (db<0) b=mix(b,a,-db/(da-db));
    vec2 viewport=max(u_viewport,vec2(1));
    vec2 ap=a.xy/a.w*viewport*0.5, bp=b.xy/b.w*viewport*0.5;
    vec2 delta=bp-ap;
    float len=length(delta);
    bool point=vs_point>0.5 || len<0.0001;
    vec2 tangent=point?vec2(1,0):delta/max(len,0.0001);
    vec2 normal=vec2(-tangent.y,tangent.x);
    float w=mix(a.w,b.w,corner.x);
    float radius=vs_width*0.5;
    if (vs_pixels<0.5) {
        float scale=max(length(u_model[0].xyz),max(length(u_model[1].xyz),length(u_model[2].xyz)));
        radius*=scale*abs(u_proj[1][1])*viewport.y*0.5/max(w,0.0001);
    }
    radius=max(radius,0.0);
    float half_len=point?0.0:len*0.5;
    vec2 local=vec2((corner.x*2.0-1.0)*(half_len+radius+1.0),corner.y*(radius+1.0));
    vec2 center=point?ap:(ap+bp)*0.5;
    vec2 screen=center+tangent*local.x+normal*local.y;
    float z=mix(a.z/a.w,b.z/b.w,point?0.0:corner.x);
    gl_Position=vec4(screen*(2.0/viewport)*w,z*w,w);
    fs_local=local; fs_radius=radius; fs_half_len=half_len;
}
