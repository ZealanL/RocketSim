#version 330 core

in vec3 fs_pos;
in vec3 fs_pa;
in vec3 fs_pb;
in vec4 fs_color;
in float fs_width;
flat in float fs_ribbon;

out vec4 out_frag_color;

// Ref: https://stackoverflow.com/a/56467661
float point_segment_dist(vec3 p, vec3 a, vec3 b) {
   vec3 ab = b - a;
   vec3 ap = p - a;
   float t = clamp(dot(ap, ab) / dot(ab, ab), 0.0, 1.0);
   return distance(p, a + t * ab);
}

void main() {
   // Ribbon quads share edges. Internal round caps would overlap at each join.
   if (fs_ribbon > 0.5) {
      out_frag_color = fs_color;
      return;
   }
   // Cap support
   // TODO: Needs antialiasing
   float dist_to_line = point_segment_dist(fs_pos, fs_pa, fs_pb);
   if (dist_to_line < fs_width / 2) {
      out_frag_color = fs_color;
   } else {
      discard;
   }
}
