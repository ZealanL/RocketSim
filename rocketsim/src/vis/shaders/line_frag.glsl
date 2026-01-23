#version 330 core

in vec3 fs_pos;
in vec3 fs_line_pos_a;
in vec3 fs_line_pos_b;
in vec4 fs_line_color;
in float fs_line_width;

uniform mat4 u_proj;
uniform mat4 u_view;

out vec4 out_frag_color;

// Ref: https://stackoverflow.com/a/56467661
float point_segment_dist(vec3 p, vec3 a, vec3 b) {
   vec3 ab = b - a;
   vec3 ap = p - a;
   float t = clamp(dot(ap, ab) / dot(ab, ab), 0.0, 1.0);
   return distance(p, a + t * ab);
}

void main() {
   if (fs_line_pos_a == fs_line_pos_b) {
      // Cap support
      // TODO: Needs antialiasing
      float dist_to_line = point_segment_dist(fs_pos, fs_line_pos_a, fs_line_pos_b);
      if (dist_to_line < fs_line_width / 2) {
         out_frag_color = fs_line_color;
      } else {
         out_frag_color = vec4(fs_line_color.rgb, 0.0);
      }
   } else {
      out_frag_color = fs_line_color;
   }
}