use glam::Vec3A;

const VERTA: usize = 0;
const VERTB: usize = 1;
const VERTC: usize = 2;

const DEGENERATE_TETRAHEDRON_EPS: f32 = 1.0e-4;

#[derive(Clone, Copy)]
pub struct UsageBitfield {
    pub used_vertex_a: bool,
    pub used_vertex_b: bool,
    pub used_vertex_c: bool,
    pub used_vertex_d: bool,
}

impl UsageBitfield {
    pub const fn new() -> Self {
        Self {
            used_vertex_a: false,
            used_vertex_b: false,
            used_vertex_c: false,
            used_vertex_d: false,
        }
    }

    pub const fn reset(&mut self) {
        self.used_vertex_a = false;
        self.used_vertex_b = false;
        self.used_vertex_c = false;
        self.used_vertex_d = false;
    }
}

pub struct SubSimplexClosestResult {
    pub closest_point_on_simplex: Vec3A,
    pub used_vertices: UsageBitfield,
    pub barycentric_coords: [f32; 4],
    pub degenerate: bool,
}

impl SubSimplexClosestResult {
    pub const fn new() -> Self {
        Self {
            closest_point_on_simplex: Vec3A::ZERO,
            used_vertices: UsageBitfield::new(),
            barycentric_coords: [0.0; 4],
            degenerate: false,
        }
    }

    pub const fn reset(&mut self) {
        self.degenerate = false;
        self.set_barycentric_coordinates(0.0, 0.0, 0.0, 0.0);
        self.used_vertices.reset();
    }

    pub fn is_valid(&self) -> bool {
        self.barycentric_coords[0] >= 0.0
            && self.barycentric_coords[1] >= 0.0
            && self.barycentric_coords[2] >= 0.0
            && self.barycentric_coords[3] >= 0.0
    }

    pub const fn set_barycentric_coordinates(&mut self, a: f32, b: f32, c: f32, d: f32) {
        self.barycentric_coords[0] = a;
        self.barycentric_coords[1] = b;
        self.barycentric_coords[2] = c;
        self.barycentric_coords[3] = d;
    }
}

pub struct VoronoiSimplexSolver {
    num_vertices: usize,

    simplex_vector_w: [Vec3A; Self::SIMPLEX_MAX_VERTS],
    simplex_points_p: [Vec3A; Self::SIMPLEX_MAX_VERTS],
    simplex_points_q: [Vec3A; Self::SIMPLEX_MAX_VERTS],

    cached_p1: Vec3A,
    cached_p2: Vec3A,
    cached_v: Vec3A,
    last_w: Vec3A,

    cached_valid_closest: bool,

    cached_bc: SubSimplexClosestResult,
    needs_update: bool,
}

impl VoronoiSimplexSolver {
    const SIMPLEX_MAX_VERTS: usize = 5;
    const EQUAL_VERTEX_THRESHOLD: f32 = 0.0001;

    pub const fn new() -> Self {
        Self {
            num_vertices: 0,
            simplex_vector_w: [Vec3A::ZERO; Self::SIMPLEX_MAX_VERTS],
            simplex_points_p: [Vec3A::ZERO; Self::SIMPLEX_MAX_VERTS],
            simplex_points_q: [Vec3A::ZERO; Self::SIMPLEX_MAX_VERTS],
            cached_p1: Vec3A::ZERO,
            cached_p2: Vec3A::ZERO,
            cached_v: Vec3A::ZERO,
            last_w: Vec3A::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            cached_valid_closest: false,
            cached_bc: SubSimplexClosestResult::new(),
            needs_update: true,
        }
    }

    pub const fn add_vertex(&mut self, w: Vec3A, p: Vec3A, q: Vec3A) {
        self.last_w = w;
        self.needs_update = true;

        self.simplex_vector_w[self.num_vertices] = w;
        self.simplex_points_p[self.num_vertices] = p;
        self.simplex_points_q[self.num_vertices] = q;

        self.num_vertices += 1;
    }

    pub fn closest(&mut self, v: &mut Vec3A) -> bool {
        let success = self.update_closest_vector_and_points();
        *v = self.cached_v;
        success
    }

    pub const fn full_simplex(&self) -> bool {
        self.num_vertices == 4
    }

    pub fn in_simplex(&self, w: Vec3A) -> bool {
        let mut found = false;
        for i in 0..self.num_vertices {
            let dist2 = (self.simplex_vector_w[i] - w).length_squared();
            if dist2 <= Self::EQUAL_VERTEX_THRESHOLD {
                found = true;
                break;
            }
        }

        if w == self.last_w {
            return true;
        }

        found
    }

    pub fn compute_points(&mut self) -> Vec3A {
        self.update_closest_vector_and_points();
        self.cached_p2
    }

    fn remove_vertex(&mut self, index: usize) {
        debug_assert!(self.num_vertices > 0);
        self.num_vertices -= 1;
        self.simplex_vector_w[index] = self.simplex_vector_w[self.num_vertices];
        self.simplex_points_p[index] = self.simplex_points_p[self.num_vertices];
        self.simplex_points_q[index] = self.simplex_points_q[self.num_vertices];
    }

    fn reduce_vertices(&mut self, used_verts: UsageBitfield) {
        if self.num_vertices >= 4 && !used_verts.used_vertex_d {
            self.remove_vertex(3);
        }

        if self.num_vertices >= 3 && !used_verts.used_vertex_c {
            self.remove_vertex(2);
        }

        if self.num_vertices >= 2 && !used_verts.used_vertex_b {
            self.remove_vertex(1);
        }

        if self.num_vertices >= 1 && !used_verts.used_vertex_a {
            self.remove_vertex(0);
        }
    }

    fn update_closest_vector_and_points(&mut self) -> bool {
        if !self.needs_update {
            return self.cached_valid_closest;
        }

        self.cached_bc.reset();
        self.needs_update = false;

        match self.num_vertices {
            1 => {
                self.cached_p1 = self.simplex_points_p[0];
                self.cached_p2 = self.simplex_points_q[0];
                self.cached_v = self.cached_p1 - self.cached_p2;
                self.cached_bc.reset();
                self.cached_bc
                    .set_barycentric_coordinates(1.0, 0.0, 0.0, 0.0);
                self.cached_valid_closest = self.cached_bc.is_valid();
            }
            2 => {
                let from = self.simplex_vector_w[0];
                let to = self.simplex_vector_w[1];

                let p = Vec3A::ZERO;
                let mut diff = p - from;
                let v = to - from;
                let mut t = v.dot(diff);

                if t > 0.0 {
                    let dot_vv = v.dot(v);
                    if t < dot_vv {
                        t /= dot_vv;
                        diff -= t * v;
                        self.cached_bc.used_vertices.used_vertex_a = true;
                    } else {
                        t = 1.0;
                        diff -= v;
                    }

                    self.cached_bc.used_vertices.used_vertex_b = true;
                } else {
                    t = 0.0;
                    self.cached_bc.used_vertices.used_vertex_a = true;
                }

                self.cached_bc
                    .set_barycentric_coordinates(1.0 - t, t, 0.0, 0.0);

                self.cached_p1 = self.simplex_points_p[0]
                    + t * (self.simplex_points_p[1] - self.simplex_points_p[0]);
                self.cached_p2 = self.simplex_points_q[0]
                    + t * (self.simplex_points_q[1] - self.simplex_points_q[0]);
                self.cached_v = self.cached_p1 - self.cached_p2;

                self.reduce_vertices(self.cached_bc.used_vertices);
                self.cached_valid_closest = self.cached_bc.is_valid();
            }
            3 => {
                let p = Vec3A::ZERO;

                let a = self.simplex_vector_w[0];
                let b = self.simplex_vector_w[1];
                let c = self.simplex_vector_w[2];

                Self::closest_pt_point_triangle(p, a, b, c, &mut self.cached_bc);

                self.cached_p1 = self.simplex_points_p[0] * self.cached_bc.barycentric_coords[0]
                    + self.simplex_points_p[1] * self.cached_bc.barycentric_coords[1]
                    + self.simplex_points_p[2] * self.cached_bc.barycentric_coords[2];

                self.cached_p2 = self.simplex_points_q[0] * self.cached_bc.barycentric_coords[0]
                    + self.simplex_points_q[1] * self.cached_bc.barycentric_coords[1]
                    + self.simplex_points_q[2] * self.cached_bc.barycentric_coords[2];

                self.cached_v = self.cached_p1 - self.cached_p2;

                self.reduce_vertices(self.cached_bc.used_vertices);
                self.cached_valid_closest = self.cached_bc.is_valid();
            }
            4 => {
                let p = Vec3A::ZERO;

                let a = self.simplex_vector_w[0];
                let b = self.simplex_vector_w[1];
                let c = self.simplex_vector_w[2];
                let d = self.simplex_vector_w[3];

                let has_separation =
                    Self::closest_pt_point_tetrahedron(p, a, b, c, d, &mut self.cached_bc);

                if !has_separation {
                    if self.cached_bc.degenerate {
                        self.cached_valid_closest = false;
                    } else {
                        self.cached_valid_closest = true;
                        self.cached_v = Vec3A::ZERO;
                    }

                    return self.cached_valid_closest;
                }

                self.cached_p1 = self.simplex_points_p[0] * self.cached_bc.barycentric_coords[0]
                    + self.simplex_points_p[1] * self.cached_bc.barycentric_coords[1]
                    + self.simplex_points_p[2] * self.cached_bc.barycentric_coords[2]
                    + self.simplex_points_p[3] * self.cached_bc.barycentric_coords[3];

                self.cached_p2 = self.simplex_points_q[0] * self.cached_bc.barycentric_coords[0]
                    + self.simplex_points_q[1] * self.cached_bc.barycentric_coords[1]
                    + self.simplex_points_q[2] * self.cached_bc.barycentric_coords[2]
                    + self.simplex_points_q[3] * self.cached_bc.barycentric_coords[3];

                self.cached_v = self.cached_p1 - self.cached_p2;
                self.reduce_vertices(self.cached_bc.used_vertices);

                self.cached_valid_closest = self.cached_bc.is_valid();
            }
            _ => {
                self.cached_valid_closest = false;
            }
        }

        self.cached_valid_closest
    }

    fn closest_pt_point_triangle(
        p: Vec3A,
        a: Vec3A,
        b: Vec3A,
        c: Vec3A,
        result: &mut SubSimplexClosestResult,
    ) -> bool {
        result.used_vertices.reset();

        let ab = b - a;
        let ac = c - a;
        let ap = p - a;
        let d1 = ab.dot(ap);
        let d2 = ac.dot(ap);
        if d1 <= 0.0 && d2 <= 0.0 {
            result.closest_point_on_simplex = a;
            result.used_vertices.used_vertex_a = true;
            result.set_barycentric_coordinates(1.0, 0.0, 0.0, 0.0);
            return true;
        }

        let bp = p - b;
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);
        if d3 >= 0.0 && d4 <= d3 {
            result.closest_point_on_simplex = b;
            result.used_vertices.used_vertex_b = true;
            result.set_barycentric_coordinates(0.0, 1.0, 0.0, 0.0);
            return true;
        }

        let vc = d1 * d4 - d3 * d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let v = d1 / (d1 - d3);
            result.closest_point_on_simplex = a + v * ab;
            result.used_vertices.used_vertex_a = true;
            result.used_vertices.used_vertex_b = true;
            result.set_barycentric_coordinates(1.0 - v, v, 0.0, 0.0);
            return true;
        }

        let cp = p - c;
        let d5 = ab.dot(cp);
        let d6 = ac.dot(cp);
        if d6 >= 0.0 && d5 <= d6 {
            result.closest_point_on_simplex = c;
            result.used_vertices.used_vertex_c = true;
            result.set_barycentric_coordinates(0.0, 0.0, 1.0, 0.0);
            return true;
        }

        let vb = d5 * d2 - d1 * d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let w = d2 / (d2 - d6);
            result.closest_point_on_simplex = a + w * ac;
            result.used_vertices.used_vertex_a = true;
            result.used_vertices.used_vertex_c = true;
            result.set_barycentric_coordinates(1.0 - w, 0.0, w, 0.0);
            return true;
        }

        let va = d3 * d6 - d5 * d4;
        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            result.closest_point_on_simplex = b + w * (c - b);
            result.used_vertices.used_vertex_b = true;
            result.used_vertices.used_vertex_c = true;
            result.set_barycentric_coordinates(0.0, 1.0 - w, w, 0.0);
            return true;
        }

        let denom = 1.0 / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;

        result.closest_point_on_simplex = a + ab * v + ac * w;
        result.used_vertices.used_vertex_a = true;
        result.used_vertices.used_vertex_b = true;
        result.used_vertices.used_vertex_c = true;
        result.set_barycentric_coordinates(1.0 - v - w, v, w, 0.0);

        true
    }

    fn point_outside_of_plane(p: Vec3A, a: Vec3A, b: Vec3A, c: Vec3A, d: Vec3A) -> Option<bool> {
        let normal = (b - a).cross(c - a);

        let signp = (p - a).dot(normal);
        let signd = (d - a).dot(normal);

        if signd * signd < DEGENERATE_TETRAHEDRON_EPS * DEGENERATE_TETRAHEDRON_EPS {
            return None;
        }

        Some(signp * signd < 0.0)
    }

    fn closest_pt_point_tetrahedron(
        p: Vec3A,
        a: Vec3A,
        b: Vec3A,
        c: Vec3A,
        d: Vec3A,
        final_result: &mut SubSimplexClosestResult,
    ) -> bool {
        final_result.closest_point_on_simplex = p;
        final_result.used_vertices.used_vertex_a = true;
        final_result.used_vertices.used_vertex_b = true;
        final_result.used_vertices.used_vertex_c = true;
        final_result.used_vertices.used_vertex_d = true;

        let Some(point_outside_abc) = Self::point_outside_of_plane(p, a, b, c, d) else {
            final_result.degenerate = true;
            return false;
        };

        let Some(point_outside_acd) = Self::point_outside_of_plane(p, a, c, d, b) else {
            final_result.degenerate = true;
            return false;
        };

        let Some(point_outside_adb) = Self::point_outside_of_plane(p, a, d, b, c) else {
            final_result.degenerate = true;
            return false;
        };

        let Some(point_outside_bdc) = Self::point_outside_of_plane(p, b, d, c, a) else {
            final_result.degenerate = true;
            return false;
        };

        final_result.degenerate = false;
        if !point_outside_abc && !point_outside_acd && !point_outside_adb && !point_outside_bdc {
            return false;
        }

        let mut best_sq_dist = f32::MAX;
        let mut temp_result = SubSimplexClosestResult::new();

        if point_outside_abc {
            Self::closest_pt_point_triangle(p, a, b, c, &mut temp_result);
            let q = temp_result.closest_point_on_simplex;
            let sq_dist = (q - p).length_squared();
            if sq_dist < best_sq_dist {
                best_sq_dist = sq_dist;
                final_result.closest_point_on_simplex = q;
                final_result.used_vertices.reset();
                final_result.used_vertices.used_vertex_a = temp_result.used_vertices.used_vertex_a;
                final_result.used_vertices.used_vertex_b = temp_result.used_vertices.used_vertex_b;
                final_result.used_vertices.used_vertex_c = temp_result.used_vertices.used_vertex_c;
                final_result.set_barycentric_coordinates(
                    temp_result.barycentric_coords[VERTA],
                    temp_result.barycentric_coords[VERTB],
                    temp_result.barycentric_coords[VERTC],
                    0.0,
                );
            }
        }

        if point_outside_acd {
            Self::closest_pt_point_triangle(p, a, c, d, &mut temp_result);
            let q = temp_result.closest_point_on_simplex;
            let sq_dist = (q - p).length_squared();
            if sq_dist < best_sq_dist {
                best_sq_dist = sq_dist;
                final_result.closest_point_on_simplex = q;
                final_result.used_vertices.reset();
                final_result.used_vertices.used_vertex_a = temp_result.used_vertices.used_vertex_a;
                final_result.used_vertices.used_vertex_c = temp_result.used_vertices.used_vertex_b;
                final_result.used_vertices.used_vertex_d = temp_result.used_vertices.used_vertex_c;
                final_result.set_barycentric_coordinates(
                    temp_result.barycentric_coords[VERTA],
                    0.0,
                    temp_result.barycentric_coords[VERTB],
                    temp_result.barycentric_coords[VERTC],
                );
            }
        }

        if point_outside_adb {
            Self::closest_pt_point_triangle(p, a, d, b, &mut temp_result);
            let q = temp_result.closest_point_on_simplex;
            let sq_dist = (q - p).length_squared();
            if sq_dist < best_sq_dist {
                best_sq_dist = sq_dist;
                final_result.closest_point_on_simplex = q;
                final_result.used_vertices.reset();
                final_result.used_vertices.used_vertex_a = temp_result.used_vertices.used_vertex_a;
                final_result.used_vertices.used_vertex_b = temp_result.used_vertices.used_vertex_c;
                final_result.used_vertices.used_vertex_d = temp_result.used_vertices.used_vertex_b;
                final_result.set_barycentric_coordinates(
                    temp_result.barycentric_coords[VERTA],
                    temp_result.barycentric_coords[VERTC],
                    0.0,
                    temp_result.barycentric_coords[VERTB],
                );
            }
        }

        if point_outside_bdc {
            Self::closest_pt_point_triangle(p, b, d, c, &mut temp_result);
            let q = temp_result.closest_point_on_simplex;
            let sq_dist = (q - p).length_squared();
            if sq_dist < best_sq_dist {
                // best_sq_dist = sq_dist;
                final_result.closest_point_on_simplex = q;
                final_result.used_vertices.reset();
                final_result.used_vertices.used_vertex_b = temp_result.used_vertices.used_vertex_a;
                final_result.used_vertices.used_vertex_c = temp_result.used_vertices.used_vertex_c;
                final_result.used_vertices.used_vertex_d = temp_result.used_vertices.used_vertex_b;
                final_result.set_barycentric_coordinates(
                    0.0,
                    temp_result.barycentric_coords[VERTA],
                    temp_result.barycentric_coords[VERTC],
                    temp_result.barycentric_coords[VERTB],
                );
            }
        }

        true
    }
}
