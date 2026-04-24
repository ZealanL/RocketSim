use arrayvec::ArrayVec;
use glam::{Mat3A, Vec3A};

use super::MinkowskiDiff;

#[derive(Clone, Copy)]
pub struct GjkSupport {
    pub d: Vec3A,
    pub w: Vec3A,
}

impl GjkSupport {
    const fn new() -> Self {
        Self {
            d: Vec3A::ZERO,
            w: Vec3A::ZERO,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct GjkSimplex {
    pub rank: usize,
    pub c: [usize; 4],
    pub p: [f32; 4],
}

impl GjkSimplex {
    pub const fn new() -> Self {
        Self {
            rank: 0,
            c: [0; 4],
            p: [0.0; 4],
        }
    }
}

#[derive(PartialEq, Eq)]
pub enum GjkStatus {
    Valid,
    Inside,
    Failed,
}

pub struct Gjk<'a> {
    pub shape: &'a MinkowskiDiff<'a>,
    ray: Vec3A,
    distance: f32,
    simplices: [GjkSimplex; 2],
    pub store: [GjkSupport; 4],
    free: ArrayVec<usize, 4>,
    current: usize,
    simplex: usize,
}

impl<'a> Gjk<'a> {
    const MAX_ITERATIONS: usize = 128;

    const ACCURACY: f32 = 1.0e-4;
    const MIN_DISTANCE: f32 = 1.0e-4;
    const DUPLICATED_EPS: f32 = 1.0e-4;

    const SIMPLEX2_EPS: f32 = 0.0;
    const SIMPLEX3_EPS: f32 = 0.0;
    const SIMPLEX4_EPS: f32 = 0.0;

    pub const fn new(shape: &'a MinkowskiDiff<'a>) -> Self {
        Self {
            shape,
            ray: Vec3A::ZERO,
            distance: 0.0,
            simplices: [GjkSimplex::new(); 2],
            store: [GjkSupport::new(); 4],
            free: ArrayVec::new_const(),
            current: 0,
            simplex: 0,
        }
    }

    pub fn simplex(&self) -> GjkSimplex {
        self.simplices[self.simplex]
    }

    pub fn evaluate<const ENABLE_MARGIN: bool>(&mut self, guess: Vec3A) -> GjkStatus {
        let mut status = GjkStatus::Valid;
        let mut iterations = 0usize;
        let mut alpha = 0.0f32;
        let mut clastw = 0usize;

        self.free.extend([0, 1, 2, 3]);
        self.ray = guess;

        let mut sqdist = self.ray.length_squared();
        let dir = if sqdist > 0.0 { -self.ray } else { Vec3A::X };
        self.append_vertex::<ENABLE_MARGIN>(0, dir);
        self.simplices[0].p[0] = 1.0;
        let first_index = self.simplices[0].c[0];
        self.ray = self.store[first_index].w;

        let mut lastw = [self.ray; 4];

        while status == GjkStatus::Valid {
            let rl = self.ray.length();
            if rl < Self::MIN_DISTANCE {
                status = GjkStatus::Inside;
                break;
            }

            let current = self.current;
            let next = 1 - current;

            self.append_vertex::<ENABLE_MARGIN>(current, -self.ray);
            let cs = self.simplices[current];
            let w = self.store[cs.c[cs.rank - 1]].w;

            let mut found = false;
            for lastw in lastw {
                if (w - lastw).length_squared() < Self::DUPLICATED_EPS {
                    found = true;
                    break;
                }
            }

            if found {
                self.remove_vertex(current);
                break;
            } else {
                clastw = (clastw + 1) & 3;
                lastw[clastw] = w;
            }

            let omega = self.ray.dot(w) / rl;
            if omega > alpha {
                alpha = omega;
            }

            if ((rl - alpha) - (Self::ACCURACY * rl)) <= 0.0 {
                self.remove_vertex(current);
                break;
            }

            let mut weights = [0.0f32; 4];
            let mut mask: u8 = 0;

            sqdist = match cs.rank {
                2 => Self::project_origin_2(
                    self.store[cs.c[0]].w,
                    self.store[cs.c[1]].w,
                    &mut weights,
                    &mut mask,
                ),
                3 => Self::project_origin_3(
                    self.store[cs.c[0]].w,
                    self.store[cs.c[1]].w,
                    self.store[cs.c[2]].w,
                    &mut weights,
                    &mut mask,
                ),
                4 => Self::project_origin_4(
                    self.store[cs.c[0]].w,
                    self.store[cs.c[1]].w,
                    self.store[cs.c[2]].w,
                    self.store[cs.c[3]].w,
                    &mut weights,
                    &mut mask,
                ),
                _ => -1.0,
            };

            if sqdist >= 0.0 {
                let mut ns = GjkSimplex::new();
                self.ray = Vec3A::ZERO;
                for (i, weight) in weights[0..cs.rank].iter().copied().enumerate() {
                    if (mask & (1 << i)) != 0 {
                        ns.c[ns.rank] = cs.c[i];
                        ns.p[ns.rank] = weight;
                        self.ray += self.store[cs.c[i]].w * weight;
                        ns.rank += 1;
                    } else {
                        self.free.push(cs.c[i]);
                    }
                }

                if mask == 15 {
                    status = GjkStatus::Inside;
                }

                self.simplices[next] = ns;
                self.current = next;
            } else {
                self.remove_vertex(current);
                break;
            }

            iterations += 1;
            if iterations >= Self::MAX_ITERATIONS {
                status = GjkStatus::Failed;
            }
        }

        self.simplex = self.current;
        match status {
            GjkStatus::Valid => {
                self.distance = self.ray.length();
            }
            GjkStatus::Inside => {
                self.distance = 0.0;
            }
            GjkStatus::Failed => {}
        }

        status
    }

    pub fn enclose_origin<const ENABLE_MARGIN: bool>(&mut self) -> bool {
        let simplex_index = self.simplex;
        match self.simplices[simplex_index].rank {
            1 => {
                for i in 0..3 {
                    let mut axis = Vec3A::ZERO;
                    axis[i] = 1.0;

                    self.append_vertex::<ENABLE_MARGIN>(simplex_index, axis);
                    if self.enclose_origin::<ENABLE_MARGIN>() {
                        return true;
                    }

                    self.remove_vertex(simplex_index);
                    self.append_vertex::<ENABLE_MARGIN>(simplex_index, -axis);
                    if self.enclose_origin::<ENABLE_MARGIN>() {
                        return true;
                    }

                    self.remove_vertex(simplex_index);
                }
            }
            2 => {
                let c0 = self.simplices[simplex_index].c[0];
                let c1 = self.simplices[simplex_index].c[1];
                let d = self.store[c1].w - self.store[c0].w;
                for i in 0..3 {
                    let mut axis = Vec3A::ZERO;
                    match i {
                        0 => axis.x = 1.0,
                        1 => axis.y = 1.0,
                        _ => axis.z = 1.0,
                    }
                    let p = d.cross(axis);
                    if p.length_squared() > 0.0 {
                        self.append_vertex::<ENABLE_MARGIN>(simplex_index, p);
                        if self.enclose_origin::<ENABLE_MARGIN>() {
                            return true;
                        }

                        self.remove_vertex(simplex_index);
                        self.append_vertex::<ENABLE_MARGIN>(simplex_index, -p);
                        if self.enclose_origin::<ENABLE_MARGIN>() {
                            return true;
                        }

                        self.remove_vertex(simplex_index);
                    }
                }
            }
            3 => {
                let c0 = self.simplices[simplex_index].c[0];
                let c1 = self.simplices[simplex_index].c[1];
                let c2 = self.simplices[simplex_index].c[2];
                let n = (self.store[c1].w - self.store[c0].w)
                    .cross(self.store[c2].w - self.store[c0].w);
                if n.length_squared() > 0.0 {
                    self.append_vertex::<ENABLE_MARGIN>(simplex_index, n);
                    if self.enclose_origin::<ENABLE_MARGIN>() {
                        return true;
                    }

                    self.remove_vertex(simplex_index);
                    self.append_vertex::<ENABLE_MARGIN>(simplex_index, -n);
                    if self.enclose_origin::<ENABLE_MARGIN>() {
                        return true;
                    }

                    self.remove_vertex(simplex_index);
                }
            }
            4 => {
                let c0 = self.simplices[simplex_index].c[0];
                let c1 = self.simplices[simplex_index].c[1];
                let c2 = self.simplices[simplex_index].c[2];
                let c3 = self.simplices[simplex_index].c[3];

                let det = Self::det(
                    self.store[c0].w - self.store[c3].w,
                    self.store[c1].w - self.store[c3].w,
                    self.store[c2].w - self.store[c3].w,
                );

                if det.abs() > 0.0 {
                    return true;
                }
            }
            _ => {}
        }
        false
    }

    fn getsupport<const ENABLE_MAGIN: bool>(&mut self, d: Vec3A, sv_index: usize) {
        let dir = d / d.length();
        self.store[sv_index].d = dir;
        self.store[sv_index].w = self.shape.support::<ENABLE_MAGIN>(dir);
    }

    fn remove_vertex(&mut self, simplex_index: usize) {
        let simplex = &mut self.simplices[simplex_index];
        simplex.rank -= 1;
        self.free.push(simplex.c[simplex.rank]);
    }

    fn append_vertex<const ENABLE_MAGIN: bool>(&mut self, simplex_index: usize, v: Vec3A) {
        let rank = self.simplices[simplex_index].rank;
        self.simplices[simplex_index].p[rank] = 0.0;
        let sv_index = self.free.pop().unwrap();
        self.getsupport::<ENABLE_MAGIN>(v, sv_index);
        self.simplices[simplex_index].c[rank] = sv_index;
        self.simplices[simplex_index].rank = rank + 1;
    }

    fn det(a: Vec3A, b: Vec3A, c: Vec3A) -> f32 {
        Mat3A::from_cols(a, b, c).determinant()
    }

    fn project_origin_2(a: Vec3A, b: Vec3A, w: &mut [f32; 4], m: &mut u8) -> f32 {
        let d = b - a;
        let l = d.length_squared();
        if l > Self::SIMPLEX2_EPS {
            let t = if l > 0.0 { -a.dot(d) / l } else { 0.0 };
            if t >= 1.0 {
                w[0] = 0.0;
                w[1] = 1.0;
                *m = 2;
                b.length_squared()
            } else if t <= 0.0 {
                w[0] = 1.0;
                w[1] = 0.0;
                *m = 1;
                a.length_squared()
            } else {
                w[1] = t;
                w[0] = 1.0 - t;
                *m = 3;
                (a + d * t).length_squared()
            }
        } else {
            -1.0
        }
    }

    fn project_origin_3(a: Vec3A, b: Vec3A, c: Vec3A, w: &mut [f32; 4], m: &mut u8) -> f32 {
        const IMD3: [usize; 3] = [1, 2, 0];
        let vt = [a, b, c];
        let dl = [a - b, b - c, c - a];
        let n = dl[0].cross(dl[1]);
        let l = n.length_squared();
        if l > Self::SIMPLEX3_EPS {
            let mut mindist = -1.0f32;
            let mut subw = [0.0f32; 4];
            let mut subm: u8 = 0;
            for i in 0..3 {
                if vt[i].dot(dl[i].cross(n)) > 0.0 {
                    let j = IMD3[i];
                    let subd = Self::project_origin_2(vt[i], vt[j], &mut subw, &mut subm);
                    if (mindist < 0.0) || (subd < mindist) {
                        mindist = subd;
                        *m = ((if (subm & 1) != 0 { 1 << i } else { 0 })
                            + (if (subm & 2) != 0 { 1 << j } else { 0 }))
                            as u8;
                        w[i] = subw[0];
                        w[j] = subw[1];
                        w[IMD3[j]] = 0.0;
                    }
                }
            }
            if mindist < 0.0 {
                let d = a.dot(n);
                let s = l.sqrt();
                let p = n * (d / l);
                mindist = p.length_squared();
                *m = 7;
                w[0] = (dl[1].cross(b - p)).length() / s;
                w[1] = (dl[2].cross(c - p)).length() / s;
                w[2] = 1.0 - (w[0] + w[1]);
            }
            mindist
        } else {
            -1.0
        }
    }

    fn project_origin_4(
        a: Vec3A,
        b: Vec3A,
        c: Vec3A,
        d: Vec3A,
        w: &mut [f32; 4],
        m: &mut u8,
    ) -> f32 {
        const IMD3: [usize; 3] = [1, 2, 0];
        let vt = [a, b, c, d];
        let dl = [a - d, b - d, c - d];
        let vl = Self::det(dl[0], dl[1], dl[2]);
        let ng = (vl * a.dot((b - c).cross(a - b))) <= 0.0;
        if ng && vl.abs() > Self::SIMPLEX4_EPS {
            let mut mindist = -1.0f32;
            let mut subw = [0.0f32; 4];
            let mut subm: u8 = 0;
            for i in 0..3 {
                let j = IMD3[i];
                let s = vl * d.dot(dl[i].cross(dl[j]));
                if s > 0.0 {
                    let subd = Self::project_origin_3(vt[i], vt[j], d, &mut subw, &mut subm);
                    if (mindist < 0.0) || (subd < mindist) {
                        mindist = subd;
                        *m = (if (subm & 1) != 0 { 1 << i } else { 0 })
                            + (if (subm & 2) != 0 { 1 << j } else { 0 })
                            + (if (subm & 4) != 0 { 8 } else { 0 });
                        w[i] = subw[0];
                        w[j] = subw[1];
                        w[IMD3[j]] = 0.0;
                        w[3] = subw[2];
                    }
                }
            }
            if mindist < 0.0 {
                mindist = 0.0;
                *m = 15;
                w[0] = Self::det(c, b, d) / vl;
                w[1] = Self::det(a, c, d) / vl;
                w[2] = Self::det(b, a, d) / vl;
                w[3] = 1.0 - (w[0] + w[1] + w[2]);
            }
            mindist
        } else {
            -1.0
        }
    }
}
