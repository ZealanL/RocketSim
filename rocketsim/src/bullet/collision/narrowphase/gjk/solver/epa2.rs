use glam::{Mat3A, Vec3A};

use super::{Gjk, GjkSimplex};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EpaStatus {
    Valid,
    Degenerated,
    NonConvex,
    InvalidHull,
    OutOfFaces,
    OutOfVertices,
    AccuracyReached,
    FallBack,
}

#[derive(Clone, Copy, Debug)]
pub struct EpaVertex {
    pub w: Vec3A,
    pub d: Vec3A,
    pub support_a: Vec3A,
    pub support_b: Vec3A,
}

impl EpaVertex {
    pub const fn new() -> Self {
        Self {
            w: Vec3A::ZERO,
            d: Vec3A::ZERO,
            support_a: Vec3A::ZERO,
            support_b: Vec3A::ZERO,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct EpaFace {
    pub n: Vec3A,
    pub d: f32,
    pub indices: [usize; 3],
    pub adj: [Option<usize>; 3],
    pub edge: [u8; 3],
    pub prev: Option<usize>,
    pub next: Option<usize>,
    pub pass: u8,
}

impl EpaFace {
    pub const fn new() -> Self {
        Self {
            n: Vec3A::ZERO,
            d: 0.0,
            indices: [0; 3],
            adj: [None; 3],
            edge: [0; 3],
            prev: None,
            next: None,
            pass: 0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct EpaList {
    pub root: Option<usize>,
    pub count: usize,
}

impl EpaList {
    pub const fn new() -> Self {
        Self {
            root: None,
            count: 0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct EpaHorizon {
    cf: Option<usize>,
    ff: Option<usize>,
    nf: usize,
}

impl EpaHorizon {
    const fn new() -> Self {
        Self {
            cf: None,
            ff: None,
            nf: 0,
        }
    }
}

pub struct Epa2 {
    pub result: GjkSimplex,
    pub normal: Vec3A,
    pub depth: f32,
    pub sv_store: [EpaVertex; Self::MAX_VERTICES],
    fc_store: [EpaFace; Self::MAX_FACES],
    next_sv: usize,
    hull: EpaList,
    stock: EpaList,
}

impl Epa2 {
    const MAX_VERTICES: usize = 128;
    const MAX_ITERATIONS: u8 = 255;

    const ACCURACY: f32 = 1.0e-4;
    const PLANE_EPS: f32 = 1.0e-5;

    const MAX_FACES: usize = Self::MAX_VERTICES * 2;

    pub const fn new() -> Self {
        let mut stock = EpaList::new();
        let mut fc_store = [EpaFace::new(); Self::MAX_FACES];

        let mut i = fc_store.len();
        while i > 0 {
            i -= 1;
            Self::append(&mut stock, &mut fc_store, i);
        }

        Self {
            result: GjkSimplex::new(),
            normal: Vec3A::ZERO,
            depth: 0.0,
            sv_store: [EpaVertex::new(); Self::MAX_VERTICES],
            fc_store,
            next_sv: 0,
            hull: EpaList::new(),
            stock,
        }
    }

    const fn append(list: &mut EpaList, faces: &mut [EpaFace; Self::MAX_FACES], face_idx: usize) {
        let face = &mut faces[face_idx];
        face.prev = None;
        face.next = list.root;
        if let Some(root) = list.root {
            faces[root].prev = Some(face_idx);
        }
        list.root = Some(face_idx);
        list.count += 1;
    }

    fn remove(list: &mut EpaList, faces: &mut [EpaFace; Self::MAX_FACES], face_idx: usize) {
        let (prev, next) = {
            let face = &faces[face_idx];
            (face.prev, face.next)
        };

        if let Some(next) = next {
            faces[next].prev = prev;
        }
        if let Some(prev) = prev {
            faces[prev].next = next;
        }
        if list.root == Some(face_idx) {
            list.root = next;
        }

        let face = &mut faces[face_idx];
        face.prev = None;
        face.next = None;
        list.count -= 1;
    }

    const fn bind(&mut self, fa: usize, ea: usize, fb: usize, eb: usize) {
        self.fc_store[fa].edge[ea] = eb as u8;
        self.fc_store[fa].adj[ea] = Some(fb);
        self.fc_store[fb].edge[eb] = ea as u8;
        self.fc_store[fb].adj[eb] = Some(fa);
    }

    fn det(a: Vec3A, b: Vec3A, c: Vec3A) -> f32 {
        Mat3A::from_cols(a, b, c).determinant()
    }

    fn get_edge_dist(&self, face_n: Vec3A, a_idx: usize, b_idx: usize, dist: &mut f32) -> bool {
        let a = self.sv_store[a_idx];
        let b = self.sv_store[b_idx];

        let ba = b.w - a.w;
        let n_ab = ba.cross(face_n);
        let a_dot_nab = a.w.dot(n_ab);

        if a_dot_nab < 0.0 {
            let ba_l2 = ba.length_squared();
            let a_dot_ba = a.w.dot(ba);
            let b_dot_ba = b.w.dot(ba);

            if a_dot_ba > 0.0 {
                *dist = a.w.length();
            } else if b_dot_ba < 0.0 {
                *dist = b.w.length();
            } else {
                let a_dot_b = a.w.dot(b.w);
                let value =
                    (a.w.length_squared() * b.w.length_squared() - a_dot_b * a_dot_b) / ba_l2;
                *dist = value.max(0.0).sqrt();
            }

            return true;
        }

        false
    }

    fn new_face(
        &mut self,
        a_idx: usize,
        b_idx: usize,
        c_idx: usize,
        status: &mut EpaStatus,
    ) -> Option<usize> {
        let Some(face_idx) = self.stock.root else {
            *status = EpaStatus::OutOfFaces;
            return None;
        };

        Self::remove(&mut self.stock, &mut self.fc_store, face_idx);
        Self::append(&mut self.hull, &mut self.fc_store, face_idx);

        let a = self.sv_store[a_idx];
        let b = self.sv_store[b_idx];
        let c = self.sv_store[c_idx];

        let n = (b.w - a.w).cross(c.w - a.w);
        let l = n.length();
        if l > Self::ACCURACY {
            let mut dist = 0.0;
            if !(self.get_edge_dist(n, a_idx, b_idx, &mut dist)
                || self.get_edge_dist(n, b_idx, c_idx, &mut dist)
                || self.get_edge_dist(n, c_idx, a_idx, &mut dist))
            {
                dist = a.w.dot(n) / l;
            }

            let face = &mut self.fc_store[face_idx];
            face.pass = 0;
            face.indices = [a_idx, b_idx, c_idx];
            face.adj = [None, None, None];
            face.edge = [0, 0, 0];
            face.n = n / l;
            face.d = dist;

            if face.d >= -Self::PLANE_EPS {
                return Some(face_idx);
            }

            *status = EpaStatus::NonConvex;
        } else {
            *status = EpaStatus::Degenerated;
        }

        Self::remove(&mut self.hull, &mut self.fc_store, face_idx);
        Self::append(&mut self.stock, &mut self.fc_store, face_idx);
        None
    }

    fn new_face_forced(&mut self, a_idx: usize, b_idx: usize, c_idx: usize) -> usize {
        let face_idx = self.stock.root.unwrap();

        Self::remove(&mut self.stock, &mut self.fc_store, face_idx);
        Self::append(&mut self.hull, &mut self.fc_store, face_idx);

        let a = self.sv_store[a_idx];
        let b = self.sv_store[b_idx];
        let c = self.sv_store[c_idx];

        let n = (b.w - a.w).cross(c.w - a.w);
        let l = n.length();
        debug_assert!(l > Self::ACCURACY);

        let mut dist = 0.0;
        if !(self.get_edge_dist(n, a_idx, b_idx, &mut dist)
            || self.get_edge_dist(n, b_idx, c_idx, &mut dist)
            || self.get_edge_dist(n, c_idx, a_idx, &mut dist))
        {
            dist = a.w.dot(n) / l;
        }

        let face = &mut self.fc_store[face_idx];
        face.pass = 0;
        face.indices = [a_idx, b_idx, c_idx];
        face.adj = [None, None, None];
        face.edge = [0, 0, 0];
        face.n = n / l;
        face.d = dist;

        face_idx
    }

    fn find_best(&self) -> usize {
        let mut minf = self.hull.root.unwrap();
        let mut mind = self.fc_store[minf].d * self.fc_store[minf].d;
        let mut current = self.fc_store[minf].next;

        while let Some(idx) = current {
            let sqd = self.fc_store[idx].d * self.fc_store[idx].d;
            if sqd < mind {
                minf = idx;
                mind = sqd;
            }
            current = self.fc_store[idx].next;
        }

        minf
    }

    fn expand(
        &mut self,
        pass: u8,
        w_index: usize,
        face_idx: usize,
        edge: usize,
        horizon: &mut EpaHorizon,
        status: &mut EpaStatus,
    ) -> bool {
        const I1M3: [usize; 3] = [1, 2, 0];
        const I2M3: [usize; 3] = [2, 0, 1];

        let face = self.fc_store[face_idx];
        if face.pass == pass {
            return false;
        }

        let e1 = I1M3[edge];
        let w = self.sv_store[w_index].w;
        if (face.n.dot(w) - face.d) < -Self::PLANE_EPS {
            if let Some(nf) = self.new_face(face.indices[e1], face.indices[edge], w_index, status) {
                self.bind(nf, 0, face_idx, edge);
                if let Some(cf) = horizon.cf {
                    self.bind(cf, 1, nf, 2);
                } else {
                    horizon.ff = Some(nf);
                }
                horizon.cf = Some(nf);
                horizon.nf += 1;
                return true;
            }
        } else {
            let e2 = I2M3[edge];
            self.fc_store[face_idx].pass = pass;

            let adj1 = self.fc_store[face_idx].adj[e1];
            let adj2 = self.fc_store[face_idx].adj[e2];
            let edge1 = self.fc_store[face_idx].edge[e1] as usize;
            let edge2 = self.fc_store[face_idx].edge[e2] as usize;

            if let (Some(f1), Some(f2)) = (adj1, adj2) {
                if self.expand(pass, w_index, f1, edge1, horizon, status)
                    && self.expand(pass, w_index, f2, edge2, horizon, status)
                {
                    Self::remove(&mut self.hull, &mut self.fc_store, face_idx);
                    Self::append(&mut self.stock, &mut self.fc_store, face_idx);
                    return true;
                }
            } else {
                *status = EpaStatus::InvalidHull;
            }
        }

        false
    }

    pub fn evaluate<const ENABLE_MARGIN: bool>(&mut self, mut gjk: Gjk, guess: Vec3A) -> EpaStatus {
        if gjk.simplex().rank <= 1 || !gjk.enclose_origin::<ENABLE_MARGIN>() {
            self.normal = -guess;
            let nl = self.normal.length();
            if nl > 0.0 {
                self.normal /= nl;
            } else {
                self.normal = Vec3A::X;
            }

            self.depth = 0.0;
            self.result.rank = 1;
            self.result.c[0] = gjk.simplex().c[0];
            self.result.p[0] = 1.0;

            return EpaStatus::FallBack;
        }

        let mut status = EpaStatus::Valid;
        let mut sv_indices = [0usize; 4];

        for (i, [w, d]) in gjk.simplex_w_d().enumerate() {
            let sv_index = self.next_sv;
            self.next_sv += 1;

            self.sv_store[sv_index].w = w;
            self.sv_store[sv_index].d = d;
            self.sv_store[sv_index].support_a = Vec3A::ZERO;
            self.sv_store[sv_index].support_b = Vec3A::ZERO;

            sv_indices[i] = sv_index;
        }

        let w0 = self.sv_store[sv_indices[0]].w - self.sv_store[sv_indices[3]].w;
        let w1 = self.sv_store[sv_indices[1]].w - self.sv_store[sv_indices[3]].w;
        let w2 = self.sv_store[sv_indices[2]].w - self.sv_store[sv_indices[3]].w;

        if Self::det(w0, w1, w2) < 0.0 {
            sv_indices.swap(0, 1);
        }

        let tetra = [
            self.new_face_forced(sv_indices[0], sv_indices[1], sv_indices[2]),
            self.new_face_forced(sv_indices[1], sv_indices[0], sv_indices[3]),
            self.new_face_forced(sv_indices[2], sv_indices[1], sv_indices[3]),
            self.new_face_forced(sv_indices[0], sv_indices[2], sv_indices[3]),
        ];

        self.bind(tetra[0], 0, tetra[1], 0);
        self.bind(tetra[0], 1, tetra[2], 0);
        self.bind(tetra[0], 2, tetra[3], 0);
        self.bind(tetra[1], 1, tetra[3], 2);
        self.bind(tetra[1], 2, tetra[2], 1);
        self.bind(tetra[2], 2, tetra[3], 1);

        let mut best = self.find_best();
        let mut outer = self.fc_store[best];

        // Bullet numbers EPA passes from 1: new faces are stamped with
        // pass 0 at creation, so starting at 0 would make every adjacent
        // face look already-visited and abort expansion on the first pass.
        'epa_iter: for pass in 1..=Self::MAX_ITERATIONS {
            if self.next_sv >= Self::MAX_VERTICES {
                status = EpaStatus::OutOfVertices;
                break;
            }

            let w_index = self.next_sv;
            self.next_sv += 1;

            self.fc_store[best].pass = pass;

            let mut dir = self.fc_store[best].n;
            let dir_len = dir.length();
            if dir_len > 0.0 {
                dir /= dir_len;
            } else {
                dir = Vec3A::X;
            }

            let w = gjk.shape.support::<ENABLE_MARGIN>(dir);
            self.sv_store[w_index].w = w;
            self.sv_store[w_index].d = dir;
            self.sv_store[w_index].support_a = Vec3A::ZERO;
            self.sv_store[w_index].support_b = Vec3A::ZERO;

            let wdist = dir.dot(w) - self.fc_store[best].d;
            if wdist <= Self::ACCURACY {
                status = EpaStatus::AccuracyReached;
                break;
            }

            let mut horizon = EpaHorizon::new();
            for j in 0..3 {
                let adj = self.fc_store[best].adj[j].unwrap();
                let edge = self.fc_store[best].edge[j] as usize;
                if !self.expand(pass, w_index, adj, edge, &mut horizon, &mut status) {
                    break 'epa_iter;
                }
            }

            if horizon.nf < 3 {
                status = EpaStatus::InvalidHull;
                break;
            }

            self.bind(horizon.cf.unwrap(), 1, horizon.ff.unwrap(), 2);
            Self::remove(&mut self.hull, &mut self.fc_store, best);
            Self::append(&mut self.stock, &mut self.fc_store, best);

            best = self.find_best();
            outer = self.fc_store[best];
        }

        let projection = outer.n * outer.d;

        self.normal = outer.n;
        self.depth = outer.d;
        self.result.rank = 3;

        let a_idx = outer.indices[0];
        let b_idx = outer.indices[1];
        let c_idx = outer.indices[2];

        self.result.c[0] = a_idx;
        self.result.c[1] = b_idx;
        self.result.c[2] = c_idx;

        self.result.p[0] = (self.sv_store[b_idx].w - projection)
            .cross(self.sv_store[c_idx].w - projection)
            .length();
        self.result.p[1] = (self.sv_store[c_idx].w - projection)
            .cross(self.sv_store[a_idx].w - projection)
            .length();
        self.result.p[2] = (self.sv_store[a_idx].w - projection)
            .cross(self.sv_store[b_idx].w - projection)
            .length();

        let sum = self.result.p[0] + self.result.p[1] + self.result.p[2];
        if sum > 0.0 {
            self.result.p[0] /= sum;
            self.result.p[1] /= sum;
            self.result.p[2] /= sum;
        }

        status
    }
}
