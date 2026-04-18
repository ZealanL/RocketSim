use glam::{Vec3A, Vec4};

use crate::{bullet::linear_math::LARGE_FLOAT, shared::Aabb};

pub struct RayPacketInfo<'a> {
    pub ray_sources: &'a [Vec3A; 4],
    pub ray_targets: &'a [Vec3A; 4],
    pub lambda_max: Vec4,
    pub aabb: Aabb,
}

impl<'a> RayPacketInfo<'a> {
    pub fn new(ray_sources: &'a [Vec3A; 4], ray_targets: &'a [Vec3A; 4]) -> Self {
        // Union AABB of the 4 segments for quick root rejection.
        let mut aabb = Aabb::new(
            ray_sources[0].min(ray_targets[0]),
            ray_sources[0].max(ray_targets[0]),
        );
        for i in 1..4 {
            aabb += Aabb::new(
                ray_sources[i].min(ray_targets[i]),
                ray_sources[i].max(ray_targets[i]),
            );
        }

        Self {
            ray_sources,
            ray_targets,
            lambda_max: Vec4::ONE,
            aabb,
        }
    }

    pub fn calc_pos_dir(&self) -> ([Vec4; 3], [Vec4; 3]) {
        const LARGE_VEC: Vec4 = Vec4::splat(LARGE_FLOAT);

        let sources = [
            Vec4::from_array([
                self.ray_sources[0].x,
                self.ray_sources[1].x,
                self.ray_sources[2].x,
                self.ray_sources[3].x,
            ]),
            Vec4::from_array([
                self.ray_sources[0].y,
                self.ray_sources[1].y,
                self.ray_sources[2].y,
                self.ray_sources[3].y,
            ]),
            Vec4::from_array([
                self.ray_sources[0].z,
                self.ray_sources[1].z,
                self.ray_sources[2].z,
                self.ray_sources[3].z,
            ]),
        ];

        let targets = [
            Vec4::from_array([
                self.ray_targets[0].x,
                self.ray_targets[1].x,
                self.ray_targets[2].x,
                self.ray_targets[3].x,
            ]),
            Vec4::from_array([
                self.ray_targets[0].y,
                self.ray_targets[1].y,
                self.ray_targets[2].y,
                self.ray_targets[3].y,
            ]),
            Vec4::from_array([
                self.ray_targets[0].z,
                self.ray_targets[1].z,
                self.ray_targets[2].z,
                self.ray_targets[3].z,
            ]),
        ];

        let mut inv_dirs = [
            Vec4::ONE / (targets[0] - sources[0]),
            Vec4::ONE / (targets[1] - sources[1]),
            Vec4::ONE / (targets[2] - sources[2]),
        ];

        for inv_dir in &mut inv_dirs {
            *inv_dir = Vec4::select(inv_dir.is_finite_mask(), *inv_dir, LARGE_VEC);
        }

        (sources, inv_dirs)
    }

    /// Packet slab test for 4 rays using Vec4 lanes.
    ///
    /// Returns a bitmask (bit i set if ray i overlaps the AABB).
    /// Only the lowest 4 bits are set.
    pub fn intersect_ray_aabb_packet(
        origins: &[Vec4; 3],
        inv_dir: &[Vec4; 3],
        bounds: &Aabb,
        lambda_max: Vec4,
    ) -> u8 {
        let t0x = (bounds.min.x - origins[0]) * inv_dir[0];
        let t1x = (bounds.max.x - origins[0]) * inv_dir[0];
        let tminx = t0x.min(t1x);
        let tmaxx = t0x.max(t1x);

        let t0y = (bounds.min.y - origins[1]) * inv_dir[1];
        let t1y = (bounds.max.y - origins[1]) * inv_dir[1];
        let tminy = t0y.min(t1y);
        let tmaxy = t0y.max(t1y);

        let t0z = (bounds.min.z - origins[2]) * inv_dir[2];
        let t1z = (bounds.max.z - origins[2]) * inv_dir[2];
        let tminz = t0z.min(t1z);
        let tmaxz = t0z.max(t1z);

        let t_enter = tminx.max(tminy).max(tminz);
        let t_exit = tmaxx.min(tmaxy).min(tmaxz);

        let cond1 = t_enter.cmple(t_exit);
        let cond2 = t_exit.cmpge(Vec4::ZERO);
        let cond3 = t_enter.cmple(lambda_max);

        (cond1 & cond2 & cond3).bitmask() as u8
    }
}
