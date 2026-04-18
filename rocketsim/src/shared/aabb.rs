use std::ops::{Add, AddAssign};

use glam::{Affine3A, Vec3A};

#[derive(Clone, Copy, Debug, Default)]
pub struct Aabb {
    pub min: Vec3A,
    pub max: Vec3A,
}

impl Aabb {
    pub const ZERO: Self = Self {
        min: Vec3A::ZERO,
        max: Vec3A::ZERO,
    };

    #[inline]
    pub const fn new(min: Vec3A, max: Vec3A) -> Self {
        Self { min, max }
    }

    #[inline]
    pub fn center(&self) -> Vec3A {
        (self.min + self.max) * 0.5
    }

    pub fn area(&self) -> f32 {
        let extents = self.max - self.min;
        2.0 * (extents.x * extents.y + extents.x * extents.z + extents.y * extents.z)
    }

    #[inline]
    pub fn intersects(&self, rhs: &Self) -> bool {
        self.min.cmple(rhs.max).all() && self.max.cmpge(rhs.min).all()
    }

    pub fn combine(&self, rhs: &Self) -> Self {
        Self::new(self.min.min(rhs.min), self.max.max(rhs.max))
    }

    /// Finds the point inside the AABB nearest to the given point
    #[inline]
    pub fn clamp(&self, point: Vec3A) -> Vec3A {
        point.clamp(self.min, self.max)
    }

    pub(crate) fn from_half_extents_transform(
        half_extents: Vec3A,
        margin: f32,
        t: &Affine3A,
    ) -> Aabb {
        let half_extents_with_margin = half_extents + margin;
        let abs_b = t.matrix3.abs();
        let center = t.translation;
        let extent = abs_b * half_extents_with_margin;

        Aabb {
            min: center - extent,
            max: center + extent,
        }
    }

    pub(crate) fn transform(&self, trans: &Affine3A, margin: f32) -> Self {
        let local_half_extents = 0.5 * (self.max - self.min) + margin;

        let local_center = self.center();
        let abs_b = trans.matrix3.abs();
        let center = trans.transform_point3a(local_center);
        let extent = abs_b * local_half_extents;

        Aabb {
            min: center - extent,
            max: center + extent,
        }
    }
}

impl Add for Aabb {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            min: self.min.min(rhs.min),
            max: self.max.max(rhs.max),
        }
    }
}

impl AddAssign for Aabb {
    fn add_assign(&mut self, rhs: Self) {
        self.min = self.min.min(rhs.min);
        self.max = self.max.max(rhs.max);
    }
}
