use glam::{Vec3, Vec3A};

use crate::shared::Aabb;

pub struct ContactInfo {
    pub result_normal: Vec3A,
    pub contact_point: Vec3A,
    pub depth: f32,
}

fn segment_sqr_distance(from: Vec3A, to: Vec3A, p: Vec3A, nearest: &mut Vec3A) -> f32 {
    let mut diff = p - from;
    let v = to - from;
    let mut t = v.dot(diff);

    if t > 0. {
        let dot_vv = v.dot(v);
        if t < dot_vv {
            t /= dot_vv;
            diff -= t * v;
        } else {
            t = 1.;
            diff -= v;
        }
    } else {
        t = 0.;
    }

    *nearest = from + t * v;
    diff.dot(diff)
}

/// A triangle made from 3 points.
#[derive(Clone, Copy, Debug, Default)]
pub struct TriangleShape {
    pub points: [Vec3A; 3],

    pub normal: Vec3A,
}

impl TriangleShape {
    pub fn edge(&self, index: usize) -> Vec3A {
        match index {
            0 => self.points[1] - self.points[0],
            1 => self.points[2] - self.points[1],
            2 => self.points[0] - self.points[2],
            _ => unreachable!(),
        }
    }

    #[inline]
    pub fn aabb(&self) -> Aabb {
        Aabb {
            min: self.points[0].min(self.points[1]).min(self.points[2]),
            max: self.points[0].max(self.points[1]).max(self.points[2]),
        }
    }

    /// Create a new triangle from 3 points.
    pub fn new(points: [Vec3A; 3]) -> Self {
        let edges = [
            points[1] - points[0],
            points[2] - points[1],
            points[0] - points[2],
        ];

        let normal = edges[0].cross(-edges[2]).normalize();

        Self { points, normal }
    }

    #[inline]
    pub fn normal_length(&self) -> f32 {
        let edge_0 = self.points[1] - self.points[0];
        let edge_2 = self.points[0] - self.points[2];
        edge_0.cross(-edge_2).length()
    }

    #[inline]
    /// Create a new triangle from an iterator that must be of 3 points
    pub fn from_points_iter(mut iter: impl Iterator<Item = Vec3A>) -> Self {
        Self::new([
            iter.next().unwrap(),
            iter.next().unwrap(),
            iter.next().unwrap(),
        ])
    }

    /// Check if a point projected onto the same place as the triangle
    /// is within the bounds of it.
    pub fn face_contains(&self, n: Vec3A, obj_to_points: &[Vec3A; 3]) -> bool {
        let c0 = self.edge(0).cross(obj_to_points[0]);
        if c0.dot(n) < 0. {
            return false;
        }

        let c1 = self.edge(1).cross(obj_to_points[1]);
        if c1.dot(n) < 0. {
            return false;
        }

        let c2 = self.edge(2).cross(obj_to_points[2]);
        c2.dot(n) >= 0.
    }

    /// Check if a sphere intersects the triangle.
    pub fn intersect_sphere(
        &self,
        obj_center: Vec3A,
        radius: f32,
        threshold: f32,
    ) -> Option<ContactInfo> {
        let mut triangle_normal = self.normal;
        let obj_to_center = obj_center - self.points[0];
        let mut distance_from_plane = obj_to_center.dot(triangle_normal);

        if distance_from_plane < 0. {
            distance_from_plane *= -1.0;
            triangle_normal *= -1.0;
        }

        let radius_with_threshold = radius + threshold;
        let radius_with_threshold_sqr = radius_with_threshold * radius_with_threshold;
        self.intersect_sphere_from_plane(
            obj_center,
            obj_to_center,
            triangle_normal,
            distance_from_plane,
            radius,
            radius_with_threshold,
            radius_with_threshold_sqr,
        )
    }

    /// Front-side-only variant used by the concave sphere callback. The
    /// caller discards back-facing contacts, so reject them before the edge
    /// and closest-feature work.
    #[inline]
    pub fn intersect_sphere_front(
        &self,
        obj_center: Vec3A,
        radius: f32,
        threshold: f32,
    ) -> Option<ContactInfo> {
        let radius_with_threshold = radius + threshold;
        let radius_with_threshold_sqr = radius_with_threshold * radius_with_threshold;
        self.intersect_sphere_front_precomputed(
            obj_center,
            radius,
            radius_with_threshold,
            radius_with_threshold_sqr,
        )
    }

    #[inline]
    pub fn intersect_sphere_front_precomputed(
        &self,
        obj_center: Vec3A,
        radius: f32,
        radius_with_threshold: f32,
        radius_with_threshold_sqr: f32,
    ) -> Option<ContactInfo> {
        let obj_to_center = obj_center - self.points[0];
        let distance_from_plane = obj_to_center.dot(self.normal);
        if distance_from_plane < 0. {
            return None;
        }

        self.intersect_sphere_from_plane(
            obj_center,
            obj_to_center,
            self.normal,
            distance_from_plane,
            radius,
            radius_with_threshold,
            radius_with_threshold_sqr,
        )
    }

    #[inline]
    #[allow(clippy::too_many_arguments)]
    fn intersect_sphere_from_plane(
        &self,
        obj_center: Vec3A,
        obj_to_center: Vec3A,
        triangle_normal: Vec3A,
        distance_from_plane: f32,
        radius: f32,
        radius_with_threshold: f32,
        radius_with_threshold_sqr: f32,
    ) -> Option<ContactInfo> {
        if distance_from_plane >= radius_with_threshold {
            return None;
        }

        let obj_to_points = [
            obj_to_center,
            obj_center - self.points[1],
            obj_center - self.points[2],
        ];

        let contact_point = if self.face_contains(triangle_normal, &obj_to_points) {
            obj_center - triangle_normal * distance_from_plane
        } else {
            let contact_capsule_radius_sqr = radius_with_threshold_sqr;
            let mut min_distance_sqr = contact_capsule_radius_sqr;
            let mut contact_point = Vec3A::ZERO;

            for edge_idx in 0..3 {
                let (from, to) = match edge_idx {
                    0 => (self.points[0], self.points[1]),
                    1 => (self.points[1], self.points[2]),
                    2 => (self.points[2], self.points[0]),
                    _ => unreachable!(),
                };
                let mut nearest_on_edge = Vec3A::ZERO;
                let distance_sqr = segment_sqr_distance(from, to, obj_center, &mut nearest_on_edge);
                if distance_sqr < min_distance_sqr {
                    min_distance_sqr = distance_sqr;
                    contact_point = nearest_on_edge;
                }
            }

            if min_distance_sqr < contact_capsule_radius_sqr {
                contact_point
            } else {
                return None;
            }
        };

        let contact_to_center = obj_center - contact_point;
        let distance_sqr = contact_to_center.length_squared();

        if distance_sqr >= radius_with_threshold_sqr {
            return None;
        }

        let (result_normal, depth) = if distance_sqr > f32::EPSILON {
            let distance = distance_sqr.sqrt();
            let inverse_distance = 1.0 / distance;
            (contact_to_center * inverse_distance, -(radius - distance))
        } else {
            (triangle_normal, -radius)
        };

        Some(ContactInfo {
            result_normal,
            contact_point,
            depth,
        })
    }

    pub fn local_get_supporting_vertex_without_margin(&self, vec: Vec3A) -> Vec3A {
        let dots = Vec3::new(
            vec.dot(self.points[0]),
            vec.dot(self.points[1]),
            vec.dot(self.points[2]),
        );

        self.points[dots.max_position()]
    }

    #[inline]
    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        self.local_get_supporting_vertex_without_margin(vec)
    }
}
