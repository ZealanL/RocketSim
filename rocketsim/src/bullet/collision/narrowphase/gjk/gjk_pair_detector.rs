use glam::{Affine3A, Vec3A};

use super::{
    GjkResult, closest_point_input::ClosestPointInput, penetration::calc_pen_depth,
    solver::VoronoiSimplexSolver,
};
use crate::bullet::collision::shapes::collision_shape::CollisionShapes;

const MAX_ITERATIONS: usize = 1000;
const REL_ERROR2: f32 = 1.0e-6;

struct SimplexContactResult {
    point_on_b: Vec3A,
    normal_in_b: Vec3A,
    distance: f32,
    is_valid: bool,
    degenerate_simplex: u8,
}

struct GjkIterationResult {
    check_simplex: bool,
    degenerate_simplex: u8,
    squared_distance: f32,
}

pub struct GjkPairDetector {
    margin: f32,
    margin_b: f32,
    separating_axis: Vec3A,
    separating_distance: f32,
    simplex_solver: VoronoiSimplexSolver,
}

impl GjkPairDetector {
    pub const fn new(margin_a: f32, margin_b: f32) -> Self {
        Self {
            margin: margin_a + margin_b,
            margin_b,
            separating_axis: Vec3A::Y,
            separating_distance: 0.0,
            simplex_solver: VoronoiSimplexSolver::new(),
        }
    }

    pub fn get_closest_points<T: GjkResult>(
        mut self,
        input: &ClosestPointInput,
        shape_a: &CollisionShapes,
        shape_b: &CollisionShapes,
        output: &mut T,
    ) {
        let mut local_trans_a = *input.transform_a;
        let mut local_trans_b = *input.transform_b;
        let position_offset = (local_trans_a.translation + local_trans_b.translation) * 0.5;
        local_trans_a.translation -= position_offset;
        local_trans_b.translation -= position_offset;

        let GjkIterationResult {
            check_simplex,
            mut degenerate_simplex,
            squared_distance,
        } = self.update_separating_axis_and_simplex(
            &local_trans_a,
            &local_trans_b,
            shape_a,
            shape_b,
            input.maximum_distance_squared,
        );

        let mut is_valid = false;
        let mut distance = 0.0;
        let mut point_on_b = Vec3A::ZERO;
        let mut normal_in_b = Vec3A::ZERO;

        if check_simplex {
            SimplexContactResult {
                point_on_b,
                normal_in_b,
                distance,
                is_valid,
                degenerate_simplex,
            } = self.compute_simplex_contact(squared_distance, degenerate_simplex);
        }

        let catch_degenerate_penetration_case =
            degenerate_simplex != 0 && (distance + self.margin) < 0.01;

        if (!is_valid || catch_degenerate_penetration_case)
            && let Some(result) = calc_pen_depth(shape_a, shape_b, &local_trans_a, &local_trans_b)
        {
            let [tmp_point_on_a, tmp_point_on_b] = result.witnesses;

            if result.penetrating {
                let tmp_normal_in_b = tmp_point_on_b - tmp_point_on_a;
                let len_sqr = tmp_normal_in_b.length_squared();
                if len_sqr > f32::EPSILON * f32::EPSILON {
                    let length = len_sqr.sqrt();
                    let distance_2 = -length;

                    // only replace valid penetrations when the result is deeper
                    if !is_valid || distance_2 < distance {
                        distance = distance_2;
                        point_on_b = tmp_point_on_b;
                        normal_in_b = tmp_normal_in_b / length;
                        is_valid = true;
                    }
                }
            } else {
                let distance_2 = (tmp_point_on_a - tmp_point_on_b).length() - self.margin;
                if !is_valid || distance_2 < distance {
                    distance = distance_2;
                    point_on_b = tmp_point_on_b + result.normal * self.margin_b;
                    normal_in_b = result.normal.normalize();
                    is_valid = true;
                }
            }
        }

        // Bullet emits every penetrating contact and gates only separated
        // ones by the maximum distance.
        if is_valid && (distance < 0.0 || distance * distance < input.maximum_distance_squared) {
            self.separating_axis = normal_in_b;
            self.separating_distance = distance;

            output.add_contact_point(normal_in_b, point_on_b + position_offset, distance);
        }
    }

    fn compute_simplex_contact(
        &mut self,
        squared_distance: f32,
        degenerate_simplex: u8,
    ) -> SimplexContactResult {
        let mut result = SimplexContactResult {
            point_on_b: Vec3A::ZERO,
            normal_in_b: Vec3A::ZERO,
            distance: 0.0,
            is_valid: false,
            degenerate_simplex,
        };

        result.point_on_b = self.simplex_solver.compute_points();
        result.normal_in_b = self.separating_axis;

        // valid normal
        let len_sqr = self.separating_axis.length_squared();
        if len_sqr < 0.0001 {
            result.degenerate_simplex = 5;
        }

        if len_sqr > f32::EPSILON * f32::EPSILON {
            let rlen = 1.0 / len_sqr.sqrt();
            result.normal_in_b *= rlen;

            let s = squared_distance.sqrt();
            result.point_on_b += self.separating_axis * (self.margin_b / s);
            result.distance = (1.0 / rlen) - self.margin;
            result.is_valid = true;
        }

        result
    }

    fn update_separating_axis_and_simplex(
        &mut self,
        transform_a: &Affine3A,
        transform_b: &Affine3A,
        shape_a: &CollisionShapes,
        shape_b: &CollisionShapes,
        maximum_distance_squared: f32,
    ) -> GjkIterationResult {
        let mut check_simplex = false;
        let mut degenerate_simplex = 0;
        let mut squared_distance = f32::MAX;

        for _ in 0..MAX_ITERATIONS {
            let separating_axis_in_a = transform_a
                .matrix3
                .mul_transpose_vec3a(-self.separating_axis);
            let separating_axis_in_b = transform_b
                .matrix3
                .mul_transpose_vec3a(self.separating_axis);

            let p_in_a = shape_a.local_get_support_vertex_without_margin(separating_axis_in_a);
            let p_world = transform_a.transform_point3a(p_in_a);

            let q_in_b = shape_b.local_get_support_vertex_without_margin(separating_axis_in_b);
            let q_world = transform_b.transform_point3a(q_in_b);

            let w = p_world - q_world;
            let delta = self.separating_axis.dot(w);

            // Potential exit: the shapes are separated far enough.
            if delta > 0.0 && delta * delta > squared_distance * maximum_distance_squared {
                degenerate_simplex = 10;
                check_simplex = true;
                break;
            }

            // Exit: new point already exists in the simplex.
            if self.simplex_solver.in_simplex(w) {
                degenerate_simplex = 1;
                check_simplex = true;
                break;
            }

            // Are we getting any closer?
            let f0 = squared_distance - delta;
            let f1 = squared_distance * REL_ERROR2;
            if f0 <= f1 {
                degenerate_simplex = if f0.is_sign_negative() { 2 } else { 11 };
                check_simplex = true;
                break;
            }

            // Add current vertex to simplex.
            self.simplex_solver.add_vertex(w, p_world, q_world);

            let mut new_cached_separating_axis = Vec3A::ZERO;

            // Calculate the closest point to the origin.
            if !self.simplex_solver.closest(&mut new_cached_separating_axis) {
                degenerate_simplex = 3;
                check_simplex = true;
                break;
            }

            if new_cached_separating_axis.length_squared() < REL_ERROR2 {
                self.separating_axis = new_cached_separating_axis;
                degenerate_simplex = 6;
                check_simplex = true;
                break;
            }

            let previous_squared_distance = squared_distance;
            squared_distance = new_cached_separating_axis.length_squared();

            // Are we getting any closer?
            if previous_squared_distance - squared_distance
                <= f32::EPSILON * previous_squared_distance
            {
                check_simplex = true;
                degenerate_simplex = 12;
                break;
            }

            self.separating_axis = new_cached_separating_axis;

            if self.simplex_solver.full_simplex() {
                degenerate_simplex = 13;
                break;
            }
        }

        GjkIterationResult {
            check_simplex,
            degenerate_simplex,
            squared_distance,
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec3A;

    use super::*;
    use crate::bullet::collision::shapes::{
        box_shape::BoxShape, compound_shape::CompoundShape, triangle_shape::TriangleShape,
    };

    struct CollectContact {
        pub point: Option<Vec3A>,
        pub normal: Option<Vec3A>,
        pub depth: Option<f32>,
    }

    impl GjkResult for CollectContact {
        fn add_contact_point(&mut self, normal_on_b: Vec3A, point_on_b_world: Vec3A, depth: f32) {
            self.point = Some(point_on_b_world);
            self.normal = Some(normal_on_b);
            self.depth = Some(depth);
        }
    }

    /// Grazing box-vs-triangle penetration far from the origin.
    /// The Voronoi GJK run ends with a near-zero separating axis, so the
    /// EPA penetration fallback supplies the contact. Its witnesses must be
    /// world-space points on the shapes. A double transform by trans_a used
    /// to push them one full box translation away from the pair.
    #[test]
    fn box_triangle_graze_emits_witness_near_box() {
        let box_shape = BoxShape::new(Vec3A::new(1.0, 2.0, 0.5));
        let margin_a = box_shape.get_margin();
        let compound = CompoundShape::new(box_shape, Affine3A::IDENTITY);
        let shape_a = CollisionShapes::Compound(compound);

        // Box parked far from the origin, like a car body near the goal.
        let trans_a = Affine3A::from_translation(Vec3A::new(0.0, 119.0, 4.66).into());
        let trans_b = Affine3A::IDENTITY;

        // Wide mesh-style triangle dipping slightly into the box top face.
        // Box top (without margin) is z = 4.66 + 0.5 = 5.16.
        let triangle = TriangleShape::new([
            Vec3A::new(-5.12, 118.9, 5.14),
            Vec3A::new(5.12, 118.9, 5.14),
            Vec3A::new(0.0, 120.0, 5.1),
        ]);
        let shape_b = CollisionShapes::Triangle(triangle);

        let input = ClosestPointInput::new(&trans_a, &trans_b, margin_a + 0.05);
        let detector = GjkPairDetector::new(margin_a, 0.0);
        let mut out = CollectContact {
            point: None,
            normal: None,
            depth: None,
        };
        detector.get_closest_points(&input, &shape_a, &shape_b, &mut out);

        let point = out.point.expect("grazing pair must emit a contact");
        let normal = out.normal.expect("grazing pair must emit a normal");
        let depth = out.depth.expect("grazing pair must emit a depth");
        assert!(depth < 0.0, "grazing pair must penetrate, got {depth}");
        assert!(
            (normal.length() - 1.0).abs() < 1e-5,
            "normal {normal:?} not unit"
        );

        // The witness must sit on the pair, not a full translation away.
        let box_center = trans_a.translation;
        let witness_gap = (point - box_center).length();
        assert!(
            witness_gap < 5.0,
            "witness {point:?} is {witness_gap} from box center {box_center:?}"
        );

        // Target adapter identity worldA = point + normal * depth: the
        // reconstructed A point must land back on the box top face, and the
        // emitted depth must equal the witness separation magnitude.
        let world_a = point + normal * depth;
        assert!(
            (world_a - box_center).length() < 5.0,
            "reconstructed A {world_a:?} left the pair"
        );
        assert!(
            (world_a.z - 5.16).abs() < 0.15,
            "reconstructed A {world_a:?} not on the box top face"
        );
        assert!(
            ((point - world_a).length() + depth).abs() < 1e-3,
            "depth {depth} != -|B - A|"
        );
    }
}
