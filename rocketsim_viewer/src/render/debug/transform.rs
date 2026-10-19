use rocketsim::ArenaState;
use rocketsim_vis::debug::{Anchor, DebugGroup, MeshInstanceItem};

/// Resolve an anchor from the displayed snapshot. Return `None` when its target is missing.
pub fn resolve_group_matrix(group: &DebugGroup, state: Option<&ArenaState>) -> Option<glam::Mat4> {
    Some(anchor_matrix(&group.anchor, state)? * group_matrix(group))
}

pub(super) fn instance_matrix(instance: &MeshInstanceItem) -> glam::Mat4 {
    glam::Mat4::from_scale_rotation_translation(
        glam::Vec3::from(instance.scale),
        glam::Quat::from_mat3a(&rotation(&instance.rotation)),
        glam::Vec3::from(instance.position),
    )
}

fn group_matrix(group: &DebugGroup) -> glam::Mat4 {
    glam::Mat4::from_scale_rotation_translation(
        glam::Vec3::from(group.transform.scale),
        glam::Quat::from_mat3a(&rotation(&group.transform.rotation)),
        glam::Vec3::from(group.transform.position),
    )
}

fn anchor_matrix(anchor: &Anchor, state: Option<&ArenaState>) -> Option<glam::Mat4> {
    let (position, rotation) = match anchor {
        Anchor::World => (glam::Vec3A::ZERO, glam::Mat3A::IDENTITY),
        Anchor::Ball { .. } => {
            let state = state?;
            (state.ball.phys.pos, state.ball.phys.rot_mat)
        }
        Anchor::Car { car_id, .. } => {
            let state = state?;
            let (_, car) = state
                .cars
                .iter()
                .find(|(info, _)| info.idx as u32 == *car_id)?;
            (car.phys.pos, car.phys.rot_mat)
        }
    };
    let offset = match anchor {
        Anchor::World => glam::Vec3A::ZERO,
        Anchor::Ball { offset } | Anchor::Car { offset, .. } => glam::Vec3A::from_array(*offset),
    };
    Some(glam::Mat4::from_rotation_translation(
        glam::Quat::from_mat3a(&rotation),
        (position + rotation * offset).into(),
    ))
}

fn rotation(rotation: &[[f32; 3]; 3]) -> glam::Mat3A {
    glam::Mat3A::from_cols_array_2d(rotation).transpose()
}

#[cfg(test)]
mod tests {
    use rocketsim_vis::debug::{Color, DebugGroup, DepthMode};

    use super::*;

    #[test]
    fn transform_scenarios_compose_local_group_and_anchor_transforms() {
        let instance = MeshInstanceItem {
            mesh: "mesh".into(),
            position: [2.0, 0.0, 0.0],
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            scale: [1.0; 3],
            tint: Color::WHITE,
            texture: None,
            depth: DepthMode::Test,
        };
        let mut group = DebugGroup::new("g", Default::default(), Default::default()).unwrap();
        group.transform.position = [0.0, 3.0, 0.0];
        group.transform.scale = [2.0; 3];
        let world = resolve_group_matrix(&group, None).unwrap();
        let local_then_group =
            (world * instance_matrix(&instance)).transform_point3(glam::Vec3::ZERO);
        assert_eq!(local_then_group, glam::Vec3::new(4.0, 3.0, 0.0));

        group.anchor = Anchor::car(123456);
        assert!(resolve_group_matrix(&group, None).is_none());

        let mut state = ArenaState::new_empty(rocketsim::GameMode::TheVoid);
        state.ball.phys.pos = glam::Vec3A::new(10.0, 20.0, 30.0);
        state.ball.phys.rot_mat = glam::Mat3A::from_rotation_z(std::f32::consts::FRAC_PI_2);
        group.anchor = Anchor::Ball {
            offset: [2.0, 0.0, 0.0],
        };
        let anchored = resolve_group_matrix(&group, Some(&state)).unwrap();
        let point = anchored.transform_point3(glam::Vec3::ZERO);
        assert!(point.abs_diff_eq(glam::Vec3::new(7.0, 22.0, 30.0), 1e-5));
    }
}
