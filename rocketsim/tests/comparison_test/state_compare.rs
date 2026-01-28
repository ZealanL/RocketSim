use glam::{Mat3A, Vec3A};
use rocketsim::{BallState, CarState, PhysState};
use rocketsim_rs::consts;

fn calc_vec3_err(a: Vec3A, b: Vec3A) -> f32 {
    a.distance(b)
}

/// Outputs in range [0, 4] (assuming the rotation matrices are proper)
fn calc_rot_err(a: Mat3A, b: Mat3A) -> f32 {
    calc_vec3_err(a.x_axis, b.x_axis)
        + calc_vec3_err(a.y_axis, b.y_axis)
        + calc_vec3_err(a.z_axis, b.z_axis)
}

fn calc_bool_err(a: bool, b: bool) -> f32 {
    f32::from(a != b)
}

/// Every tick gives 0.5 error
fn calc_time_err(a: f32, b: f32) -> f32 {
    (a - b).abs() * 60.0
}

///////////////

pub type StateErrSet = Vec<(String, f32)>;

pub fn map_phys_err(err_set: &mut StateErrSet, a: &PhysState, b: &PhysState, include_rot: bool) {
    err_set.push(("pos".to_string(), calc_vec3_err(a.pos, b.pos) / 100.0));

    if include_rot {
        err_set.push(("rot".to_string(), calc_rot_err(a.rot_mat, b.rot_mat) / 2.0));
    }

    // NOTE: Vel is typically less important than pos, but tends to indicate the error earlier,
    //  so it gets a disproportionately-high priority.
    err_set.push(("vel".to_string(), calc_vec3_err(a.vel, b.vel) / 100.0));
    err_set.push((
        "ang_vel".to_string(),
        calc_vec3_err(a.ang_vel, b.ang_vel) / 5.0,
    ));
}

pub fn map_ball_err(a: &BallState, b: &BallState, include_rot: bool) -> StateErrSet {
    let mut err_set = StateErrSet::default();
    map_phys_err(&mut err_set, &a.phys, &b.phys, include_rot);

    // TODO: Add error for heatseeker and dropshot vars

    for (name, _) in &mut err_set {
        *name = format!("ball_{name}");
    }
    err_set
}

pub fn map_car_err(a: &CarState, b: &CarState) -> StateErrSet {
    let mut err_set = StateErrSet::default();

    if a.is_demoed || b.is_demoed {
        // Nothing apart from the demo bool actually matters
        err_set.push((
            "is_demoed".to_string(),
            calc_bool_err(a.is_demoed, b.is_demoed),
        ));
        return err_set;
    }

    map_phys_err(&mut err_set, &a.phys, &b.phys, true);

    err_set.push((
        "boost".to_string(),
        (a.boost - b.boost).abs() / consts::BOOST_MAX,
    ));

    err_set.push((
        "is_on_ground".to_string(),
        calc_bool_err(a.is_on_ground, b.is_on_ground),
    ));

    err_set.push((
        "is_jumping".to_string(),
        calc_bool_err(a.is_jumping, b.is_jumping),
    ));
    err_set.push((
        "has_jumped".to_string(),
        calc_bool_err(a.has_jumped, b.has_jumped),
    ));

    err_set.push((
        "is_flipping".to_string(),
        calc_bool_err(a.is_flipping, b.is_flipping),
    ));
    err_set.push((
        "has_flipped".to_string(),
        calc_bool_err(a.has_flipped, b.has_flipped),
    ));
    err_set.push((
        "has_double_jumped".to_string(),
        calc_bool_err(a.has_double_jumped, b.has_double_jumped),
    ));
    err_set.push((
        "air_time".to_string(),
        calc_time_err(a.air_time, b.air_time),
    ));
    err_set.push((
        "air_time_since_jump".to_string(),
        calc_time_err(a.air_time_since_jump, b.air_time_since_jump),
    ));

    // TODO: Reimplement
    //err_set.push((
    //    "has_hit_ball".to_string(),
    //    calc_bool_err(a.ball_hit_info.is_some(), b.ball_hit_info.is_some()),
    //));

    // NOTE: Have to do these by hand because the strings must all be static
    err_set.push((
        "wheel_0_contact".to_string(),
        calc_bool_err(a.wheels_with_contact[0], b.wheels_with_contact[0]),
    ));
    err_set.push((
        "wheel_1_contact".to_string(),
        calc_bool_err(a.wheels_with_contact[1], b.wheels_with_contact[1]),
    ));
    err_set.push((
        "wheel_2_contact".to_string(),
        calc_bool_err(a.wheels_with_contact[2], b.wheels_with_contact[2]),
    ));
    err_set.push((
        "wheel_3_contact".to_string(),
        calc_bool_err(a.wheels_with_contact[3], b.wheels_with_contact[3]),
    ));

    err_set.push((
        "flip_rel_torque".to_string(),
        calc_vec3_err(a.flip_rel_torque, b.flip_rel_torque),
    ));

    err_set.push((
        "jump_time".to_string(),
        calc_time_err(a.jump_time, b.jump_time),
    ));

    err_set.push((
        "flip_time".to_string(),
        calc_time_err(a.flip_time, b.flip_time),
    ));

    err_set.push((
        "time_since_boosted".to_string(),
        calc_time_err(a.time_since_boosted, b.time_since_boosted),
    ));

    err_set.push((
        "is_boosting".to_string(),
        calc_bool_err(a.is_boosting, b.is_boosting),
    ));

    err_set.push((
        "boosting_time".to_string(),
        calc_time_err(a.boosting_time, b.boosting_time),
    ));

    err_set.push((
        "is_supersonic".to_string(),
        calc_bool_err(a.is_supersonic, b.is_supersonic),
    ));

    err_set.push((
        "supersonic_time".to_string(),
        calc_time_err(a.supersonic_time, b.supersonic_time),
    ));

    err_set.push((
        "handbrake_val".to_string(),
        (a.handbrake_val - b.handbrake_val).abs(),
    ));

    err_set.push((
        "is_auto_flipping".to_string(),
        calc_bool_err(a.is_auto_flipping, b.is_auto_flipping),
    ));

    err_set.push((
        "auto_flip_timer".to_string(),
        calc_time_err(a.auto_flip_timer, b.auto_flip_timer),
    ));

    err_set.push((
        "auto_flip_torque_scale".to_string(),
        (a.auto_flip_torque_scale - b.auto_flip_torque_scale).abs(),
    ));

    for (name, _) in &mut err_set {
        *name = format!("car_{name}");
    }

    err_set
}

#[derive(Debug, Clone, Default)]
pub struct StateComparison {
    pub car_errs: Vec<StateErrSet>,
    pub ball_err: Option<StateErrSet>,
}

impl StateComparison {
    pub fn combine_all_err_sets(&self) -> StateErrSet {
        // Hard-to-read iterator shenanigans
        self.ball_err
            .iter()
            .chain(self.car_errs.iter())
            .flatten()
            .cloned()
            .collect()
    }
}
