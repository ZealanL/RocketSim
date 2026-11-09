//! Validate available replay state before segmentation or simulator conversion.

use glam::Vec3A;
use rocketsim_test::rlpr::{
    cpp_records::{PhysRecord, VecRecord},
    tick_record::TickRecord,
};

use super::validate_v12_body_state;

/// Reject invalid recorded state with its row, native frame, and car slot.
///
/// Unavailable bodies contain placeholders. Do not validate those placeholders.
/// Do not repair rotations or remove invalid rows from the score denominator.
/// The rotation checks include the checks made by `PhysRecord::into`.
/// The determinant check also rejects a reflected basis.
pub fn validate_recorded_state(ticks: &[TickRecord]) -> Result<(), String> {
    for (row, tick) in ticks.iter().enumerate() {
        if !tick.has_physics() {
            continue;
        }
        if tick.ball_available() {
            validate_body(&tick.ball_record)
                .map_err(|error| body_error(row, "ball", &tick.ball_record, error))?;
        }
        for (slot, car) in tick.car_records.iter().enumerate() {
            if !car.is_available() {
                continue;
            }
            let validate = || -> Result<(), String> {
                validate_body(&car.phys)?;
                for (name, value) in [
                    ("throttle", car.prev_controls.throttle),
                    ("steer", car.prev_controls.steer),
                    ("pitch", car.prev_controls.pitch),
                    ("yaw", car.prev_controls.yaw),
                    ("roll", car.prev_controls.roll),
                    ("jump time", car.jump_time),
                    ("flip time", car.flip_time),
                    ("boost amount", car.boost_amount),
                    ("boosting time", car.boosting_time),
                    ("handbrake value", car.handbrake_val),
                ] {
                    if !value.is_finite() {
                        return Err(format!("{name} is not finite"));
                    }
                }
                if !vector_is_finite(car.flip_rel_torque) {
                    return Err("flip relative torque is not finite".to_string());
                }
                Ok(())
            };
            validate()
                .map_err(|error| body_error(row, &format!("car slot {slot}"), &car.phys, error))?;
        }
    }
    validate_v12_body_state(ticks)
}

fn body_error(row: usize, body: &str, phys: &PhysRecord, error: String) -> String {
    format!(
        "RLPR row {row} {body} native frame {}: {error}",
        phys.physics_frame
    )
}

fn vector_is_finite(vector: VecRecord) -> bool {
    vector.x.is_finite() && vector.y.is_finite() && vector.z.is_finite()
}

fn validate_body(phys: &PhysRecord) -> Result<(), String> {
    for (name, vector) in [
        ("position", phys.pos),
        ("linear velocity", phys.lin_vel),
        ("angular velocity", phys.ang_vel),
    ] {
        if !vector_is_finite(vector) {
            return Err(format!("{name} is not finite"));
        }
    }
    let rows = phys.rot.rows;
    if !rows.iter().all(|&row| vector_is_finite(row)) {
        return Err("rotation is not finite".to_string());
    }
    for i in 0..3 {
        if (1.0 - rows[i].length()).abs() >= 1e-6 {
            return Err(format!("rotation row {i} is not a unit vector"));
        }
        if Vec3A::dot(rows[i].into(), rows[(i + 1) % 3].into()).abs() >= 1e-6 {
            return Err(format!(
                "rotation rows {i} and {} are not orthogonal",
                (i + 1) % 3
            ));
        }
    }
    let [x, y, z] = rows.map(Vec3A::from);
    if x.cross(y).dot(z) <= 0.0 {
        return Err("rotation is not right-handed".to_string());
    }
    Ok(())
}
