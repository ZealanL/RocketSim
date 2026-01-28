use std::fmt::{Debug, Display};

use rocketsim::{BallState, CarState, GameMode};

use crate::comparison_test::state_compare::StateComparison;

#[derive(Debug, Clone)]
pub struct TestResultState {
    pub game_mode: GameMode,
    pub car_state_pairs: Vec<(CarState, CarState)>,
    pub ball_state_pair: Option<(BallState, BallState)>,
    pub comparison: StateComparison,
}

fn show_diff<T: Display + PartialEq + Copy>(
    f: &mut std::fmt::Formatter,
    prefix: &str,
    new: T,
    old: T,
) -> std::fmt::Result {
    if new == old {
        f.write_fmt(format_args!("{prefix} [same]: {new}"))
    } else {
        f.write_fmt(format_args!("{prefix} [DIFF]: new={new} | old={old}"))
    }
}

impl Display for TestResultState {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_str("TestResultState {")?;
        for (car_idx, (car_state_new, car_state_old)) in self.car_state_pairs.iter().enumerate() {
            f.write_fmt(format_args!("\n\tCar[{car_idx}] {{"))?;

            let csn = car_state_new;
            let cso = car_state_old;

            f.write_fmt(format_args!("\n\t\tNew controls: {:?}", csn.controls))?;
            f.write_str("\n")?;
            show_diff(f, "\n\t\tpos", csn.phys.pos, cso.phys.pos)?;
            show_diff(
                f,
                "\n\t\trot_mat.forward",
                csn.phys.rot_mat.x_axis,
                cso.phys.rot_mat.x_axis,
            )?;
            show_diff(
                f,
                "\n\t\trot_mat.up",
                csn.phys.rot_mat.z_axis,
                cso.phys.rot_mat.z_axis,
            )?;
            show_diff(f, "\n\t\tvel", csn.phys.vel, cso.phys.vel)?;
            show_diff(f, "\n\t\tang_vel", csn.phys.ang_vel, cso.phys.ang_vel)?;
            f.write_str("\n")?;
            show_diff(f, "\n\t\tboost", csn.boost, cso.boost)?;
            show_diff(f, "\n\t\tis_on_ground", csn.is_on_ground, cso.is_on_ground)?;
            show_diff(f, "\n\t\thas_jumped", csn.has_jumped, cso.has_jumped)?;
            show_diff(f, "\n\t\tis_jumping", csn.is_jumping, cso.is_jumping)?;
            show_diff(f, "\n\t\tjump_time", csn.jump_time, cso.jump_time)?;
            show_diff(f, "\n\t\thas_flipped", csn.has_flipped, cso.has_flipped)?;
            show_diff(f, "\n\t\tflip_time", csn.flip_time, cso.flip_time)?;
            show_diff(f, "\n\t\tis_demoed", csn.is_demoed, cso.is_demoed)?;
            show_diff(
                f,
                "\n\t\twheels_in_contact",
                glam::BVec4::from_array(csn.wheels_with_contact),
                glam::BVec4::from_array(cso.wheels_with_contact),
            )?;

            f.write_str("\n\t}")?;
        }

        if let Some((ball_state_new, ball_state_old)) = self.ball_state_pair {
            f.write_fmt(format_args!("\n\tBall {{"))?;

            let bsn = ball_state_new;
            let bso = ball_state_old;

            show_diff(f, "\n\t\tpos", bsn.phys.pos, bso.phys.pos)?;

            if self.game_mode == GameMode::Snowday {
                show_diff(
                    f,
                    "\n\t\trot_mat.forward",
                    bsn.phys.rot_mat.x_axis,
                    bso.phys.rot_mat.x_axis,
                )?;
                show_diff(
                    f,
                    "\n\t\trot_mat.up",
                    bsn.phys.rot_mat.z_axis,
                    bso.phys.rot_mat.z_axis,
                )?;
            }

            show_diff(f, "\n\t\tvel", bsn.phys.vel, bso.phys.vel)?;
            show_diff(f, "\n\t\tang_vel", bsn.phys.ang_vel, bso.phys.ang_vel)?;

            // TODO: Display heatseeker/dropshot info in those respective gamemodes
            f.write_str("\n\t}")?;
        }

        f.write_str("\n}")
    }
}
