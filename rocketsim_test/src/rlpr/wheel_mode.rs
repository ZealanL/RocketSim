//! Recording-level detection of the wheel suspension raycast model the
//! capture's producer used.
//!
//! RocketSim has two suspension-collision paths (see
//! [`rocketsim::WheelRaycastMode`]): a full collision-world raycast, and the
//! `SuspensionCollisionGrid` fast path whose unmarked cells resolve against
//! the analytic arena planes. Recordings produced by planes-only suspension
//! engines are only reproducible under the analytic model, so an evaluator
//! must select the matching mode per recording rather than globally.
//!
//! The check is data-driven: each recorded wheel sample is compared against
//! the analytic-plane prediction reconstructed from the previous tick's car
//! state (contact flag, suspension length, and contact normal). A recording
//! whose wheel stream is reproduced by the analytic model is classified
//! `ArenaPlanes`; facet-dependent captures keep the full raycast.

use glam::Vec3A;
use rocketsim::{
    CarBodyConfig, GameMode, WheelRaycastMode, consts,
    consts::arena::get_aabb,
};

use super::Recording;

/// Mismatch fraction of total wheel records tolerated before a capture is
/// declared facet-dependent. The planes-only reference capture disagrees
/// once per ~43k wheel records, while mesh-produced captures that need the
/// trimesh disagree on several percent of records.
const MAX_ANALYTIC_MISMATCH_FRAC: f64 = 0.005;
/// Wheel samples needed for a reliable classification; sparse captures fall
/// back to the full raycast.
const MIN_WHEEL_RECORDS: usize = 400;
/// Suspension-length agreement tolerance, BT.
const SUSP_TOL: f32 = 0.01;
/// Contact-normal agreement tolerance (length of the normal difference).
const NORMAL_TOL: f32 = 0.05;

#[derive(Debug, Default, Clone, Copy)]
pub struct WheelModeStats {
    pub wheel_records: usize,
    pub matches: usize,
    pub mismatches: usize,
    /// Analytic model predicts a contact the recording does not report.
    pub pred_only_contacts: usize,
    /// Recording reports a contact the analytic model cannot reproduce.
    pub real_only_contacts: usize,
    /// Contact flags agree but suspension or normal differs.
    pub contact_value_mismatches: usize,
}

impl WheelModeStats {
    fn analytic_consistent(&self) -> bool {
        self.wheel_records >= MIN_WHEEL_RECORDS
            && (self.mismatches as f64)
                <= self.wheel_records as f64 * MAX_ANALYTIC_MISMATCH_FRAC
    }
}

/// Compare a recording's wheel stream against the analytic arena-plane
/// suspension model, reconstructing each ray from the previous tick's car
/// state (wheel records are written by the step that produced their tick).
pub fn analytic_wheel_stats(
    recording: &Recording,
    game_mode: GameMode,
    car_body: &CarBodyConfig,
) -> WheelModeStats {
    let uu = consts::UU_TO_BT;
    // Wheel order matches the car setup: front-left, front-right, back-left,
    // back-right; `connection_point_offset.y` is positive and negated for the
    // left wheels.
    let conns = [
        Vec3A::new(
            car_body.front_wheels.connection_point_offset.x,
            -car_body.front_wheels.connection_point_offset.y,
            car_body.front_wheels.connection_point_offset.z,
        ),
        car_body.front_wheels.connection_point_offset,
        Vec3A::new(
            car_body.back_wheels.connection_point_offset.x,
            -car_body.back_wheels.connection_point_offset.y,
            car_body.back_wheels.connection_point_offset.z,
        ),
        car_body.back_wheels.connection_point_offset,
    ];
    let radius = [
        car_body.front_wheels.wheel_radius,
        car_body.front_wheels.wheel_radius,
        car_body.back_wheels.wheel_radius,
        car_body.back_wheels.wheel_radius,
    ];
    let sus_rest = [
        car_body.front_wheels.suspension_rest_length,
        car_body.front_wheels.suspension_rest_length,
        car_body.back_wheels.suspension_rest_length,
        car_body.back_wheels.suspension_rest_length,
    ];

    let aabb = get_aabb(game_mode);
    let extent_x = aabb.max.x * uu;
    let extent_y = aabb.max.y * uu;
    let height = aabb.max.z * uu;
    let is_hoops = game_mode == GameMode::Hoops;

    let mut stats = WheelModeStats::default();

    for i in 1..recording.ticks.len() {
        let prev = &recording.ticks[i - 1];
        let cur = &recording.ticks[i];
        for (prev_car, cur_car) in prev.car_records.iter().zip(&cur.car_records) {
            let pos = Vec3A::new(
                prev_car.phys.pos.x,
                prev_car.phys.pos.y,
                prev_car.phys.pos.z,
            ) * uu;
            let r = &prev_car.phys.rot;
            let fwd = Vec3A::new(r.rows[0].x, r.rows[1].x, r.rows[2].x);
            let right = Vec3A::new(r.rows[0].y, r.rows[1].y, r.rows[2].y);
            let up = Vec3A::new(r.rows[0].z, r.rows[1].z, r.rows[2].z);
            let dir = -up;

            for w in 0..4 {
                stats.wheel_records += 1;
                let hard = pos + (fwd * conns[w].x + right * conns[w].y + up * conns[w].z) * uu;
                // Effective production ray length: the stored rest is reduced
                // by max travel, then travel and the wheel radius are added
                // back, yielding configured rest + radius.
                let ray_len = (sus_rest[w] + radius[w]) * uu;
                let end = hard + dir * ray_len;
                let delta = end - hard;
                let dist = delta.length();
                if dist == 0.0 {
                    continue;
                }
                let d = delta / dist;

                let mut hit: Option<(f32, Vec3A)> = None;
                if end.z <= 0.0 || end.z >= height {
                    hit = if d.z < 0.0 {
                        Some(((5.96e-8 - hard.z) / d.z, Vec3A::Z))
                    } else {
                        Some(((height - hard.z) / d.z, Vec3A::NEG_Z))
                    };
                } else {
                    if d.x.signum() == hard.x.signum() {
                        let dp = (hard.x.abs() - extent_x).abs() / d.x.abs();
                        hit = Some((dp, Vec3A::new(-end.x.signum(), 0.0, 0.0)));
                    }
                    if is_hoops && d.y.signum() == hard.y.signum() {
                        let dp = (hard.y.abs() - extent_y).abs() / d.y.abs();
                        hit = Some((dp, Vec3A::new(0.0, -end.y.signum(), 0.0)));
                    }
                }

                let rw = &cur_car.wheels[w];
                let (pred_contact, pred_susp, pred_n) = match hit {
                    Some((dtp, n)) if dtp < dist => {
                        let cp = hard + d * dtp;
                        let trace = (hard - cp).dot(up);
                        let susp = (trace - radius[w] * uu).min(sus_rest[w] * uu);
                        (true, susp, n)
                    }
                    _ => (false, 0.0, Vec3A::ZERO),
                };

                if pred_contact != rw.has_contact {
                    stats.mismatches += 1;
                    if pred_contact {
                        stats.pred_only_contacts += 1;
                    } else {
                        stats.real_only_contacts += 1;
                    }
                    continue;
                }
                if rw.has_contact {
                    let susp_err = (pred_susp - rw.susp_length).abs();
                    let n_err = (pred_n
                        - Vec3A::new(
                            rw.contact_normal.x,
                            rw.contact_normal.y,
                            rw.contact_normal.z,
                        ))
                    .length();
                    if susp_err < SUSP_TOL && n_err < NORMAL_TOL {
                        stats.matches += 1;
                    } else {
                        stats.mismatches += 1;
                        stats.contact_value_mismatches += 1;
                    }
                } else {
                    stats.matches += 1;
                }
            }
        }
    }

    stats
}

/// Pick the suspension raycast mode whose wheel stream the recording
/// actually exhibits. Captures consistent with the analytic arena planes are
/// evaluated with `ArenaPlanes`; all others keep the full collision-world
/// raycast.
#[must_use]
pub fn detect_wheel_raycast_mode(
    recording: &Recording,
    game_mode: GameMode,
    car_body: &CarBodyConfig,
) -> WheelRaycastMode {
    if analytic_wheel_stats(recording, game_mode, car_body).analytic_consistent() {
        WheelRaycastMode::ArenaPlanes
    } else {
        WheelRaycastMode::World
    }
}
