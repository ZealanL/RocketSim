//! One-step reset evaluator for the non-validation Foxe Replay recordings.
//!
//! Each comparable transition restores the recorded state at tick `i`, applies
//! the recorded controls, advances RocketSim once, and reports the first
//! transition whose normalized state error reaches the comparison threshold.
//! Legacy Foxe multi-car captures can be sampled every two physics frames, so
//! only contiguous one-tick transitions are scored; this avoids inventing
//! missing intermediate controls. Captures made with the corrected recorder
//! should have frame deltas of one throughout active play.

use std::path::{Path, PathBuf};

use glam::Vec3A;
use rocketsim::{
    consts, Arena, ArenaConfig, ArenaEvent, CarBodyConfig, CarControls, CarState, GameMode,
    PhysState, Team,
};
use rocketsim_test::rlpr::{
    cpp_records::CarRecord, tick_record::TickRecord, wheel_mode, Recording,
};

const POS_TOL: f32 = 10.0;
const VEL_TOL: f32 = 3.0;
const ANG_TOL: f32 = 1.0;
const AXIS_TOL: f32 = 1.0;
const COMP_NAMES: [&str; 10] = ["c_pos", "c_vel", "c_ang", "c_fwd", "c_up", "b_pos", "b_vel", "b_ang", "b_fwd", "b_up"];

/// Reconstruct `air_time_since_jump`, which the RLPR record does not
/// serialize. Forward-tracked: it counts while airborne with `has_jumped &&
/// !is_jumping`, and resets otherwise. For cars airborne since the capture
/// start only a lower bound is known, so a dodge triggered before capture
/// cannot be timed exactly.
fn reconstruct_air_time_since_jump(recording: &Recording, num_cars: usize) -> Vec<Vec<f32>> {
    const DT: f32 = consts::TICK_TIME;
    let n = recording.ticks.len();
    let mut est = vec![vec![0.0f32; num_cars]; n];
    for j in 0..num_cars {
        let mut cur = 0.0f32;
        for i in 0..n {
            est[i][j] = cur;
            let r = &recording.ticks[i].car_records[j];
            // The v10 recorder can expose a single-tick airborne `is_jumping`
            // flicker for an ignored jump press (window expired, no impulse).
            // A real jump hold lasts multiple ticks or starts on the ground;
            // don't let the phantom state zero the window timer.
            let phantom_jump = r.is_jumping
                && !r.is_on_ground
                && r.jump_time == 0.0
                && i > 0
                && !recording.ticks[i - 1].car_records[j].is_jumping
                && i + 1 < n
                && !recording.ticks[i + 1].car_records[j].is_jumping;
            cur = if r.is_on_ground || (r.is_jumping && !phantom_jump) || !r.has_jumped {
                0.0
            } else {
                cur + DT
            };
        }
    }
    est
}

/// Per-car flag: a flip already in progress at the first observed tick has an
/// unverifiable internal `flip_time` (the recorder can serialize a timer reset
/// to zero for a dodge that began before the capture). Those transitions are
/// stepped but not scored until the car is observed not flipping.
fn compute_flip_untrusted(recording: &Recording, num_cars: usize) -> Vec<Vec<bool>> {
    let n = recording.ticks.len();
    let mut out = vec![vec![false; num_cars]; n];
    for j in 0..num_cars {
        let mut untrusted = recording.ticks.first().is_some_and(|t| {
            t.car_records.get(j).is_some_and(|r| r.is_flipping)
        });
        for i in 0..n {
            let r = &recording.ticks[i].car_records[j];
            if !r.is_flipping {
                untrusted = false;
            }
            out[i][j] = untrusted && r.is_flipping;
        }
    }
    out
}

fn set_car_states(
    arena: &mut Arena,
    car_ids: &[usize],
    tick: &TickRecord,
    controls: &[CarControls],
    previous_tick: Option<&TickRecord>,
    air_time_since_jump: &[f32],
    car_stale: &[bool],
) {
    for (i, &car_id) in car_ids.iter().enumerate() {
        let record = &tick.car_records[i];
        let recorded: CarState = (*record).into();
        let mut state = *arena.get_car_state(car_id);
        state.phys = recorded.phys;
        state.is_on_ground = recorded.is_on_ground;
        state.wheels_with_contact = recorded.wheels_with_contact;
        state.is_jumping = recorded.is_jumping;
        // The v10 recorder exposes the current jump input phase, while
        // RocketSim's legacy state machine also synthesizes the minimum hold
        // after a release. For replay restoration, an active state with a
        // released previous input is already post-impulse.
        if recorded.is_jumping && !controls[i].jump {
            state.is_jumping = false;
        }
        // A v10 capture exposes the jump-hold bit for the same delayed input
        // window in which a second jump is reported as `is_flipping`.  The
        // latter takes precedence: the flip tick must not receive another
        // upward hold impulse.
        if record.is_flipping {
            state.is_jumping = false;
        }
        state.is_flipping = recorded.is_flipping;
        // The corrected v10 Foxe recorder serializes the current jump hold
        // phase as a zero-time sample on the first active tick. Reconstruct
        // RocketSim's internal one-tick counter for this evaluator only;
        // legacy comparison recordings retain the canonical conversion.
        state.jump_ticks = if recorded.jump_ticks == 0
            && record.is_jumping
            && record.prev_controls.jump
        {
            1
        } else {
            recorded.jump_ticks
        };
        // In v10, a single in-air sample can expose `is_jumping` with a
        // zero jump timer even though the following transition has no jump
        // impulse. If the preceding sample was not jumping, retire that
        // transient state before stepping; ongoing zero-time jump samples
        // still use the normal first-tick reconstruction.
        if recorded.is_jumping
            && !recorded.is_on_ground
            && record.jump_time == 0.0
            && controls[i].jump
            && previous_tick
                .and_then(|previous| previous.car_records.get(i))
                .is_some_and(|previous| !previous.is_jumping)
        {
            state.jump_ticks = rocketsim::consts::car::jump::MAX_TICKS;
        }
        state.flip_time = recorded.flip_time;
        state.has_jumped = recorded.has_jumped;
        state.air_time_since_jump = air_time_since_jump[i];
        state.prev_controls = record.prev_controls.into();
        state.controls = controls[i];
        // The v10 RLPR writer stores the already-scaled dodge torque, while
        // RocketSim's state machine consumes the unit flip direction.
        let mut flip_rel_torque = recorded.flip_rel_torque;
        if flip_rel_torque.x.abs() > 1.5 || flip_rel_torque.y.abs() > 1.5 {
            flip_rel_torque.x /= consts::car::flip::TORQUE.x;
            flip_rel_torque.y /= consts::car::flip::TORQUE.y;
            flip_rel_torque.z = 0.0;
        }
        state.flip_rel_torque = flip_rel_torque;
        state.boost = recorded.boost;
        // RLPR v2 does not serialize BallHitInfo. Preserve the live cooldown
        // between contiguous transitions, matching the C++ RLPR runner; an
        // independent reset cannot reconstruct this hidden per-car state.
        state.world_contact_normal = record
            .phys
            .has_world_contact
            .then(|| record.phys.world_contact_normal.into());
        // A car whose phys record repeats verbatim while its physics frame
        // advances was not simulated by the engine that tick (demolished or
        // paused placeholder). Park it as demoed so it cannot interact with
        // the ball; a live parked car restores to the same pose anyway.
        state.is_demoed = car_stale[i];
        state.demo_respawn_timer = if car_stale[i] { 1e9 } else { 0.0 };

        if record.has_flip {
            state.has_double_jumped = false;
            state.has_flipped = false;
        } else if record.is_flipping {
            state.has_double_jumped = false;
            // The public v10 sample can expose the flip flag one frame before
            // the delayed jump input reaches the physics step. Keep the
            // hidden availability clear until that delayed edge is present;
            // once the flip time is non-zero (or the jump edge is recorded),
            // preserve the consumed state for ongoing torque.
            state.has_flipped = record.prev_controls.jump || record.flip_time > 0.0;
        } else if record.double_jumped_or_flipped && !state.has_flipped {
            // v10 exposes this bit as the available double-jump/flip phase;
            // the legacy corpus uses it as the consumed-double-jump marker.
            state.has_double_jumped = false;
        }
        arena.set_car_state(car_id, state);
    }
}

fn set_state(
    arena: &mut Arena,
    car_ids: &[usize],
    tick: &TickRecord,
    controls: &[CarControls],
    previous_tick: Option<&TickRecord>,
    air_time_since_jump: &[f32],
    car_stale: &[bool],
    ball_stale: bool,
) {
    set_car_states(arena, car_ids, tick, controls, previous_tick, air_time_since_jump, car_stale);
    let recorded_ball: PhysState = tick.ball_record.into();
    let mut ball = *arena.get_ball_state();
    ball.phys = recorded_ball;
    if ball_stale {
        // The record is a parked placeholder; keep the sim ball out of play so
        // it cannot phantom-touch a car and consume the extra-hit cooldown.
        ball.phys.pos = Vec3A::new(0.0, 0.0, 30000.0);
        ball.phys.vel = Vec3A::ZERO;
        ball.phys.ang_vel = Vec3A::ZERO;
    }
    arena.set_ball_state(ball);
}

fn norm(pred: &CarState, real: &CarRecord, ball_pred: &PhysState, ball_real: &rocketsim_test::rlpr::cpp_records::PhysRecord) -> (f32, [f32; 10]) {
    let rv: Vec3A = real.phys.lin_vel.into();
    let rp: Vec3A = real.phys.pos.into();
    let ra: Vec3A = real.phys.ang_vel.into();
    let rf: Vec3A = real.phys.rot.forward().into();
    let ru: Vec3A = real.phys.rot.up().into();
    let bp: Vec3A = ball_real.pos.into();
    let bv: Vec3A = ball_real.lin_vel.into();
    let ba: Vec3A = ball_real.ang_vel.into();
    let bf: Vec3A = ball_real.rot.forward().into();
    let bu: Vec3A = ball_real.rot.up().into();
    let errors = [
        (pred.phys.pos - rp).length() / POS_TOL,
        (pred.phys.vel - rv).length() / VEL_TOL,
        (pred.phys.ang_vel - ra).length() / ANG_TOL,
        (pred.phys.get_forward_dir() - rf).length() / AXIS_TOL,
        (pred.phys.get_up_dir() - ru).length() / AXIS_TOL,
        (ball_pred.pos - bp).length() / POS_TOL,
        (ball_pred.vel - bv).length() / VEL_TOL,
        (ball_pred.ang_vel - ba).length() / ANG_TOL,
        (ball_pred.get_forward_dir() - bf).length() / AXIS_TOL,
        (ball_pred.get_up_dir() - bu).length() / AXIS_TOL,
    ];
    let n = errors.iter().map(|e| e * e).sum::<f32>().sqrt();
    (n, errors)
}

fn tick_is_frozen(from: &TickRecord, to: &TickRecord) -> bool {
    if from.car_records.len() != to.car_records.len() {
        return false;
    }
    from.car_records.iter().zip(&to.car_records).all(|(a, b)| {
        a.phys.pos == b.phys.pos
            && a.phys.lin_vel == b.phys.lin_vel
            && a.phys.ang_vel == b.phys.ang_vel
    }) && from.ball_record.pos == to.ball_record.pos
        && from.ball_record.lin_vel == to.ball_record.lin_vel
        && from.ball_record.ang_vel == to.ball_record.ang_vel
}



#[derive(Debug)]
struct EvaluationSummary {
    name: String,
    ticks: usize,
    comparable: usize,
    survived: usize,
    first_fail: Option<(usize, u32)>,
}

fn evaluate(path: &Path) -> Result<EvaluationSummary, Box<dyn std::error::Error>> {
    let recording = Recording::from_file(path)?;
    let wheel_mode =
        wheel_mode::detect_wheel_raycast_mode(&recording, GameMode::Soccar, &CarBodyConfig::OCTANE);
    let mut arena = Arena::new_with_config(
        ArenaConfig::new(GameMode::Soccar).with_wheel_raycast_mode(wheel_mode),
    );
    let car_ids: Vec<usize> = (0..recording.info.num_cars as usize)
        .map(|i| {
            let team = if i % 2 == 0 { Team::Blue } else { Team::Orange };
            arena.add_car(team, CarBodyConfig::OCTANE)
        })
        .collect();

    let air_time_since_jump = reconstruct_air_time_since_jump(&recording, car_ids.len());
    let flip_untrusted = compute_flip_untrusted(&recording, car_ids.len());
    let mut first_fail = None;
    let mut max_norm = 0.0f32;
    let mut max_tick = 0usize;
    let mut first_car_fail: Vec<Option<(usize, f32, [f32; 10])>> = vec![None; car_ids.len()];
    let mut comparable = 0usize;
    let mut passing = 0usize;
    // Indexed by `car_idx * 10 + component`; sized for the recording's car
    // count so captures with more than four cars cannot index out of bounds.
    let mut fail_comp = vec![0usize; car_ids.len() * COMP_NAMES.len()];
    let only_i = std::env::var("FOX_ONLY_I").ok().and_then(|s| s.parse::<usize>().ok());
    for i in 0..recording.ticks.len().saturating_sub(1) {
        if let Some(only_i) = only_i {
            if i != only_i {
                continue;
            }
        }
        let from = &recording.ticks[i];
        let to = &recording.ticks[i + 1];
        if to.car_records.len() != car_ids.len() {
            continue;
        }
        let frame_delta = to.car_records[0]
            .phys
            .physics_frame
            .saturating_sub(from.car_records[0].phys.physics_frame);
        // The v10 capture serializes the input held at record time, while the
        // physics step consumes the input polled at tick start; a press
        // landing between the two can appear one record early or late. When a
        // neighbouring record carries a different input, which sample the game
        // applied to this transition is ambiguous and the step does not test
        // physics.
        let ambiguous_input = (i + 2 < recording.ticks.len()
            && to
                .car_records
                .iter()
                .zip(&recording.ticks[i + 2].car_records)
                .any(|(a, b)| a.prev_controls != b.prev_controls))
            || from
                .car_records
                .iter()
                .zip(&to.car_records)
                .any(|(a, b)| a.prev_controls != b.prev_controls);
        // Foxe's multi-car captures are sampled every two physics frames;
        // without the missing intermediate controls, those transitions are
        // not strict one-tick parity observations. Frozen and gapped
        // transitions are still simulated so hidden wheel/solver state keeps
        // tracking the recorded trajectory; only the comparison is skipped.
        // A flip in progress at the capture boundary carries a serialized
        // `flip_time` that does not reflect the real dodge timer, so its
        // torque/damp behavior cannot be reproduced; still step the state.
        let untrusted_flip = flip_untrusted[i].iter().any(|&u| u);
        // The recorder keeps writing a car's last phys sample while the engine
        // stops simulating it (demolition, paused placeholder): the sample is
        // bit-identical to the previous tick's even though the physics frame
        // advanced. Genuinely parked cars are kept live; they would score ~0
        // either way, and wrongly parking them would make them intangible to
        // the ball.
        let mut car_stale = vec![false; car_ids.len()];
        let mut car_teleport = vec![false; car_ids.len()];
        for (j, (ca, cb)) in from.car_records.iter().zip(&to.car_records).enumerate() {
            let frame_advanced = cb.phys.physics_frame != ca.phys.physics_frame;
            let phys_identical = ca.phys.pos == cb.phys.pos
                && ca.phys.rot == cb.phys.rot
                && ca.phys.lin_vel == cb.phys.lin_vel
                && ca.phys.ang_vel == cb.phys.ang_vel;
            let parked = ca.is_on_ground
                && Vec3A::from(ca.phys.lin_vel).length() < 1.0;
            car_stale[j] = frame_advanced && phys_identical && !parked;
            // A respawn teleports the car far beyond what one tick of travel
            // can cover; there is no intermediate state to reproduce.
            let moved = (Vec3A::from(cb.phys.pos) - Vec3A::from(ca.phys.pos)).length();
            car_teleport[j] = moved > Vec3A::from(ca.phys.lin_vel).length() * consts::TICK_TIME + 100.0;
        }
        // Goal/replay bookkeeping teleports the ball between phases; the jump
        // has no physical transition to compare against. During the replay the
        // recorder also repeats the ball's last live sample verbatim (a parked
        // mid-air placeholder) while cars keep driving — not a physical step.
        let ball_teleport = (Vec3A::from(to.ball_record.pos) - Vec3A::from(from.ball_record.pos))
            .length()
            > 100.0;
        let ball_stale = to.ball_record.physics_frame != from.ball_record.physics_frame
            && from.ball_record.pos == to.ball_record.pos
            && from.ball_record.rot == to.ball_record.rot
            && from.ball_record.lin_vel == to.ball_record.lin_vel
            && from.ball_record.ang_vel == to.ball_record.ang_vel
            && !(Vec3A::from(from.ball_record.pos).z < 120.0
                && Vec3A::from(from.ball_record.lin_vel).length() < 1.0);
        // Also park the sim ball while its *source* sample is already a stale
        // placeholder so it never interacts with the cars mid-tick.
        let ball_park = ball_stale
            || (from.ball_record.pos.z >= 120.0
                && Vec3A::from(from.ball_record.lin_vel).length() < 1.0
                && Vec3A::from(from.ball_record.ang_vel).length() < 1.0);
        let scorable = !ambiguous_input
            && !untrusted_flip
            && !tick_is_frozen(from, to)
            && !ball_teleport
            && !ball_stale
            && !car_stale.iter().all(|&s| s)
            && frame_delta == 1;
        if scorable {
            comparable += 1;
        }
        let controls: Vec<CarControls> = to
            .car_records
            .iter()
            .map(|record| record.prev_controls.into())
            .collect();
        set_state(
            &mut arena,
            &car_ids,
            from,
            &controls,
            i.checked_sub(1).and_then(|previous| recording.ticks.get(previous)),
            &air_time_since_jump[i],
            &car_stale,
            ball_park,
        );
        if std::env::var("FOX_DEBUG_PRE_I").ok().and_then(|s| s.parse::<usize>().ok()) == Some(i) {
            println!("DEBUG pre ball: pos={:?} vel={:?} ang={:?}", arena.get_ball_state().phys.pos, arena.get_ball_state().phys.vel, arena.get_ball_state().phys.ang_vel);
        }
        if std::env::var("FOX_DEBUG_PRE_I").ok().and_then(|s| s.parse::<usize>().ok()) == Some(i) {
            for (j, &car_id) in car_ids.iter().enumerate() {
                let s = arena.get_car_state(car_id);
                println!("DEBUG pre car{j}: pos={:?} vel={:?} ang={:?} fwd={:?} right={:?} up={:?} flags g/j/f={}/{}/{} jump_ticks={} flip_time={:.6} airsj={:.4} hasjump/flip/double={}/{}/{} prev={:?} controls={:?}",
                    s.phys.pos, s.phys.vel, s.phys.ang_vel, s.phys.get_forward_dir(), s.phys.get_right_dir(), s.phys.get_up_dir(),
                    s.is_on_ground, s.is_jumping, s.is_flipping, s.jump_ticks, s.flip_time, s.air_time_since_jump, s.has_jumped, s.has_flipped, s.has_double_jumped,
                    s.prev_controls, s.controls);
            if std::env::var_os("FOX_FULL_STATE").is_some() {
                println!("DEBUG full car{j}: {s:#?}");
            }
            }
        }
        arena.step_tick();
        let boundary_ball = arena.get_ball_state().phys;
        if arena.get_last_step_events().len() > 0 && std::env::var_os("FOX_DEBUG_EVENTS").is_some() {
            println!("DEBUG events at i={i} source_ball_world={}/{} source_touch={} target_ball_vel={:?}: {:?}",
                from.ball_record.has_world_contact, to.ball_record.has_world_contact,
                to.car_records[0].is_touching_ball, to.ball_record.lin_vel,
                arena.get_last_step_events());
        }
        if std::env::var("FOX_DEBUG_I").ok().and_then(|s| s.parse::<usize>().ok()) == Some(i) {
            println!("DEBUG events at i={i}: {:?}", arena.get_last_step_events());
        }
        if i == 0 {
            continue;
        }
        let ball = boundary_ball;
        if std::env::var("FOX_DEBUG_I").ok().and_then(|s| s.parse::<usize>().ok()) == Some(i) {
            println!("DEBUG state at i={i}: pred ball vel={:?} source target={:?}", ball.vel, Vec3A::from(to.ball_record.lin_vel));
            for (j, &car_id) in car_ids.iter().enumerate() {
                let pred = arena.get_car_state(car_id);
                println!("DEBUG car{j} pred pos={:?} vel={:?} ang={:?} flags jumping/flipping={} / {} ticks={} flip_time={:.6} torque={:?} | source to pos={:?} vel={:?} ang={:?} flags={} / {} jump_time={:.6} flip_time={:.6} torque={:?}",
                    pred.phys.pos, pred.phys.vel, pred.phys.ang_vel,
                    pred.is_jumping, pred.is_flipping, pred.jump_ticks, pred.flip_time,
                    pred.flip_rel_torque,
                    to.car_records[j].phys.pos, to.car_records[j].phys.lin_vel, to.car_records[j].phys.ang_vel,
                    to.car_records[j].is_jumping, to.car_records[j].is_flipping,
                    to.car_records[j].jump_time, to.car_records[j].flip_time,
                    to.car_records[j].flip_rel_torque);
                println!("DEBUG wheels car{j}: {:?}", arena.get_car_wheel_debug(car_id));
                println!("DEBUG wheel normals car{j}: {:?}", arena.get_car_wheel_contact_normals(car_id));
                println!("DEBUG target wheels car{j}: {:?}", to.car_records[j].wheels);
            }
        }
        let mut tick_norm_sq = 0.0;
        let mut errors = [0.0f32; 10];
        for (j, &car_id) in car_ids.iter().enumerate() {
            if car_stale[j] || car_teleport[j] {
                continue;
            }
            let (car_norm, car_errors) = norm(
                arena.get_car_state(car_id),
                &to.car_records[j],
                &ball,
                &to.ball_record,
            );
            tick_norm_sq += car_norm * car_norm;
            if std::env::var("FOX_PRINT_NORM_I").ok().and_then(|s| s.parse::<usize>().ok()) == Some(i) {
                println!("DEBUG norm i={i} car{j} norm={car_norm} errors={car_errors:?}");
            }
            if scorable && car_norm >= 1.0 && first_car_fail[j].is_none() {
                first_car_fail[j] = Some((i, car_norm, car_errors));
            }
            if j == 0 {
                errors = car_errors;
            }
        }
        let tick_norm = tick_norm_sq.sqrt();
        if scorable && tick_norm < 1.0 {
            passing += 1;
        } else if scorable {
            // argmax over the dominant error component across cars
            let mut worst = 0usize;
            let mut worst_v = 0.0f32;
            for (j, &car_id) in car_ids.iter().enumerate() {
                if car_stale[j] || car_teleport[j] {
                    continue;
                }
                let (_, e) = norm(
                    arena.get_car_state(car_id),
                    &to.car_records[j],
                    &ball,
                    &to.ball_record,
                );
                for (k, &v) in e.iter().enumerate() {
                    if v > worst_v {
                        worst_v = v;
                        worst = j * 10 + k;
                    }
                }
            }
            fail_comp[worst] += 1;
            if std::env::var_os("FOX_PRINT_FAILS").is_some() {
                println!("FAIL i={i} norm={tick_norm:.3} worst={}:{}", COMP_NAMES[worst % 10], worst / 10);
            }
        }
        if tick_norm > max_norm {
            max_norm = tick_norm;
            max_tick = i;
        }
        if scorable && tick_norm >= 1.0 && first_fail.is_none() {
            first_fail = Some((i, to.car_records[0].phys.physics_frame, errors));
            println!("  first-fail details at i={i}, target frame={}:", to.car_records[0].phys.physics_frame);
            for (j, &car_id) in car_ids.iter().enumerate() {
                let predicted = arena.get_car_state(car_id);
                let recorded: CarState = to.car_records[j].into();
                println!("    car {j}: pred pos={:?} vel={:?} ang={:?} | real pos={:?} vel={:?} ang={:?} ground pred/real={}/{} wheels={:?}/{:?}",
                    predicted.phys.pos, predicted.phys.vel, predicted.phys.ang_vel,
                    recorded.phys.pos, recorded.phys.vel, recorded.phys.ang_vel,
                    predicted.is_on_ground, recorded.is_on_ground,
                    predicted.wheels_with_contact, recorded.wheels_with_contact);
                println!("      world_contact pred={:?} real={} point={:?} normal={:?}",
                    predicted.world_contact_normal,
                    to.car_records[j].phys.has_world_contact,
                    Vec3A::from(to.car_records[j].phys.world_contact_point),
                    Vec3A::from(to.car_records[j].phys.world_contact_normal));
                println!("      touching_ball prev/target={}/{}", from.car_records[j].is_touching_ball, to.car_records[j].is_touching_ball);
                println!("      controls prev={:?} target_prev={:?}", from.car_records[j].prev_controls, to.car_records[j].prev_controls);
            }
            println!("    ball: pred pos={:?} vel={:?} ang={:?} | real pos={:?} vel={:?} ang={:?}",
                ball.pos, ball.vel, ball.ang_vel,
                Vec3A::from(to.ball_record.pos), Vec3A::from(to.ball_record.lin_vel), Vec3A::from(to.ball_record.ang_vel));
            println!("    ball world contact prev/target={}/{}; car0 world contact prev/target={}/{}",
                from.ball_record.has_world_contact, to.ball_record.has_world_contact,
                from.car_records[0].phys.has_world_contact, to.car_records[0].phys.has_world_contact);
            println!("    ball transition prev pos={:?} vel={:?} -> target pos={:?} vel={:?}",
                Vec3A::from(from.ball_record.pos), Vec3A::from(from.ball_record.lin_vel),
                Vec3A::from(to.ball_record.pos), Vec3A::from(to.ball_record.lin_vel));
        }
    }

    let mut parts: Vec<String> = Vec::new();
    for (k, &c) in fail_comp.iter().enumerate() {
        if c > 0 {
            let car_idx = k / COMP_NAMES.len();
            let prefix = if car_idx > 0 {
                format!("{car_idx}.")
            } else {
                String::new()
            };
            parts.push(format!("{}{}={c}", prefix, COMP_NAMES[k % COMP_NAMES.len()]));
        }
    }
    println!("{}: ticks={}, first_fail={:?}, max_norm={:.4} at {}, passing={}/{} ({:.2}%) fail_comp: {}", recording.name, recording.ticks.len(), first_fail.as_ref().map(|(i, frame, _)| (*i, *frame)), max_norm, max_tick, passing, comparable, 100.0 * passing as f32 / comparable.max(1) as f32, parts.join(" "));
    println!("  first per-car failures: {:?}", first_car_fail);
    if let Some((i, frame, errors)) = first_fail.as_ref() {
        println!("  first failing transition i={i}, target_frame={frame}; car/ball components=[pos,vel,ang,forward,up] each");
        println!("  normalized components: {:?}", errors);
    }
    let survived = first_fail
        .as_ref()
        .map_or(comparable, |(i, _, _)| *i);
    Ok(EvaluationSummary {
        name: recording.name,
        ticks: recording.ticks.len(),
        comparable,
        survived,
        first_fail: first_fail.map(|(i, frame, _)| (i, frame)),
    })
}

/// Replay the captured controls continuously, retaining Bullet's private
/// manifolds and the per-car ball-hit cooldown between observations. At
/// frame gaps or frozen samples, teleport only to the next recorded state and
/// resume on the following contiguous transition.
fn evaluate_live(path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let recording = Recording::from_file(path)?;
    let wheel_mode =
        wheel_mode::detect_wheel_raycast_mode(&recording, GameMode::Soccar, &CarBodyConfig::OCTANE);
    let mut arena = Arena::new_with_config(
        ArenaConfig::new(GameMode::Soccar).with_wheel_raycast_mode(wheel_mode),
    );
    let car_ids: Vec<usize> = (0..recording.info.num_cars as usize)
        .map(|i| {
            let team = if i % 2 == 0 { Team::Blue } else { Team::Orange };
            arena.add_car(team, CarBodyConfig::OCTANE)
        })
        .collect();
    if recording.ticks.len() < 2 || recording.ticks[0].car_records.len() != car_ids.len() {
        println!("{}: insufficient compatible ticks", recording.name);
        return Ok(());
    }

    let air_time_since_jump = reconstruct_air_time_since_jump(&recording, car_ids.len());
    let controls_for = |tick: &TickRecord| -> Vec<CarControls> {
        tick.car_records.iter().map(|r| r.prev_controls.into()).collect()
    };
    set_state(
        &mut arena,
        &car_ids,
        &recording.ticks[0],
        &controls_for(&recording.ticks[1]),
        None,
        &air_time_since_jump[0],
        &vec![false; car_ids.len()],
        false,
    );
    arena.step_tick();

    let mut first_fail = None;
    let mut max_norm = 0.0f32;
    let mut max_tick = 0usize;
    let mut scored = 0usize;
    let mut skip_after_reset = false;
    let mut preserve_ball_state = false;
    let mut boundary_closer = 0usize;
    let mut completed_closer = 0usize;
    let mut phase_ties = 0usize;
    for i in 1..recording.ticks.len().saturating_sub(1) {
        let from = &recording.ticks[i];
        let to = &recording.ticks[i + 1];
        if from.car_records.len() != car_ids.len() || to.car_records.len() != car_ids.len() {
            continue;
        }
        let frame_delta = to.car_records[0]
            .phys
            .physics_frame
            .saturating_sub(from.car_records[0].phys.physics_frame);
        if frame_delta != 1 || tick_is_frozen(from, to) {
            if i + 1 < recording.ticks.len() {
                let next_controls = if i + 2 < recording.ticks.len() {
                    controls_for(&recording.ticks[i + 2])
                } else {
                    controls_for(to)
                };
                set_state(
                    &mut arena,
                    &car_ids,
                    to,
                    &next_controls,
                    Some(from),
                    &air_time_since_jump[i + 1],
                    &vec![false; car_ids.len()],
                    false,
                );
            }
            preserve_ball_state = false;
            skip_after_reset = std::env::var_os("FOX_LIVE_WARMUP_SKIP").is_some();
            continue;
        }

        let controls = if std::env::var_os("FOX_USE_FROM_CONTROLS").is_some() {
            controls_for(from)
        } else {
            controls_for(to)
        };
        // A car whose phys record repeats verbatim while its frame advances
        // was not simulated that tick (demolished or paused placeholder):
        // park it as demoed so it neither drifts nor interacts. Mirrors the
        // stale-car handling in `evaluate` and `rlpr_bench`'s live mode.
        let mut car_stale = vec![false; car_ids.len()];
        for (j, (ca, cb)) in from.car_records.iter().zip(&to.car_records).enumerate() {
            let frame_advanced = cb.phys.physics_frame != ca.phys.physics_frame;
            let phys_identical = ca.phys.pos == cb.phys.pos
                && ca.phys.rot == cb.phys.rot
                && ca.phys.lin_vel == cb.phys.lin_vel
                && ca.phys.ang_vel == cb.phys.ang_vel;
            let parked = ca.is_on_ground && Vec3A::from(ca.phys.lin_vel).length() < 1.0;
            car_stale[j] = frame_advanced && phys_identical && !parked;
        }
        if !preserve_ball_state {
            let recorded_ball: PhysState = from.ball_record.into();
            let mut ball = *arena.get_ball_state();
            ball.phys = recorded_ball;
            arena.set_ball_state(ball);
        }
        // Keep the car trajectory on the recorder's exact post-step boundary;
        // each scored transition starts from the restored source state.
        set_car_states(&mut arena, &car_ids, from, &controls, Some(&recording.ticks[i.saturating_sub(1)]), &air_time_since_jump[i], &car_stale);
        arena.step_tick();
        let boundary_ball = arena.get_ball_state().phys;
        preserve_ball_state = arena
            .get_last_step_events()
            .iter()
            .any(|event| matches!(event, ArenaEvent::CarHitBall(_)));
        if std::env::var_os("FOX_INFINITE_BOOST").is_some() {
            for &car_id in &car_ids {
                let mut state = *arena.get_car_state(car_id);
                state.boost = 100.0;
                arena.set_car_state(car_id, state);
            }
        }
        scored += 1;
        if skip_after_reset {
            skip_after_reset = false;
            continue;
        }
        let ball = boundary_ball;
        let completed_ball = arena.get_ball_state().phys;
        let mut tick_norm_sq = 0.0;
        let mut completed_norm_sq = 0.0;
        for (j, &car_id) in car_ids.iter().enumerate() {
            let (car_norm, _) = norm(
                arena.get_car_state(car_id),
                &to.car_records[j],
                &ball,
                &to.ball_record,
            );
            tick_norm_sq += car_norm * car_norm;
            let (completed_norm, _) = norm(
                arena.get_car_state(car_id),
                &to.car_records[j],
                &completed_ball,
                &to.ball_record,
            );
            completed_norm_sq += completed_norm * completed_norm;
        }
        let tick_norm = tick_norm_sq.sqrt();
        let completed_norm = completed_norm_sq.sqrt();
        match tick_norm.partial_cmp(&completed_norm) {
            Some(std::cmp::Ordering::Less) => boundary_closer += 1,
            Some(std::cmp::Ordering::Greater) => completed_closer += 1,
            _ => phase_ties += 1,
        }
        if std::env::var_os("FOX_LIVE_TRACE").is_some() && (390..=400).contains(&i) {
            let p = arena.get_car_state(car_ids[0]);
            println!("LIVE TRACE i={i} norm={tick_norm:.4} pred_vel={:?} src_vel={:?} pred_ang={:?} src_ang={:?} pred_boost={:.3} src_boost={:.3} pred_wheels={:?} src_wheels={:?} events={:?}",
                p.phys.vel, Vec3A::from(to.car_records[0].phys.lin_vel),
                p.phys.ang_vel, Vec3A::from(to.car_records[0].phys.ang_vel),
                p.boost, to.car_records[0].boost_amount * 100.0,
                p.wheels_with_contact, to.car_records[0].wheels.map(|w| w.has_contact),
                arena.get_last_step_events());
            println!("  controls from={:?} target_prev={:?}", from.car_records[0].prev_controls, to.car_records[0].prev_controls);
        }
        if std::env::var("FOX_LIVE_DEBUG_I").ok().and_then(|s| s.parse::<usize>().ok()) == Some(i) {
            println!("LIVE DEBUG i={i} source_frame={} norm={tick_norm:.4} pred_ball={:?} source_ball={:?} events={:?}",
                to.car_records[0].phys.physics_frame, ball.vel, Vec3A::from(to.ball_record.lin_vel), arena.get_last_step_events());
            println!("  completed ball={:?}", arena.get_ball_state().phys);
            for (j, &car_id) in car_ids.iter().enumerate() {
                let predicted = arena.get_car_state(car_id);
                let recorded: CarState = to.car_records[j].into();
                let from_rec = &from.car_records[j];
                println!("  car{j} pred pos={:?} vel={:?} ang={:?} | source pos={:?} vel={:?} ang={:?} world={}/{} wheels={:?}/{:?}",
                    predicted.phys.pos, predicted.phys.vel, predicted.phys.ang_vel,
                    recorded.phys.pos, recorded.phys.vel, recorded.phys.ang_vel,
                    predicted.is_on_ground, recorded.is_on_ground,
                    predicted.wheels_with_contact, recorded.wheels_with_contact);
                println!("    source impulses: {:?}", to.car_records[j].phys.impulse_records());
                println!("    source flags: ground={} jumping={} flipping={} jump_time={:.6} flip_time={:.6} has_jumped={} double_or_flip={} has_flip={} flip_torque={:?} controls={:?}",
                    to.car_records[j].is_on_ground, to.car_records[j].is_jumping,
                    to.car_records[j].is_flipping, to.car_records[j].jump_time,
                    to.car_records[j].flip_time, to.car_records[j].has_jumped,
                    to.car_records[j].double_jumped_or_flipped, to.car_records[j].has_flip,
                    to.car_records[j].flip_rel_torque, to.car_records[j].prev_controls);
                println!("    from flags: ground={} jumping={} flipping={} jump_time={:.6} flip_time={:.6} has_jumped={} double_or_flip={} has_flip={} flip_torque={:?} controls={:?}",
                    from_rec.is_on_ground, from_rec.is_jumping, from_rec.is_flipping,
                    from_rec.jump_time, from_rec.flip_time, from_rec.has_jumped,
                    from_rec.double_jumped_or_flipped, from_rec.has_flip,
                    from_rec.flip_rel_torque, from_rec.prev_controls);
                println!("    predicted flags: ground={} jumping={} flipping={} jump_ticks={} flip_time={:.6} has_jumped={} has_flipped={} has_double_jumped={} flip_torque={:?} controls={:?}",
                    predicted.is_on_ground, predicted.is_jumping, predicted.is_flipping,
                    predicted.jump_ticks, predicted.flip_time, predicted.has_jumped,
                    predicted.has_flipped, predicted.has_double_jumped,
                    predicted.flip_rel_torque, predicted.controls);
                #[cfg(debug_assertions)]
                println!("    predicted impulses: {:?}", arena.get_car_impulse_history(j));
            }
        }
        if tick_norm > max_norm {
            max_norm = tick_norm;
            max_tick = i;
        }
        if tick_norm >= 1.0 && first_fail.is_none() {
            first_fail = Some((i, to.car_records[0].phys.physics_frame, tick_norm));
            println!("  live first-fail i={i} frame={} norm={tick_norm:.4}", to.car_records[0].phys.physics_frame);
        }
    }
    println!(
        "{}: live scored={}, first_fail={:?}, max_norm={:.4} at {}; phase closer boundary/completed/tie={}/{}/{}",
        recording.name, scored, first_fail, max_norm, max_tick,
        boundary_closer, completed_closer, phase_ties
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // The corrected v10 recorder exposes post-step caps and jump/air-control
    // semantics that differ from the legacy comparison corpus. Enable those
    // semantics only for this authoritative Foxe validation executable.
    unsafe { std::env::set_var("RS_V10_PARITY", "1") };
    rocketsim::init(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../collision_meshes"),
        true,
    )?;
    let replay_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("recordings")
        .join("Foxe Replay");
    // New recorder captures are kept in `Foxe Replay/fixed` so the original
    // debugging set remains immutable. Fall back to the original directory
    // when running against the existing files.
    let fixed_root = replay_root.join("fixed");
    let fixed_has_recordings = fixed_root.is_dir()
        && std::fs::read_dir(&fixed_root)
            .ok()
            .into_iter()
            .flat_map(|entries| entries.filter_map(Result::ok))
            .any(|entry| {
                entry
                    .path()
                    .extension()
                    .is_some_and(|ext| ext.eq_ignore_ascii_case("rlpr"))
            });
    let root = if fixed_has_recordings {
        fixed_root
    } else {
        replay_root
    };
    let requested: Vec<PathBuf> = std::env::args_os().skip(1).map(PathBuf::from).collect();
    let (paths, benchmark_requested) = if requested.is_empty() {
        let mut paths: Vec<_> = std::fs::read_dir(&root)?
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .filter(|path| path.extension().is_some_and(|ext| ext.eq_ignore_ascii_case("rlpr")))
            .collect();
        paths.sort();
        (paths, false)
    } else if requested.len() == 1 && requested[0].is_dir() {
        let mut paths: Vec<_> = std::fs::read_dir(&requested[0])?
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .filter(|path| path.extension().is_some_and(|ext| ext.eq_ignore_ascii_case("rlpr")))
            .collect();
        paths.sort();
        let is_foxe_benchmark = requested[0]
            .file_name()
            .is_some_and(|name| name.to_string_lossy().eq_ignore_ascii_case("test"));
        (paths, is_foxe_benchmark || std::env::var_os("FOX_BENCHMARK").is_some())
    } else {
        (requested, false)
    };

    let live = std::env::var_os("FOX_LIVE").is_some();
    let mut summaries = Vec::new();
    for path in paths {
        if live {
            evaluate_live(&path)?;
        } else {
            summaries.push(evaluate(&path)?);
        }
    }

    if benchmark_requested && !live {
        let recordings = summaries.len();
        let passed = summaries.iter().filter(|summary| summary.first_fail.is_none()).count();
        let comparable: usize = summaries.iter().map(|summary| summary.comparable).sum();
        let survived: usize = summaries.iter().map(|summary| summary.survived).sum();
        let recording_percent = if recordings == 0 {
            0.0
        } else {
            100.0 * passed as f32 / recordings as f32
        };
        let transition_percent = if comparable == 0 {
            0.0
        } else {
            100.0 * survived as f32 / comparable as f32
        };
        println!(
            "FoxeBeachmark: recordings={passed}/{recordings} ({recording_percent:.2}%), strict_transitions={survived}/{comparable} ({transition_percent:.2}%)"
        );
    }
    Ok(())
}
