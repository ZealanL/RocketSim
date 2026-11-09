use glam::{Mat3A, Vec3A};
use rocketsim::{ActivationState, CarState, consts::BT_TO_UU};
use rocketsim_test::rlpr::{BallExt, CarExt, RowExt, WheelContactExt, cpp_records::VecRecord};

use super::{
    super::common::{
        SETTLED_HEIGHT_TOL_UU, restore_jump_hold_continuity, restore_recorded_boost_state,
        restore_recorded_handbrake, restore_test_tick,
    },
    *,
};

fn info_for(min: [f32; 3], max: [f32; 3]) -> RecordingInfo {
    RecordingInfo {
        num_cars: 4,
        hitbox_rel_min_bt: VecRecord::new(min[0], min[1], min[2]),
        hitbox_rel_max_bt: VecRecord::new(max[0], max[1], max[2]),
    }
}

/// Exact Daizen header bounds (decompressed bytes 17-40).
fn daizen_info() -> RecordingInfo {
    info_for(
        [-1.133026, -0.871704, -0.077060],
        [1.493369, 0.871704, 0.560828],
    )
}

/// Shared Wisp/PartyCannon header bounds (decompressed bytes 17-40).
fn octane_info() -> RecordingInfo {
    info_for(
        [-0.927561, -0.866994, 0.028509],
        [1.482587, 0.866994, 0.801690],
    )
}

#[test]
fn daizen_header_maps_to_plank() {
    let index = body_preset_index(&daizen_info()).expect("Daizen header is a known preset");
    assert_eq!(BODY_PRESET_NAMES[index], "Plank");
    assert_eq!(
        BODY_PRESETS[index].hitbox_size,
        CarBodyConfig::PLANK.hitbox_size
    );
    assert_eq!(
        BODY_PRESETS[index].hitbox_pos_offset,
        CarBodyConfig::PLANK.hitbox_pos_offset
    );
}

#[test]
fn octane_header_stays_octane() {
    let index = body_preset_index(&octane_info()).expect("Wisp header is a known preset");
    assert_eq!(BODY_PRESET_NAMES[index], "Octane");
    assert_eq!(
        BODY_PRESETS[index].hitbox_size,
        CarBodyConfig::OCTANE.hitbox_size
    );
    assert_eq!(
        BODY_PRESETS[index].hitbox_pos_offset,
        CarBodyConfig::OCTANE.hitbox_pos_offset
    );
}

#[test]
fn closest_presets_stay_distinct() {
    // Psyclops is Octane + 0.134 uu on every size axis: with a 0.01 uu
    // tolerance it must match Psyclops, never Octane.
    let half = HITBOX_SIZES[6] * 0.5;
    let min = (HITBOX_OFFSETS[6] - half) / BT_TO_UU;
    let max = (HITBOX_OFFSETS[6] + half) / BT_TO_UU;
    let info = info_for(min.to_array(), max.to_array());
    let index = body_preset_index(&info).expect("exact preset bounds match");
    assert_eq!(BODY_PRESET_NAMES[index], "Psyclops");
}

#[test]
fn unknown_header_is_an_error() {
    let info = info_for([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]);
    assert!(body_preset_index(&info).is_err());
}

#[test]
fn same_count_body_switch_rebuilds_arena() {
    init();
    let mut backend = V3Backend::new();
    backend.set_body_from_info(&octane_info()).unwrap();
    backend.ensure_all_cars(4);
    // Same body, same count: no rebuild, so a planted state survives.
    let sentinel = Vec3A::new(1234.0, 567.0, 89.0);
    let mut planted = *backend.arena.get_car_state(backend.car_ids[0].unwrap());
    planted.phys.pos = sentinel;
    backend
        .arena
        .set_car_state_full(backend.car_ids[0].unwrap(), planted);
    backend.ensure_all_cars(4);
    assert_eq!(
        backend
            .arena
            .get_car_state(backend.car_ids[0].unwrap())
            .phys
            .pos,
        sentinel
    );
    // Same count, new body: rebuild, so the planted state is gone.
    assert_eq!(backend.set_body_from_info(&daizen_info()).unwrap(), "Plank");
    backend.ensure_all_cars(4);
    assert_eq!(backend.car_ids.len(), 4);
    assert_ne!(
        backend
            .arena
            .get_car_state(backend.car_ids[0].unwrap())
            .phys
            .pos,
        sentinel
    );
}

#[test]
fn unknown_header_keeps_previous_roster() {
    init();
    let mut backend = V3Backend::new();
    backend.set_bodies(&[2, 0]);
    let bad = info_for([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]);
    assert!(backend.set_body_from_info(&bad).is_err());
    assert_eq!(backend.bodies, vec![2, 0]);
}

#[test]
fn partial_arena_maps_active_ids_and_frozen_teams() {
    init();
    let mut backend = V3Backend::new();
    backend.set_bodies(&[0; 6]);
    // Slot 3 is unavailable in this mask.
    backend.ensure_cars(&[true, true, true, false, true, true]);
    assert_eq!(
        backend.car_ids,
        vec![Some(0), Some(1), Some(2), None, Some(3), Some(4)]
    );
    for (slot, car_id) in backend.car_ids.iter().enumerate() {
        let Some(car_id) = car_id else {
            continue;
        };
        assert_eq!(
            backend.arena.get_car_info(*car_id).team,
            backend.team_for_slot(slot),
            "slot {slot} team"
        );
    }
    // A full mask rebuilds six cars and keeps the frozen slot teams.
    backend.ensure_cars(&[true; 6]);
    assert_eq!(backend.car_ids.len(), 6);
    for (slot, car_id) in backend.car_ids.iter().enumerate() {
        let car_id = car_id.expect("full mask has every slot");
        assert_eq!(
            backend.arena.get_car_info(car_id).team,
            backend.team_for_slot(slot),
            "slot {slot} team"
        );
    }
}

#[test]
fn mixed_roster_builds_per_car_bodies() {
    init();
    let mut backend = V3Backend::new();
    // Plank against Octane, as in the bundled london_vs_nexto capture.
    backend.set_bodies(&[2, 0]);
    backend.ensure_all_cars(2);
    assert_eq!(backend.arena_body.as_ref().unwrap().len(), 2);
    assert_eq!(
        backend.arena_body.as_ref().unwrap()[0].hitbox_size,
        CarBodyConfig::PLANK.hitbox_size
    );
    assert_eq!(
        backend.arena_body.as_ref().unwrap()[1].hitbox_size,
        CarBodyConfig::OCTANE.hitbox_size
    );
}

#[test]
fn explicit_octane_hybrid_roster_builds_each_body_geometry() {
    init();
    let mut backend = V3Backend::new();
    backend.set_bodies(&[0, 4]);
    backend.ensure_all_cars(2);

    let configs = backend.arena_body.as_ref().unwrap();
    for (slot, expected) in [(0, CarBodyConfig::OCTANE), (1, CarBodyConfig::HYBRID)] {
        assert_eq!(configs[slot].hitbox_size, expected.hitbox_size);
        assert_eq!(configs[slot].hitbox_pos_offset, expected.hitbox_pos_offset);
    }
}

#[test]
fn short_roster_pads_with_octane() {
    init();
    let mut backend = V3Backend::new();
    backend.set_bodies(&[2]);
    backend.ensure_all_cars(3);
    let configs = backend.arena_body.as_ref().unwrap();
    assert_eq!(configs[0].hitbox_size, CarBodyConfig::PLANK.hitbox_size);
    assert_eq!(configs[1].hitbox_size, CarBodyConfig::OCTANE.hitbox_size);
    assert_eq!(configs[2].hitbox_size, CarBodyConfig::OCTANE.hitbox_size);
}

#[test]
fn roster_change_rebuilds_without_count_change() {
    init();
    let mut backend = V3Backend::new();
    backend.set_bodies(&[0, 0]);
    backend.ensure_all_cars(2);
    let before = backend.arena_body.clone();
    backend.set_bodies(&[2, 0]);
    backend.ensure_all_cars(2);
    assert_ne!(backend.arena_body, before);
}

#[test]
fn fresh_arena_extra_hit_suppression_over_blocks_one_step_deferred() {
    use super::super::common::ReplayBackend;

    init();
    let mut backend = V3Backend::new();
    backend.ensure_all_cars(1);
    let car_id = backend.car_ids[0].unwrap();
    // Fresh backend: gating open.
    assert_eq!(
        backend.arena.get_car_state(car_id).last_extra_hit_tick,
        None
    );
    // At tick count 0 the production gate cannot block exactly one step:
    // every `Some(last)` blocks step 0 and step 1. The harness keeps the
    // conservative over-block and counts it. Exact one-step suppression
    // is deferred until the production gate can represent it.
    assert_eq!(backend.arena.tick_count(), 0);
    backend.suppress_next_extra_hit(0, true);
    assert_eq!(
        backend.arena.get_car_state(car_id).last_extra_hit_tick,
        Some(0)
    );
    assert_eq!(backend.restore_counts.extra_hit_fresh_arena_overblock, 1);
    // Clearing re-opens the gate.
    backend.suppress_next_extra_hit(0, false);
    assert_eq!(
        backend.arena.get_car_state(car_id).last_extra_hit_tick,
        None
    );
    // At count C >= 1 the block covers exactly the next step: last is
    // set to C - 1, so last + 1 < C fails once, then passes. This path
    // does not increment the fresh-arena counter.
    backend.arena.step_tick();
    assert_eq!(backend.arena.tick_count(), 1);
    backend.suppress_next_extra_hit(0, true);
    assert_eq!(
        backend.arena.get_car_state(car_id).last_extra_hit_tick,
        Some(0)
    );
    assert_eq!(backend.restore_counts.extra_hit_fresh_arena_overblock, 1);
}

#[test]
fn preset_settled_heights_separate_the_wheels() {
    init();
    let heights = preset_settled_heights();
    for (index, height) in heights.iter().enumerate() {
        assert!(
            height.is_finite() && *height > 0.0,
            "{} settled at {height}",
            BODY_PRESET_NAMES[index]
        );
    }
    // Octane and Hybrid share their wheels and are the one genuine tie.
    assert!((heights[0] - heights[4]).abs() < SETTLED_HEIGHT_TOL_UU);
    // Every other preset must be separable by the match tolerance.
    for a in 0..NUM_BODY_PRESETS {
        for b in (a + 1)..NUM_BODY_PRESETS {
            let gap = (heights[a] - heights[b]).abs();
            let tied = (a == 0 && b == 4) || (a == 4 && b == 0);
            assert!(
                tied || gap > SETTLED_HEIGHT_TOL_UU,
                "{} and {} are only {gap} uu apart",
                BODY_PRESET_NAMES[a],
                BODY_PRESET_NAMES[b]
            );
        }
    }
}

fn motion_vec(x: f32, y: f32, z: f32) -> VecRecord {
    VecRecord::new(x, y, z)
}

fn motion_ident_rot() -> rocketsim_test::rlpr::cpp_records::Mat3Record {
    rocketsim_test::rlpr::cpp_records::Mat3Record {
        rows: [
            motion_vec(1.0, 0.0, 0.0),
            motion_vec(0.0, 1.0, 0.0),
            motion_vec(0.0, 0.0, 1.0),
        ],
    }
}

fn motion_phys(
    pos: (f32, f32, f32),
    vel: (f32, f32, f32),
    ang: (f32, f32, f32),
) -> rocketsim_test::rlpr::cpp_records::PhysRecord {
    let mut phys: rocketsim_test::rlpr::cpp_records::PhysRecord = unsafe { std::mem::zeroed() };
    phys.rot = motion_ident_rot();
    phys.pos = motion_vec(pos.0, pos.1, pos.2);
    phys.lin_vel = motion_vec(vel.0, vel.1, vel.2);
    phys.ang_vel = motion_vec(ang.0, ang.1, ang.2);
    phys
}

fn motion_car(
    pos: (f32, f32, f32),
    vel: (f32, f32, f32),
    ang: (f32, f32, f32),
) -> rocketsim_test::rlpr::cpp_records::CarRecord {
    rocketsim_test::rlpr::cpp_records::CarRecord {
        phys: motion_phys(pos, vel, ang),
        is_on_ground: true,
        is_jumping: false,
        is_flipping: false,
        jump_time: 0.0,
        flip_time: 0.0,
        has_jumped: false,
        double_jumped_or_flipped: false,
        has_flip: false,
        flip_rel_torque: motion_vec(0.0, 0.0, 0.0),
        boost_amount: 0.33,
        is_touching_ball: false,
        prev_controls: rocketsim_test::rlpr::cpp_records::ControlsRecord {
            throttle: 0.0,
            steer: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
            jump: false,
            boost: false,
            handbrake: false,
        },
        wheels: [rocketsim_test::rlpr::cpp_records::WheelRecord {
            susp_length: 0.0,
            susp_rel_vel: 0.0,
            has_contact: false,
            contact_normal: motion_vec(0.0, 0.0, 1.0),
            steer_amount: 0.0,
            engine_force: 0.0,
            brake: 0.0,
            lat_friction: 0.0,
            long_friction: 0.0,
            extra_pushback: 0.0,
        }; 4],
        is_touching_car: false,
        _touch_pad: [0; 3],
        is_boosting: false,
        _boost_pad: [0; 3],
        boosting_time: 0.0,
        handbrake_val: 0.0,
        last_ball_touch_frame: rocketsim_test::rlpr::TOUCH_FRAME_UNKNOWN,
        last_demolish_frame: rocketsim_test::rlpr::DEMOLISH_FRAME_UNKNOWN,
        physics_available: 1,
        _availability_pad: [0; 3],
    }
}

fn motion_tick(car0: (f32, f32, f32), car1: (f32, f32, f32), ball: (f32, f32, f32)) -> TickRecord {
    TickRecord {
        level_tick: 0,
        row_flags: rocketsim_test::rlpr::ROW_FLAGS_BALL_AVAILABLE,
        reserved0: 0,
        car_records: vec![
            motion_car(car0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            motion_car(car1, (10.0, 0.0, 0.0), (0.1, 0.0, 0.0)),
        ],
        ball_record: motion_phys(ball, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        replay_state: None,
    }
}

#[test]
fn reset_motion_updates_only_pos_vel_ang_vel() {
    init();
    let mut backend = V3Backend::new();
    backend.set_bodies(&[0, 0]);
    let seed = motion_tick((0.0, 0.0, 100.0), (100.0, 0.0, 100.0), (0.0, 0.0, 500.0));
    backend.reset(&seed);
    // Plant distinct live history: orientation, timers, flags, controls,
    // boost latch, cooldowns, and ball mode state.
    let yaw90 = glam::Mat3A::from_rotation_z(std::f32::consts::FRAC_PI_2);
    for slot in 0..2 {
        let car_id = backend.car_ids[slot].unwrap();
        let mut live = *backend.arena.get_car_state(car_id);
        live.phys.rot_mat = yaw90;
        live.handbrake_val = 0.7;
        live.is_boosting = true;
        live.boosting_time = 1.25;
        live.bump_cooldown_timer = 0.4;
        live.bump_cooldown_victim = Some(backend.car_ids[1 - slot].unwrap());
        live.last_extra_hit_tick = Some(42);
        live.jump_hold_broken = true;
        live.is_jumping = true;
        live.boost = 50.0;
        live.controls.throttle = 1.0;
        live.jump_ticks = 7;
        live.flip_time = 2.5;
        backend.arena.set_car_state_full(car_id, live);
    }
    let mut ball = *backend.arena.get_ball_state();
    ball.phys.rot_mat = yaw90;
    ball.tick_count_since_kickoff = 123;
    backend.arena.set_ball_state(ball);
    let ball_before = *backend.arena.get_ball_state();
    let ground_before: Vec<bool> = backend
        .car_ids
        .iter()
        .flatten()
        .map(|&car_id| backend.arena.get_car_state(car_id).is_on_ground())
        .collect();

    // Target carries new motion vectors but identity orientation: the
    // recorded orientation must be ignored.
    let mut target = motion_tick(
        (1000.0, 2000.0, 300.0),
        (1100.0, 2000.0, 300.0),
        (4000.0, 5000.0, 600.0),
    );
    target.car_records[0].phys.lin_vel = motion_vec(100.0, 200.0, 300.0);
    target.car_records[0].phys.ang_vel = motion_vec(1.0, 2.0, 3.0);
    target.car_records[1].phys.lin_vel = motion_vec(110.0, 210.0, 310.0);
    target.car_records[1].phys.ang_vel = motion_vec(1.5, 2.5, 3.5);
    target.ball_record.lin_vel = motion_vec(400.0, 500.0, 600.0);
    target.ball_record.ang_vel = motion_vec(4.0, 5.0, 6.0);
    backend.reset_motion(&target);

    for slot in 0..2 {
        let car_id = backend.car_ids[slot].unwrap();
        let live = *backend.arena.get_car_state(car_id);
        let want: CarState = target.car_records[slot].into();
        // Exactly three vector fields follow the recording.
        assert_eq!(live.phys.pos, want.phys.pos, "car {slot} pos");
        assert_eq!(live.phys.vel, want.phys.vel, "car {slot} vel");
        assert_eq!(live.phys.ang_vel, want.phys.ang_vel, "car {slot} ang_vel");
        // Orientation preserved.
        assert_eq!(live.phys.rot_mat, yaw90, "car {slot} rot");
        // Representative history preserved.
        assert_eq!(live.handbrake_val, 0.7, "car {slot} handbrake");
        assert!(live.is_boosting, "car {slot} boost latch");
        assert_eq!(live.boosting_time, 1.25, "car {slot} boost time");
        assert_eq!(live.bump_cooldown_timer, 0.4, "car {slot} bump cooldown");
        assert_eq!(
            live.bump_cooldown_victim,
            Some(backend.car_ids[1 - slot].unwrap()),
            "car {slot} bump victim"
        );
        assert_eq!(live.last_extra_hit_tick, Some(42), "car {slot} extra gate");
        assert!(live.jump_hold_broken, "car {slot} jump latch");
        assert_eq!(
            live.is_on_ground(),
            ground_before[slot],
            "car {slot} ground cache"
        );
        assert!(live.is_jumping, "car {slot} jump flag");
        assert_eq!(live.boost, 50.0, "car {slot} boost");
        assert_eq!(live.controls.throttle, 1.0, "car {slot} controls");
        assert_eq!(live.jump_ticks, 7, "car {slot} jump ticks");
        assert_eq!(live.flip_time, 2.5, "car {slot} flip time");
    }
    let ball = *backend.arena.get_ball_state();
    let want_ball: PhysState = target.ball_record.into();
    assert_eq!(ball.phys.pos, want_ball.pos);
    assert_eq!(ball.phys.vel, want_ball.vel);
    assert_eq!(ball.phys.ang_vel, want_ball.ang_vel);
    assert_eq!(ball.phys.rot_mat, yaw90);
    assert_eq!(
        ball.tick_count_since_kickoff,
        ball_before.tick_count_since_kickoff
    );
    assert_eq!(ball.hs_info.y_target_dir, ball_before.hs_info.y_target_dir);
    assert_eq!(ball.ds_info.charge_level, ball_before.ds_info.charge_level);
}

#[test]
fn reset_motion_matches_full_state_setter_reference() {
    init();
    let mut optimized = V3Backend::new();
    optimized.set_bodies(&[0, 0]);
    let seed = motion_tick((0.0, 0.0, 100.0), (100.0, 0.0, 100.0), (0.0, 0.0, 500.0));
    optimized.reset(&seed);
    optimized.refresh_sticky_gates();

    // Give the live state contact and input history that a motion reset
    // must retain. The reference backend starts from an exact arena clone.
    for (slot, car_id) in optimized.car_ids.iter().enumerate() {
        let Some(car_id) = car_id else {
            continue;
        };
        let mut controls = CarControls::DEFAULT;
        controls.jump = slot == 0;
        controls.throttle = if slot == 0 { 0.75 } else { -0.25 };
        optimized.arena.edit_car_state_full(*car_id, |state| {
            state.set_prev_controls(controls);
        });
    }
    let mut reference = V3Backend {
        arena: optimized.arena.clone(),
        car_ids: optimized.car_ids.clone(),
        arena_mask: optimized.arena_mask.clone(),
        bodies: optimized.bodies.clone(),
        teams: optimized.teams.clone(),
        arena_body: optimized.arena_body.clone(),
        arena_teams: optimized.arena_teams.clone(),
        dodge_deadzone: optimized.dodge_deadzone,
        mem_weight_mode: optimized.mem_weight_mode,
        car_history: optimized.car_history.clone(),
        pad_history: optimized.pad_history.clone(),
        restore_counts: RestoreCounts::default(),
    };

    let target = motion_tick(
        (1200.0, 2100.0, 350.0),
        (1450.0, 2200.0, 400.0),
        (4100.0, 5100.0, 650.0),
    );

    // This is the prior manual set_car_state_full implementation. It
    // deliberately copies the live state and changes only three vectors.
    for (slot, car) in target.car_records.iter().enumerate() {
        let car_id = reference.car_ids[slot].unwrap();
        let recorded: CarState = (*car).into();
        let mut live = *reference.arena.get_car_state(car_id);
        live.phys.pos = recorded.phys.pos;
        live.phys.vel = recorded.phys.vel;
        live.phys.ang_vel = recorded.phys.ang_vel;
        reference.arena.set_car_state_full(car_id, live);
    }
    let recorded_ball: PhysState = target.ball_record.into();
    let mut ball = *reference.arena.get_ball_state();
    ball.phys.pos = recorded_ball.pos;
    ball.phys.vel = recorded_ball.vel;
    ball.phys.ang_vel = recorded_ball.ang_vel;
    ball.wake();
    reference.arena.set_ball_state(ball);

    optimized.reset_motion(&target);
    let assert_same_state = |actual: &V3Backend, expected: &V3Backend, phase: &str| {
        for slot in 0..actual.car_ids.len() {
            assert_eq!(
                format!(
                    "{:?}",
                    actual.arena.get_car_state(actual.car_ids[slot].unwrap())
                ),
                format!(
                    "{:?}",
                    expected
                        .arena
                        .get_car_state(expected.car_ids[slot].unwrap())
                ),
                "car {slot} differs {phase}"
            );
        }
        assert_eq!(
            format!("{:?}", actual.arena.get_ball_state()),
            format!("{:?}", expected.arena.get_ball_state()),
            "ball differs {phase}"
        );
    };
    assert_same_state(&optimized, &reference, "after reset");

    for step in 0..10 {
        let controls: Vec<CarControls> = (0..2)
            .map(|slot| CarControls {
                throttle: if slot == 0 { 0.6 } else { -0.3 },
                steer: if step % 2 == 0 { 0.2 } else { -0.2 },
                jump: step == 3 && slot == 0,
                ..CarControls::DEFAULT
            })
            .collect();
        for (slot, car_id) in optimized.car_ids.iter().enumerate() {
            let Some(car_id) = car_id else {
                continue;
            };
            optimized.arena.set_car_controls(*car_id, controls[slot]);
        }
        for (slot, car_id) in reference.car_ids.iter().enumerate() {
            let Some(car_id) = car_id else {
                continue;
            };
            reference.arena.set_car_controls(*car_id, controls[slot]);
        }
        optimized.arena.step_tick();
        reference.arena.step_tick();
        assert_same_state(&optimized, &reference, "after step");
    }
}

#[test]
fn reset_motion_needs_sized_arena() {
    init();
    let mut backend = V3Backend::new();
    let tick = motion_tick((0.0, 0.0, 100.0), (100.0, 0.0, 100.0), (0.0, 0.0, 500.0));
    // No `reset` yet, so no cars exist: explicit panic, not a rebuild.
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        backend.reset_motion(&tick);
    }));
    assert!(result.is_err());
}

#[test]
fn optional_restores_apply_fields_that_set_state_leaves_alone() {
    init();
    let mut backend = V3Backend::new();
    backend.ensure_all_cars(1);
    let car_id = backend.car_ids[0].unwrap();
    backend.arena.edit_car_state_full(car_id, |state| {
        state.handbrake_val = 0.9;
        state.jump_hold_broken = true;
        state.is_boosting = false;
        state.boosting_time = 0.0;
    });
    let mut tick = restore_test_tick(1, Vec3A::ZERO, Vec3A::ZERO);
    tick.car_records[0].is_boosting = true;
    tick.car_records[0].boosting_time = 1.5;
    tick.car_records[0].handbrake_val = 0.5;
    tick.car_records[0].is_jumping = true;
    tick.car_records[0].prev_controls.jump = true;

    backend.set_state(&tick);
    let state = *backend.arena.get_car_state(car_id);
    assert_eq!(state.handbrake_val, 0.9);
    assert!(state.jump_hold_broken);
    assert!(!state.is_boosting);

    let ticks = [tick];
    restore_recorded_boost_state(&mut backend, &ticks, 0, true);
    restore_recorded_handbrake(&mut backend, &ticks, 0, true);
    restore_jump_hold_continuity(&mut backend, &ticks, 0, 0);

    let state = *backend.arena.get_car_state(car_id);
    assert!(state.is_boosting);
    assert_eq!(state.boosting_time, 1.5);
    assert_eq!(state.handbrake_val, 0.5);
    assert!(!state.jump_hold_broken);
}

#[test]
fn reset_keeps_boost_pad_state_absent_from_rlpr() {
    init();
    let mut backend = V3Backend::new();
    backend.set_bodies(&[0]);
    backend.ensure_all_cars(1);
    let mut pad = backend.arena.get_boost_pad_state(0);
    pad.cooldown = 3.0;
    backend.arena.set_boost_pad_state(0, pad);

    let tick = restore_test_tick(0, Vec3A::ZERO, Vec3A::ZERO);
    backend.reset(&tick);

    assert_eq!(backend.arena.get_boost_pad_state(0).cooldown, 3.0);
}

/// One v12 row with one car extension.
fn v12_tick(car: CarExt) -> TickRecord {
    let mut tick = restore_test_tick(1, Vec3A::ZERO, Vec3A::ZERO);
    tick.replay_state = Some(RowExt {
        version: 1,
        group_mask: 1,
        cars: vec![car],
        ..RowExt::default()
    });
    tick
}

#[test]
fn v12_supersonic_restores_flag_and_active_timer() {
    init();
    let mut backend = V3Backend::new();
    backend.ensure_all_cars(1);
    let car_id = backend.car_ids[0].unwrap();
    let tick = v12_tick(CarExt {
        b_supersonic: Some(true),
        time_below_supersonic_s: Some(0.75),
        ..CarExt::default()
    });

    backend.set_state(&tick);
    let state = *backend.arena.get_car_state(car_id);
    assert!(state.is_supersonic);
    assert_eq!(state.supersonic_grace_timer, 0.75);

    let tick = v12_tick(CarExt {
        b_supersonic: Some(false),
        // Native timer freezes after expiry. RocketSim requires zero.
        time_below_supersonic_s: Some(1.008),
        ..CarExt::default()
    });
    backend.set_state(&tick);
    let state = *backend.arena.get_car_state(car_id);
    assert!(!state.is_supersonic);
    assert_eq!(state.supersonic_grace_timer, 0.0);
}

#[test]
fn v12_wheel_world_contact_does_not_replace_core_contact() {
    init();
    let mut backend = V3Backend::new();
    backend.ensure_all_cars(1);
    let car_id = backend.car_ids[0].unwrap();
    // The v12 world group reports one wheel against world geometry. The
    // v11 core row reports wheels 0 and 1 in contact with something that
    // is not world geometry, such as another car or the ball. Core
    // contact is authoritative, and the world group is distinct
    // unsupported data. The old substitution would keep one contact.
    let mut wheels = [WheelContactExt::default(); 4];
    wheels[2].has_contact_world_geometry = true;
    let mut tick = v12_tick(CarExt {
        wheel_contacts: Some(wheels),
        ..CarExt::default()
    });
    tick.car_records[0].is_on_ground = false;
    tick.car_records[0].wheels[0].has_contact = true;
    tick.car_records[0].wheels[1].has_contact = true;
    tick.car_records[0].wheels[2].has_contact = false;
    tick.car_records[0].wheels[3].has_contact = false;

    backend.set_state(&tick);
    let state = *backend.arena.get_car_state(car_id);
    assert_eq!(state.num_wheels_in_contact(), 2);
    assert!(!state.is_on_ground());
    assert_eq!(backend.restore_counts.wheel_world_ignored, 1);
}

#[test]
fn v12_base_grounding_wins_over_wheel_world_count() {
    init();
    let mut backend = V3Backend::new();
    backend.ensure_all_cars(1);
    let car_id = backend.car_ids[0].unwrap();
    // Native grounded, but the world group reports no world contact.
    let mut tick = v12_tick(CarExt {
        wheel_contacts: Some([WheelContactExt::default(); 4]),
        ..CarExt::default()
    });
    tick.car_records[0].is_on_ground = true;

    backend.set_state(&tick);
    assert!(backend.arena.get_car_state(car_id).is_on_ground());

    // Native airborne, but the world group reports three contacts.
    let mut wheels = [WheelContactExt::default(); 4];
    for wheel in wheels.iter_mut().take(3) {
        wheel.has_contact_world_geometry = true;
    }
    let mut tick = v12_tick(CarExt {
        wheel_contacts: Some(wheels),
        ..CarExt::default()
    });
    tick.car_records[0].is_on_ground = false;

    backend.set_state(&tick);
    assert!(!backend.arena.get_car_state(car_id).is_on_ground());
}

#[test]
fn v12_car_fields_follow_recording_slot_on_sparse_mask() {
    init();
    let mut backend = V3Backend::new();
    backend.set_bodies(&[0; 3]);
    backend.ensure_cars(&[true, false, true]);
    let mut tick = restore_test_tick(1, Vec3A::ZERO, Vec3A::ZERO);
    let mut placeholder = tick.car_records[0].clone();
    placeholder.physics_available = 0;
    tick.car_records = vec![
        tick.car_records[0].clone(),
        placeholder,
        tick.car_records[0].clone(),
    ];
    tick.replay_state = Some(RowExt {
        version: 1,
        group_mask: 1,
        cars: vec![
            CarExt {
                b_supersonic: Some(true),
                time_below_supersonic_s: Some(0.25),
                inv_inertia_world: Some([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]),
                ..CarExt::default()
            },
            CarExt::default(),
            CarExt {
                b_supersonic: Some(false),
                time_below_supersonic_s: Some(0.5),
                inv_inertia_world: Some([[9.0, 8.0, 7.0], [6.0, 5.0, 4.0], [3.0, 2.0, 1.0]]),
                ..CarExt::default()
            },
        ],
        ..RowExt::default()
    });

    backend.set_state(&tick);
    backend.restore_body_state(&tick);
    let slot0 = *backend.arena.get_car_state(backend.car_ids[0].unwrap());
    let slot2 = *backend.arena.get_car_state(backend.car_ids[2].unwrap());
    assert!(slot0.is_supersonic);
    assert_eq!(slot0.supersonic_grace_timer, 0.25);
    assert!(!slot2.is_supersonic);
    assert_eq!(slot2.supersonic_grace_timer, 0.0);
    assert_eq!(
        backend
            .arena
            .body_state(
                backend
                    .arena
                    .car_body_id(backend.car_ids[0].unwrap())
                    .unwrap()
            )
            .unwrap()
            .inv_inertia_tensor_world
            .row(0),
        Vec3A::new(1.0, 2.0, 3.0)
    );
    assert_eq!(
        backend
            .arena
            .body_state(
                backend
                    .arena
                    .car_body_id(backend.car_ids[2].unwrap())
                    .unwrap()
            )
            .unwrap()
            .inv_inertia_tensor_world
            .row(0),
        Vec3A::new(9.0, 8.0, 7.0)
    );
}

#[test]
fn v12_active_autoflip_without_history_is_partial_and_jump_uses_base_row() {
    init();
    let mut backend = V3Backend::new();
    backend.ensure_all_cars(1);
    let car_id = backend.car_ids[0].unwrap();
    let mut tick = v12_tick(CarExt {
        autoflip_active: Some(true),
        autoflip_right: Some(true),
        autoflip_activity_time_s: Some(0.5),
        autoflip_active_time_s: Some(0.25),
        autoflip_inactive_time_s: Some(0.0),
        jump_active: Some(true),
        jump_activity_time_s: Some(0.5),
        jump_inactive_time_s: Some(0.0),
        inv_inertia_world: Some([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
        ..CarExt::default()
    });
    tick.car_records[0].is_jumping = false;

    backend.set_state(&tick);
    let state = *backend.arena.get_car_state(car_id);
    assert!(!state.is_auto_flipping);
    assert_eq!(state.auto_flip_timer, 0.0);
    assert!(!state.is_jumping);
}

#[test]
fn v12_car_tensor_restores_row_order_and_preserves_absent_tensor() {
    init();
    let mut backend = V3Backend::new();
    backend.ensure_all_cars(1);
    let car_id = backend.car_ids[0].unwrap();
    let rows = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
    let tick = v12_tick(CarExt {
        inv_inertia_world: Some(rows),
        ..CarExt::default()
    });

    backend.set_state(&tick);
    backend.restore_body_state(&tick);
    backend.refresh_sticky_gates();
    let body_id = backend.arena.car_body_id(car_id).unwrap();
    let body = backend.arena.body_state(body_id).unwrap();
    for (index, row) in rows.iter().enumerate() {
        assert_eq!(
            body.inv_inertia_tensor_world.row(index),
            Vec3A::from_array(*row),
            "row {index}"
        );
    }

    // An absent tensor keeps the current body value.
    let sentinel = Mat3A::from_rows(
        Vec3A::new(0.5, 0.0, 0.0),
        Vec3A::new(0.0, 0.6, 0.0),
        Vec3A::new(0.0, 0.0, 0.7),
    );
    let mut seeded = body;
    seeded.inv_inertia_tensor_world = sentinel;
    backend
        .arena
        .set_body_state(body_id, seeded)
        .expect("test tensor is finite");
    backend.restore_body_state(&v12_tick(CarExt::default()));
    assert_eq!(
        backend
            .arena
            .body_state(body_id)
            .unwrap()
            .inv_inertia_tensor_world,
        sentinel
    );
}

#[test]
fn v12_ball_body_state_restores_after_sync_without_motion_change() {
    init();
    let mut backend = V3Backend::new();
    backend.ensure_all_cars(1);
    let mut tick = restore_test_tick(1, Vec3A::ZERO, Vec3A::ZERO);
    tick.ball_record.pos = VecRecord::new(100.0, -20.0, 200.0);
    tick.ball_record.lin_vel = VecRecord::new(1.0, 2.0, 3.0);
    tick.ball_record.ang_vel = VecRecord::new(0.1, 0.2, 0.3);
    let tensor = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
    let interp = [[9.0, 8.0, 7.0], [6.0, 5.0, 4.0], [3.0, 2.0, 1.0]];
    tick.replay_state = Some(RowExt {
        version: 1,
        group_mask: 1 << 1,
        ball: Some(BallExt {
            inv_inertia_world: Some(tensor),
            interp_basis: Some(interp),
            activation_state: Some(2),
            deactivation_time_s: Some(1.25),
            linear_sleeping_threshold: Some(0.35),
            angular_sleeping_threshold: Some(0.85),
            ..BallExt::default()
        }),
        ..RowExt::default()
    });

    backend.reset(&tick);
    let before = *backend.arena.get_ball_state();
    backend.restore_body_state(&tick);
    let after = *backend.arena.get_ball_state();
    assert_eq!(after.phys.pos, before.phys.pos);
    assert_eq!(after.phys.rot_mat, before.phys.rot_mat);
    assert_eq!(after.phys.vel, before.phys.vel);
    assert_eq!(after.phys.ang_vel, before.phys.ang_vel);

    let body = backend
        .arena
        .body_state(backend.arena.ball_body_id())
        .unwrap();
    assert_eq!(after.activation_state, ActivationState::Sleeping);
    assert_eq!(after.deactivation_time, 1.25);
    assert_eq!(body.linear_sleeping_threshold, 0.35);
    assert_eq!(body.angular_sleeping_threshold, 0.85);
    for (index, row) in tensor.iter().enumerate() {
        assert_eq!(
            body.inv_inertia_tensor_world.row(index),
            Vec3A::from_array(*row),
            "tensor row {index}"
        );
    }
    for (index, row) in interp.iter().enumerate() {
        assert_eq!(
            body.interp_world_basis.row(index),
            Vec3A::from_array(*row),
            "interp row {index}"
        );
    }
    assert!(backend.restore_counts.ball_sleep_restored);
}

#[test]
fn v12_ball_absent_fields_keep_default_components() {
    init();
    let mut backend = V3Backend::new();
    backend.ensure_all_cars(1);
    let mut tick = restore_test_tick(1, Vec3A::ZERO, Vec3A::ZERO);
    tick.replay_state = Some(RowExt {
        version: 1,
        group_mask: 1 << 1,
        ball: Some(BallExt {
            inv_inertia_world: Some([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]]),
            ..BallExt::default()
        }),
        ..RowExt::default()
    });

    backend.set_state(&tick);
    let before = backend
        .arena
        .body_state(backend.arena.ball_body_id())
        .unwrap();
    let state_before = *backend.arena.get_ball_state();
    backend.restore_body_state(&tick);
    let after = backend
        .arena
        .body_state(backend.arena.ball_body_id())
        .unwrap();
    assert_ne!(
        after.inv_inertia_tensor_world,
        before.inv_inertia_tensor_world
    );
    assert_eq!(after.interp_world_basis, before.interp_world_basis);
    assert_eq!(
        backend.arena.get_ball_state().activation_state,
        state_before.activation_state
    );
    assert_eq!(
        backend.arena.get_ball_state().deactivation_time,
        state_before.deactivation_time
    );
    assert_eq!(
        after.linear_sleeping_threshold,
        before.linear_sleeping_threshold
    );
    assert_eq!(
        after.angular_sleeping_threshold,
        before.angular_sleeping_threshold
    );
}

#[test]
fn nonalternating_teams_follow_frozen_slots_and_rebuild() {
    init();
    let mut backend = V3Backend::new();
    backend.set_bodies(&[0; 4]);
    backend.set_teams(&[Team::Blue, Team::Orange, Team::Orange, Team::Blue]);
    backend.ensure_all_cars(4);
    let mut planted = *backend.arena.get_car_state(backend.car_ids[0].unwrap());
    planted.phys.pos = Vec3A::new(1234.0, 567.0, 89.0);
    backend
        .arena
        .set_car_state_full(backend.car_ids[0].unwrap(), planted);

    backend.set_teams(&[Team::Orange, Team::Blue, Team::Blue, Team::Orange]);
    backend.ensure_all_cars(4);
    for (slot, car_id) in backend.car_ids.iter().enumerate() {
        let car_id = car_id.unwrap();
        assert_eq!(
            backend.arena.get_car_info(car_id).team,
            backend.team_for_slot(slot),
            "slot {slot}"
        );
    }
    assert_ne!(
        backend
            .arena
            .get_car_state(backend.car_ids[0].unwrap())
            .phys
            .pos,
        Vec3A::new(1234.0, 567.0, 89.0),
        "team change must rebuild the arena"
    );
}

#[test]
fn daizen_teams_follow_sparse_frozen_slots() {
    init();
    let mut backend = V3Backend::new();
    backend.set_bodies(&[2; 4]);
    backend.set_teams(&[Team::Blue, Team::Orange, Team::Orange, Team::Blue]);
    backend.ensure_cars(&[true, false, true, true]);
    assert_eq!(backend.car_ids, vec![Some(0), None, Some(1), Some(2)]);
    let teams: Vec<Team> = backend
        .car_ids
        .iter()
        .flatten()
        .map(|&car_id| backend.arena.get_car_info(car_id).team)
        .collect();
    assert_eq!(teams, vec![Team::Blue, Team::Orange, Team::Blue]);
}
