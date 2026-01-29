#![allow(dead_code)] // Don't warn for unused public stuff
use glam::{Mat3A, Vec3A};
use rocketsim::{BallState, CarControls, CarState, GameMode, PhysState, Team};
use rocketsim_rs::cxx::UniquePtr;

pub type OldArena = rocketsim_rs::sim::Arena;
pub type OldArenaPtr = UniquePtr<rocketsim_rs::sim::Arena>;

pub type OldArenaConfig = rocketsim_rs::sim::ArenaConfig;
pub type OldCarState = rocketsim_rs::sim::CarState;
pub type OldBallState = rocketsim_rs::sim::BallState;
pub type OldVec3 = rocketsim_rs::math::Vec3;
pub type OldRotMat = rocketsim_rs::math::RotMat;
pub type OldCarControls = rocketsim_rs::sim::CarControls;
pub type OldWorldContact = rocketsim_rs::sim::WorldContact;
pub type OldCarContact = rocketsim_rs::sim::CarContact;
pub type OldBallHitInfo = rocketsim_rs::sim::BallHitInfo;
pub type OldGameMode = rocketsim_rs::sim::GameMode;
pub type OldTeam = rocketsim_rs::sim::Team;
pub type OldCarConfig = rocketsim_rs::sim::CarConfig;

fn vec3_to_old(v: Vec3A) -> OldVec3 {
    OldVec3::new(v.x, v.y, v.z)
}

const fn vec3_to_new(v: OldVec3) -> Vec3A {
    Vec3A::new(v.x, v.y, v.z)
}

fn rot_mat_to_old(m: Mat3A) -> OldRotMat {
    OldRotMat {
        forward: vec3_to_old(m.col(0)),
        right: vec3_to_old(m.col(1)),
        up: vec3_to_old(m.col(2)),
    }
}

const fn rot_mat_to_new(m: OldRotMat) -> Mat3A {
    Mat3A::from_cols(
        vec3_to_new(m.forward),
        vec3_to_new(m.right),
        vec3_to_new(m.up),
    )
}

pub const fn conv_to_old_car_controls(c: CarControls) -> OldCarControls {
    OldCarControls {
        throttle: c.throttle,
        steer: c.steer,
        pitch: c.pitch,
        yaw: c.yaw,
        roll: c.roll,
        jump: c.jump,
        boost: c.boost,
        handbrake: c.handbrake,
    }
}

pub const fn conv_to_new_car_controls(c: OldCarControls) -> CarControls {
    CarControls {
        throttle: c.throttle,
        steer: c.steer,
        pitch: c.pitch,
        yaw: c.yaw,
        roll: c.roll,
        jump: c.jump,
        boost: c.boost,
        handbrake: c.handbrake,
    }
}

pub fn conv_to_old_car_state(state: &CarState) -> OldCarState {
    OldCarState {
        pos: vec3_to_old(state.phys.pos),
        rot_mat: rot_mat_to_old(state.phys.rot_mat),
        vel: vec3_to_old(state.phys.vel),
        ang_vel: vec3_to_old(state.phys.ang_vel),
        tick_count_since_update: 0, // Not a thing anymore
        is_on_ground: state.is_on_ground,
        wheels_with_contact: state.wheels_with_contact,
        has_jumped: state.has_jumped,
        has_double_jumped: state.has_double_jumped,
        has_flipped: state.has_flipped,
        flip_rel_torque: vec3_to_old(state.flip_rel_torque),
        jump_time: state.jump_time,
        flip_time: state.flip_time,
        is_flipping: state.is_flipping,
        is_jumping: state.is_jumping,
        air_time: state.air_time,
        air_time_since_jump: state.air_time_since_jump,
        boost: state.boost,
        time_since_boosted: state.time_since_boosted,
        is_boosting: state.is_boosting,
        boosting_time: state.boosting_time,
        is_supersonic: state.is_supersonic,
        supersonic_time: state.supersonic_time,
        handbrake_val: state.handbrake_val,
        is_auto_flipping: state.is_auto_flipping,
        auto_flip_timer: state.auto_flip_timer,
        auto_flip_torque_scale: state.auto_flip_torque_scale,
        world_contact: OldWorldContact {
            has_contact: state.world_contact_normal.is_some(),
            contact_normal: vec3_to_old(state.world_contact_normal.unwrap_or_default()),
        },
        car_contact: OldCarContact {
            other_car_id: 0,
            cooldown_timer: 0.0,
        },
        is_demoed: state.is_demoed,
        demo_respawn_timer: state.demo_respawn_timer,
        ball_hit_info: OldBallHitInfo::default(),
        last_controls: conv_to_old_car_controls(state.prev_controls),
    }
}

pub fn conv_to_new_car_state(old: &OldCarState, controls: CarControls) -> CarState {
    CarState {
        phys: PhysState {
            pos: vec3_to_new(old.pos),
            rot_mat: rot_mat_to_new(old.rot_mat),
            vel: vec3_to_new(old.vel),
            ang_vel: vec3_to_new(old.ang_vel),
        },
        controls,
        prev_controls: conv_to_new_car_controls(old.last_controls),
        is_on_ground: old.is_on_ground,
        wheels_with_contact: old.wheels_with_contact,
        has_jumped: old.has_jumped,
        has_double_jumped: old.has_double_jumped,
        has_flipped: old.has_flipped,
        flip_rel_torque: vec3_to_new(old.flip_rel_torque),
        jump_time: old.jump_time,
        flip_time: old.flip_time,
        is_flipping: old.is_flipping,
        is_jumping: old.is_jumping,
        air_time: old.air_time,
        air_time_since_jump: old.air_time_since_jump,
        boost: old.boost,
        time_since_boosted: old.time_since_boosted,
        is_boosting: old.is_boosting,
        boosting_time: old.boosting_time,
        is_supersonic: old.is_supersonic,
        supersonic_time: old.supersonic_time,
        handbrake_val: old.handbrake_val,
        is_auto_flipping: old.is_auto_flipping,
        auto_flip_timer: old.auto_flip_timer,
        auto_flip_torque_scale: old.auto_flip_torque_scale,
        world_contact_normal: if old.world_contact.has_contact {
            Some(vec3_to_new(old.world_contact.contact_normal))
        } else {
            None
        },
        is_demoed: old.is_demoed,
        demo_respawn_timer: old.demo_respawn_timer,

        bump_cooldown_timer: if old.car_contact.other_car_id != 0 {
            old.car_contact.cooldown_timer
        } else {
            0.0
        },
    }
}

pub fn conv_to_old_ball_state(ball_state: &BallState) -> OldBallState {
    OldBallState {
        pos: vec3_to_old(ball_state.phys.pos),
        rot_mat: rot_mat_to_old(ball_state.phys.rot_mat),
        vel: vec3_to_old(ball_state.phys.vel),
        ang_vel: vec3_to_old(ball_state.phys.ang_vel),

        tick_count_since_update: 0, // Not a thing anymore

        // TODO: Implement
        hs_info: Default::default(),
        ds_info: Default::default(),
    }
}

pub fn conv_to_new_ball_state(ball_state: &OldBallState) -> BallState {
    BallState {
        phys: PhysState {
            pos: vec3_to_new(ball_state.pos),
            rot_mat: rot_mat_to_new(ball_state.rot_mat),
            vel: vec3_to_new(ball_state.vel),
            ang_vel: vec3_to_new(ball_state.ang_vel),
        },

        // TODO: Implement
        hs_info: Default::default(),
        ds_info: Default::default(),

        // TODO: Implement
        last_extra_hit_tick: None,
    }
}

pub fn conv_to_old_game_mode(game_mode: GameMode) -> OldGameMode {
    match game_mode {
        GameMode::Soccar => OldGameMode::Soccar,
        GameMode::Hoops => OldGameMode::Hoops,
        GameMode::Snowday => OldGameMode::Snowday,

        // TODO: Not supported yet
        GameMode::Dropshot => todo!(),
        GameMode::Heatseeker => todo!(),
        GameMode::TheVoid => unimplemented!(),
    }
}

pub const fn conv_to_old_team(team: Team) -> OldTeam {
    match team {
        Team::Blue => OldTeam::Blue,
        Team::Orange => OldTeam::Orange,
    }
}
