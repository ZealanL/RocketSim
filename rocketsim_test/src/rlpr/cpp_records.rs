#![allow(dead_code)]

use std::ops::{Index, IndexMut};

use glam::{Mat3A, Vec3A};
use rocketsim::{CarControls, CarState, PhysState, consts::TICK_RATE};

const RL_BOOST_TO_ROCKETSIM_SCALE: f32 = 100.0;
#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct VecRecord {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
impl VecRecord {
    pub fn new(x: f32, y: f32, z: f32) -> VecRecord {
        VecRecord { x, y, z }
    }
    pub fn length(&self) -> f32 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }
}
impl From<VecRecord> for Vec3A {
    fn from(val: VecRecord) -> Self {
        Vec3A::new(val.x, val.y, val.z)
    }
}
impl Index<usize> for VecRecord {
    type Output = f32;
    fn index(&self, index: usize) -> &f32 {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => unreachable!(),
        }
    }
}
impl IndexMut<usize> for VecRecord {
    fn index_mut(&mut self, index: usize) -> &mut f32 {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => unreachable!(),
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Mat3Record {
    pub rows: [VecRecord; 3],
}
impl Mat3Record {
    pub fn column(&self, idx: usize) -> VecRecord {
        VecRecord::new(self.rows[0][idx], self.rows[1][idx], self.rows[2][idx])
    }
    pub fn forward(&self) -> VecRecord {
        self.column(0)
    }
    pub fn right(&self) -> VecRecord {
        self.column(1)
    }
    pub fn up(&self) -> VecRecord {
        self.column(2)
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct WheelRecord {
    pub susp_length: f32,
    pub susp_rel_vel: f32,

    pub has_contact: bool,
    pub contact_normal: VecRecord,

    pub steer_amount: f32,
    pub engine_force: f32,
    pub brake: f32,

    pub lat_friction: f32,
    pub long_friction: f32,
    pub extra_pushback: f32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ControlsRecord {
    pub throttle: f32,
    pub steer: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub jump: bool,
    pub boost: bool,
    pub handbrake: bool,
}
impl From<ControlsRecord> for CarControls {
    fn from(val: ControlsRecord) -> Self {
        CarControls {
            throttle: val.throttle,
            steer: val.steer,
            pitch: val.pitch,
            yaw: val.yaw,
            roll: val.roll,
            jump: val.jump,
            boost: val.boost,
            handbrake: val.handbrake,
        }
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum ImpulseRecordType {
    WheelsSuspension,
    WheelsFriction,
    StickyForce,
    Jump,
    DoubleJump,
    DodgeImpulse,
    DodgeTorque,
    AirControl,
    Boost,
}
#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ImpulseRecord {
    pub lin_impulse: VecRecord,
    pub ang_impulse: VecRecord,
    pub impulse_type: ImpulseRecordType,
    pub is_accum: bool,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct PhysRecord {
    pub physics_frame: u32,

    pub pos: VecRecord,
    pub rot: Mat3Record,
    pub lin_vel: VecRecord,
    pub ang_vel: VecRecord,

    pub has_world_contact: bool,
    pub world_contact_point: VecRecord,
    pub world_contact_normal: VecRecord,

    impulse_records_data: [ImpulseRecord; 8],
    num_impulse_records: u32,
}
impl PhysRecord {
    pub fn impulse_records(&self) -> &[ImpulseRecord] {
        &self.impulse_records_data[..self.num_impulse_records as usize]
    }
}
impl From<PhysRecord> for PhysState {
    fn from(phys_record: PhysRecord) -> Self {
        // Verify rotation matrix is sane
        {
            for i in 0..3 {
                let c_len = phys_record.rot.rows[i].length();
                assert!((1.0 - c_len).abs() < 1e-6);

                // Dirs should be 90 degrees from other row dirs
                assert!(
                    Vec3A::dot(
                        phys_record.rot.rows[i].into(),
                        phys_record.rot.rows[(i + 1) % 3].into()
                    )
                    .abs()
                        < 1e-6
                );
            }
        }

        PhysState {
            pos: phys_record.pos.into(),
            vel: phys_record.lin_vel.into(),
            ang_vel: phys_record.ang_vel.into(),
            rot_mat: Mat3A::from_cols(
                phys_record.rot.column(0).into(),
                phys_record.rot.column(1).into(),
                phys_record.rot.column(2).into(),
            ),
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CarRecord {
    pub phys: PhysRecord,

    pub is_on_ground: bool,
    pub is_jumping: bool,
    pub is_flipping: bool,
    pub jump_time: f32,
    pub flip_time: f32,
    pub has_jumped: bool,
    pub double_jumped_or_flipped: bool,
    pub has_flip: bool,
    pub flip_rel_torque: VecRecord,

    pub boost_amount: f32,

    pub is_touching_ball: bool,

    pub prev_controls: ControlsRecord,

    pub wheels: [WheelRecord; 4],
}
impl From<CarRecord> for CarState {
    fn from(phys_record: CarRecord) -> Self {
        Self {
            phys: phys_record.phys.into(),
            boost: phys_record.boost_amount * RL_BOOST_TO_ROCKETSIM_SCALE,
            controls: phys_record.prev_controls.into(),
            is_on_ground: phys_record.is_on_ground,
            wheels_with_contact: phys_record.wheels.map(|wheel| wheel.has_contact),
            is_jumping: phys_record.is_jumping,
            is_flipping: phys_record.is_flipping,
            flip_rel_torque: phys_record.flip_rel_torque.into(),
            // NOTE: must round, NOT truncate — RL's accumulated f32 lands
            // below the integer multiple on ~54% of recorded ticks
            // (e.g. 41.99999928), and truncating would drop a whole tick.
            jump_ticks: (phys_record.jump_time * TICK_RATE).round() as u32,
            flip_time: phys_record.flip_time,
            has_jumped: phys_record.has_jumped,
            ..Default::default()
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RecordingInfo {
    pub num_cars: u32,
    pub hitbox_rel_min_bt: VecRecord,
    pub hitbox_rel_max_bt: VecRecord,
}
