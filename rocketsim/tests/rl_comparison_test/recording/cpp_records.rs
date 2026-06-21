#![allow(dead_code)]

use std::ops::{Index, IndexMut};

use glam::{Mat3A, Vec3A};
use rocketsim::{CarControls, CarState, PhysState};
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
impl Into<Vec3A> for VecRecord {
    fn into(self) -> Vec3A {
        Vec3A::new(self.x, self.y, self.z)
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
impl Into<CarControls> for ControlsRecord {
    fn into(self) -> CarControls {
        CarControls {
            throttle: self.throttle,
            steer: self.steer,
            pitch: self.pitch,
            yaw: self.yaw,
            roll: self.roll,
            jump: self.jump,
            boost: self.boost,
            handbrake: self.handbrake,
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct PhysRecord {
    pub physics_frame: u32,

    pub pos: VecRecord,
    pub rot: Mat3Record,
    pub lin_vel: VecRecord,
    pub ang_vel: VecRecord,
}
impl Into<PhysState> for PhysRecord {
    fn into(self) -> PhysState {
        // Verify rotation matrix is sane
        {
            for i in 0..3 {
                let c_len = self.rot.rows[i].length();
                assert!((1.0 - c_len).abs() < 1e-6);

                // Dirs should be 90 degrees from other row dirs
                assert!(
                    Vec3A::dot(self.rot.rows[i].into(), self.rot.rows[(i + 1) % 3].into()).abs()
                        < 1e-6
                );
            }
        }

        PhysState {
            pos: self.pos.into(),
            vel: self.lin_vel.into(),
            ang_vel: self.ang_vel.into(),
            rot_mat: Mat3A::from_cols(
                self.rot.column(0).into(),
                self.rot.column(1).into(),
                self.rot.column(2).into(),
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
    pub double_jumped_or_flipped: bool,

    pub boost_amount: f32,

    pub prev_controls: ControlsRecord,

    pub wheels: [WheelRecord; 4],
}
impl Into<CarState> for CarRecord {
    fn into(self) -> CarState {
        let mut result = CarState::default();
        result.phys = self.phys.into();

        result.boost = self.boost_amount;
        result.controls = self.prev_controls.into();
        result.is_on_ground = self.is_on_ground;
        result.is_jumping = self.is_jumping;
        result.is_flipping = self.is_flipping;
        result.jump_time = self.jump_time;
        result.flip_time = self.flip_time;

        result
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct TickRecord {
    pub car_record: CarRecord,
    pub ball_record: PhysRecord,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RecordingInfo {
    pub hitbox_rel_min_bt: VecRecord,
    pub hitbox_rel_max_bt: VecRecord,
}
