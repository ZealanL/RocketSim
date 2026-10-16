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

    /// Test-only helper: attach impulse records of the given types.
    #[cfg(test)]
    pub(crate) fn set_test_impulses(&mut self, types: &[ImpulseRecordType]) {
        let n = types.len().min(self.impulse_records_data.len());
        for (slot, ty) in self.impulse_records_data.iter_mut().zip(types.iter()) {
            *slot = ImpulseRecord {
                lin_impulse: VecRecord::new(0.0, 0.0, 0.0),
                ang_impulse: VecRecord::new(0.0, 0.0, 0.0),
                impulse_type: *ty,
                is_accum: true,
            };
        }
        self.num_impulse_records = n as u32;
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

    /// RLPR v6 car-car contact flag. False for v2-v5.
    pub is_touching_car: bool,
    /// Reserved padding. Keeps v7 `is_boosting` at offset 588. Always zero.
    pub _touch_pad: [u8; 3],
    /// RLPR v7 boost latch bit. False for v2-v6.
    pub is_boosting: bool,
    /// Reserved padding. Keeps v7 `boosting_time` at offset 592. Always zero.
    pub _boost_pad: [u8; 3],
    /// RLPR v7 time since boost armed. 0.0 for v2-v6.
    pub boosting_time: f32,
    /// RLPR v8 handbrake integrator value. 0.0 for v2-v7.
    pub handbrake_val: f32,
    /// RLPR v9 raw last ball touch frame (`Vehicle_TA.GetLastBallTouchFrame`
    /// verbatim, so negative values arrive as large u32s). Lets consumers
    /// compute exact touch recency instead of trusting the thresholded
    /// `is_touching_ball` bool. [`TOUCH_FRAME_UNKNOWN`](super::TOUCH_FRAME_UNKNOWN)
    /// for v2-v8: unknown, not frame 0.
    pub last_ball_touch_frame: u32,
}
impl From<CarRecord> for CarState {
    fn from(record: CarRecord) -> Self {
        let mut state = CarState::DEFAULT;
        state.phys = record.phys.into();
        state.boost = record.boost_amount * RL_BOOST_TO_ROCKETSIM_SCALE;
        state.controls = record.prev_controls.into();
        state.is_jumping = record.is_jumping;
        state.is_flipping = record.is_flipping;
        state.flip_rel_torque = record.flip_rel_torque.into();
        // Round instead of truncate. Recorded f32 values can sit just below
        // an integer tick and truncation would drop a whole tick.
        state.jump_ticks = (record.jump_time * TICK_RATE).round() as u32;
        state.flip_time = record.flip_time;
        state.has_jumped = record.has_jumped;

        state.set_contact_flags(
            record.is_on_ground,
            record.wheels.map(|wheel| wheel.has_contact),
        );
        state
    }
}

/// Copy ground and wheel caches from an RLPR car record snapshot.
///
/// Keep the live world-contact cache and all core/tuning state unchanged.
pub fn copy_recorded_contact_cache(live: &CarState, recorded: &CarState) -> CarState {
    let mut live_value = serde_value::to_value(live).expect("serialize live car state");
    let live_cache = value_map_field_mut(&mut live_value, "cache");
    set_value_field(
        live_cache,
        "is_on_ground",
        serde_value::Value::Bool(recorded.is_on_ground()),
    );
    set_value_field(
        live_cache,
        "wheels_with_contact",
        serde_value::to_value(recorded.wheels_with_contact())
            .expect("serialize recorded wheel contacts"),
    );
    live_value
        .deserialize_into()
        .expect("deserialize car state with recorded cache")
}

fn value_map_field_mut<'a>(
    value: &'a mut serde_value::Value,
    field: &str,
) -> &'a mut serde_value::Value {
    match value {
        serde_value::Value::Map(map) => map
            .get_mut(&serde_value::Value::String(field.to_owned()))
            .expect("serialized map field exists"),
        _ => panic!("serialized value is not a map"),
    }
}

fn set_value_field(map: &mut serde_value::Value, field: &str, value: serde_value::Value) {
    match map {
        serde_value::Value::Map(map) => {
            map.insert(serde_value::Value::String(field.to_owned()), value);
        }
        _ => panic!("serialized value is not a map"),
    }
}

#[cfg(test)]
mod car_cache_tests {
    use rocketsim::RaycastHitInfo;

    use super::*;

    #[test]
    fn cache_copy_changes_only_ground_and_wheel_contacts() {
        let mut live = CarState::DEFAULT;
        live.boost = 23.0;
        live.handbrake_val = 0.75;
        live.set_prev_controls(CarControls {
            jump: true,
            ..CarControls::DEFAULT
        });
        let mut live_value = serde_value::to_value(live).unwrap();
        set_value_field(
            value_map_field_mut(&mut live_value, "cache"),
            "world_contact_normal",
            serde_value::to_value(Some(Vec3A::Y)).unwrap(),
        );
        let live: CarState = live_value.clone().deserialize_into().unwrap();

        let hit = RaycastHitInfo {
            hit_point: Vec3A::new(1.25, 2.5, 0.0),
            hit_normal: Vec3A::Z,
            hit_fraction: f32::from_bits(0x3e12_3456),
            user_info: rocketsim::UserInfoType::Car,
        };
        let wheels = [Some(hit), None, Some(hit), Some(hit)];
        let wheel_value = serde_value::to_value(wheels).unwrap();
        let mut recorded = CarState::DEFAULT;
        recorded.boost = 99.0;
        recorded.handbrake_val = 0.25;
        let mut recorded_value = serde_value::to_value(recorded).unwrap();
        set_value_field(
            value_map_field_mut(&mut recorded_value, "cache"),
            "wheels_with_contact",
            wheel_value.clone(),
        );
        let recorded: CarState = recorded_value.deserialize_into().unwrap();

        let restored = copy_recorded_contact_cache(&live, &recorded);
        let expected_cache = value_map_field_mut(&mut live_value, "cache");
        set_value_field(
            expected_cache,
            "is_on_ground",
            serde_value::Value::Bool(true),
        );
        set_value_field(expected_cache, "wheels_with_contact", wheel_value);
        assert_eq!(serde_value::to_value(restored).unwrap(), live_value);
        assert_eq!(restored.wheels_with_contact(), &wheels);
        assert_eq!(restored.world_contact_normal(), Some(Vec3A::Y));
        assert!(restored.prev_controls().jump);
    }

    #[test]
    fn cache_copy_preserves_primary_and_tuning_float_bits() {
        let mut live = CarState::DEFAULT;
        live.pos.x = f32::from_bits(0x3f12_3456);
        live.vel.y = f32::from_bits(0xbf65_4321);
        live.handbrake_val = f32::from_bits(0x3e12_3456);
        live.boosting_time = f32::from_bits(0x3d65_4321);

        let mut recorded_value = serde_value::to_value(CarState::DEFAULT).unwrap();
        let cache = value_map_field_mut(&mut recorded_value, "cache");
        set_value_field(cache, "is_on_ground", serde_value::to_value(false).unwrap());
        let mut controls = serde_value::to_value(CarControls::DEFAULT).unwrap();
        set_value_field(&mut controls, "jump", serde_value::to_value(true).unwrap());
        set_value_field(cache, "prev_controls", controls);
        let recorded: CarState = recorded_value.deserialize_into().unwrap();
        let mut restored = copy_recorded_contact_cache(&live, &recorded);
        restored.set_prev_controls(*recorded.prev_controls());

        assert_eq!(restored.pos.x.to_bits(), live.pos.x.to_bits());
        assert_eq!(restored.vel.y.to_bits(), live.vel.y.to_bits());
        assert_eq!(
            restored.handbrake_val.to_bits(),
            live.handbrake_val.to_bits()
        );
        assert_eq!(
            restored.boosting_time.to_bits(),
            live.boosting_time.to_bits()
        );
        assert!(!restored.is_on_ground());
        assert!(restored.prev_controls().jump);
    }

    #[test]
    fn cache_helpers_preserve_non_finite_and_signed_zero_bits() {
        let mut live = CarState::DEFAULT;
        live.pos.x = f32::from_bits(0x7fc0_1234);
        live.vel.y = f32::INFINITY;
        live.ang_vel.z = f32::NEG_INFINITY;
        live.boost = -0.0;
        live.handbrake_val = f32::from_bits(0x7fc0_5678);
        live.supersonic_grace_timer = -0.0;
        live.time_since_boosted = f32::INFINITY;
        live.boosting_time = f32::NEG_INFINITY;
        live.air_time = f32::from_bits(0x7fc0_9abc);

        let mut recorded = CarState::DEFAULT;
        let mut recorded_value = serde_value::to_value(recorded).unwrap();
        set_value_field(
            value_map_field_mut(&mut recorded_value, "cache"),
            "is_on_ground",
            serde_value::to_value(false).unwrap(),
        );
        recorded = recorded_value.deserialize_into().unwrap();

        let mut restored = copy_recorded_contact_cache(&live, &recorded);
        for (actual, expected) in [
            (restored.pos.x, live.pos.x),
            (restored.vel.y, live.vel.y),
            (restored.ang_vel.z, live.ang_vel.z),
            (restored.boost, live.boost),
            (restored.handbrake_val, live.handbrake_val),
            (restored.supersonic_grace_timer, live.supersonic_grace_timer),
            (restored.time_since_boosted, live.time_since_boosted),
            (restored.boosting_time, live.boosting_time),
            (restored.air_time, live.air_time),
        ] {
            assert_eq!(actual.to_bits(), expected.to_bits());
        }
        assert!(!restored.is_on_ground());

        let mut controls = *recorded.prev_controls();
        controls.jump = true;
        restored.set_prev_controls(controls);
        assert!(restored.prev_controls().jump);
        assert_eq!(restored.pos.x.to_bits(), live.pos.x.to_bits());
        assert_eq!(
            restored.handbrake_val.to_bits(),
            live.handbrake_val.to_bits()
        );
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RecordingInfo {
    pub num_cars: u32,
    pub hitbox_rel_min_bt: VecRecord,
    pub hitbox_rel_max_bt: VecRecord,
}
