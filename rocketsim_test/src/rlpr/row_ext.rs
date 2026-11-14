//! RLPR v12 row extension reader.
//!
//! A v12 row is the unchanged v11 row followed by one length-bounded
//! extension block. The extension has a group mask and a field mask for each
//! record. A field is present only when its mask bit is set. A missing field
//! is `None`, never a zero placeholder.
//!
//! The wire contract is `docs/replay-state-v12-contract.md` in the recorder
//! repository. All integers and floats are little-endian. Native values keep
//! their native units.

use std::io::{self, ErrorKind};

use super::{
    collision_snapshot::{CollisionSnapshot, read_collision_ext},
    data_reader::DataReader,
};

/// Extension layout version written by the recorder.
pub const EXT_VERSION: u16 = 1;
/// Version/reserved/group-mask header size.
pub const EXT_HEADER_SIZE: usize = 8;

/// Group bit for the per-car records.
pub const GROUP_CARS: u32 = 0;
/// Group bit for the shared ball record.
pub const GROUP_BALL: u32 = 1;
/// Group bit for the boost pad records.
pub const GROUP_PADS: u32 = 2;
/// Group bit for the global match state.
pub const GROUP_GLOBAL: u32 = 3;
/// Group bit for the collision snapshot.
pub const GROUP_COLLISION: u32 = 4;

/// Car field bit: world inverse inertia tensor, 9 `f32`.
pub const CAR_INV_INERTIA_WORLD: u32 = 1 << 0;
/// Car field bit: supersonic flag and native timer.
pub const CAR_SUPERSONIC: u32 = 1 << 1;
/// Car field bit: autoflip component state.
pub const CAR_AUTOFLIP: u32 = 1 << 2;
/// Car field bit: jump component state.
pub const CAR_JUMP: u32 = 1 << 3;
/// Car field bit: native vehicle update tag.
pub const CAR_UPDATE_TAG: u32 = 1 << 4;
/// Car field bit: four wheel contact records.
pub const CAR_WHEEL_CONTACT: u32 = 1 << 5;
/// Known car field bits.
pub const CAR_KNOWN_FIELD_MASK: u32 = 0x3f;

/// Ball field bit: world inverse inertia tensor, 9 `f32`.
pub const BALL_INV_INERTIA_WORLD: u32 = 1 << 0;
/// Ball field bit: interpolation world transform basis, 9 `f32`.
pub const BALL_INTERP_BASIS: u32 = 1 << 1;
/// Ball field bit: Bullet sleep and deactivation state.
pub const BALL_SLEEP: u32 = 1 << 2;
/// Known ball field bits.
pub const BALL_KNOWN_FIELD_MASK: u32 = 0x07;

/// Pad field bit: pose in Unreal units and boost amount.
pub const PAD_POSE_BOOST: u32 = 1 << 0;
/// Pad field bit: raw pickup history. The first byte is the raw
/// `PickupData2.PickedUp` counter, not a boolean. The layout is fixed for
/// unreleased v12.
pub const PAD_PICKUP_HISTORY: u32 = 1 << 1;
/// Pad field bit: native touching flag.
pub const PAD_TOUCH: u32 = 1 << 2;
/// Pad field bit: recorded instigator car slot. `-1` means null or not a
/// recorded car; frozen rows use `0..=5`. The layout is fixed for unreleased
/// v12.
pub const PAD_INSTIGATOR: u32 = 1 << 3;
/// Known pad field bits.
pub const PAD_KNOWN_FIELD_MASK: u32 = 0x0f;

/// Global field bit: kickoff and countdown state.
pub const GLOBAL_KICKOFF: u32 = 1 << 0;
/// Known global field bits.
pub const GLOBAL_KNOWN_FIELD_MASK: u32 = 0x01;

/// Maximum number of boost pads in one row.
pub const MAX_PADS: usize = 64;

/// One native wheel contact record.
///
/// All four fields are present when [`CAR_WHEEL_CONTACT`] is set.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WheelContactExt {
    /// Native `GetContact()` contact flag.
    pub has_contact: bool,
    /// Native contact against world geometry flag.
    pub has_contact_world_geometry: bool,
    /// Native `GetbHadContact()` flag.
    pub had_contact: bool,
    /// Raw native contact change time. The unit is not proven.
    pub has_contact_change_time: f32,
}

/// Native per-car hidden state.
///
/// Each field is `Some` only when its mask bit is set in [`Self::field_mask`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CarExt {
    /// Known car field bits that were present in the row.
    pub field_mask: u32,
    /// Native Bullet world inverse inertia tensor. Rows are `m[0]`, `m[1]`,
    /// `m[2]`. The values keep native Bullet units.
    pub inv_inertia_world: Option<[[f32; 3]; 3]>,
    /// Native supersonic flag.
    pub b_supersonic: Option<bool>,
    /// Raw native `TimeBelowSupersonicSpeed` value.
    pub time_below_supersonic_s: Option<f32>,
    /// Native `FlipCar` active flag.
    pub autoflip_active: Option<bool>,
    /// Raw native `bFlipRight` property.
    pub autoflip_right: Option<bool>,
    /// Raw native autoflip activity clock.
    pub autoflip_activity_time_s: Option<f32>,
    /// Raw native autoflip active clock.
    pub autoflip_active_time_s: Option<f32>,
    /// Raw native autoflip inactive clock.
    pub autoflip_inactive_time_s: Option<f32>,
    /// Native jump active flag.
    pub jump_active: Option<bool>,
    /// Raw native jump activity clock.
    pub jump_activity_time_s: Option<f32>,
    /// Raw native jump inactive clock.
    pub jump_inactive_time_s: Option<f32>,
    /// Raw native `VehicleUpdateTag`.
    pub update_tag: Option<i32>,
    /// Native wheel contact records in wheel order 0 to 3.
    pub wheel_contacts: Option<[WheelContactExt; 4]>,
}

/// Native shared ball hidden state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BallExt {
    /// Known ball field bits that were present in the row.
    pub field_mask: u32,
    /// Native Bullet world inverse inertia tensor. Rows are `m[0]`, `m[1]`,
    /// `m[2]`. The values keep native Bullet units.
    pub inv_inertia_world: Option<[[f32; 3]; 3]>,
    /// Native `m_interpolationWorldTransform` basis. Rows are `m[0]`, `m[1]`,
    /// `m[2]`.
    pub interp_basis: Option<[[f32; 3]; 3]>,
    /// Native Bullet activation state: 1 active, 2 sleeping, 3 wants
    /// deactivation.
    pub activation_state: Option<u32>,
    /// Native Bullet deactivation timer. The value keeps native units.
    pub deactivation_time_s: Option<f32>,
    /// Native Bullet linear sleeping threshold.
    pub linear_sleeping_threshold: Option<f32>,
    /// Native Bullet angular sleeping threshold.
    pub angular_sleeping_threshold: Option<f32>,
}

/// Native boost pad state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PadExt {
    /// Known pad field bits that were present in the row.
    pub field_mask: u32,
    /// Native actor position in Unreal units.
    pub pos: Option<[f32; 3]>,
    /// Native boost amount.
    pub boost_amount: Option<f32>,
    /// Native boost pad type.
    pub boost_type: Option<u8>,
    /// Raw native `PickupData2.PickedUp` value. It is a counter in `0..=255`,
    /// not a boolean.
    pub pickup_counter: Option<u8>,
    /// Native `bNoPickup` flag.
    pub no_pickup: Option<bool>,
    /// Native respawn delay configuration in seconds. This is not the live
    /// cooldown.
    pub respawn_delay_s: Option<f32>,
    /// Recorded instigator car slot. `-1` means null or not a recorded car.
    /// Frozen rows use `0..=5`.
    pub instigator_slot: Option<i8>,
    /// Native `IsTouchingAVehicle()` result.
    pub is_touching_vehicle: Option<bool>,
}

/// Native global match state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GlobalExt {
    /// Known global field bits that were present in the row.
    pub field_mask: u32,
    /// Native `bBallHasBeenHit` flag.
    pub ball_has_been_hit: Option<bool>,
    /// Native round active flag.
    pub round_active: Option<bool>,
    /// Native countdown seconds value.
    pub seconds_remaining_countdown: Option<i32>,
    /// Native countdown time value.
    pub countdown_time: Option<i32>,
}

/// One RLPR v12 row extension.
///
/// `cars` holds one entry for each row car slot when the car group is
/// present. An entry with a zero `field_mask` carries no fields. `ball` and
/// `global` are `None` when their group is absent or carries a zero field
/// mask. `pads` is empty when the pad group is absent.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RowExt {
    /// Extension layout version.
    pub version: u16,
    /// Group mask exactly as encoded in the row.
    pub group_mask: u32,
    /// Per-slot car extensions when the car group is present.
    pub cars: Vec<CarExt>,
    /// Ball extension when the ball group is present and not empty.
    pub ball: Option<BallExt>,
    /// Boost pad extensions in encoded order.
    pub pads: Vec<PadExt>,
    /// Global match extension when the global group is present and not empty.
    pub global: Option<GlobalExt>,
    /// Collision snapshot when the group 4 schema version is known.
    pub collision_snapshot: Option<CollisionSnapshot>,
}

fn invalid_data(message: impl Into<String>) -> io::Error {
    io::Error::new(ErrorKind::InvalidData, message.into())
}

fn read_bool(reader: &mut DataReader<'_>, field: &str) -> io::Result<bool> {
    let value = reader.read_u8()?;
    match value {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(invalid_data(format!(
            "RLPR v12 {field} byte must be 0 or 1, got {value}"
        ))),
    }
}

fn read_finite_f32(reader: &mut DataReader<'_>, field: &str) -> io::Result<f32> {
    let value = reader.read_f32()?;
    if !value.is_finite() {
        return Err(invalid_data(format!(
            "RLPR v12 {field} must be finite, got {value}"
        )));
    }
    Ok(value)
}

fn read_mat3(reader: &mut DataReader<'_>, field: &str) -> io::Result<[[f32; 3]; 3]> {
    let mut matrix = [[0.0f32; 3]; 3];
    for row in &mut matrix {
        for value in row {
            *value = read_finite_f32(reader, field)?;
        }
    }
    Ok(matrix)
}

fn require_consumed(reader: &mut DataReader<'_>, context: &str) -> io::Result<()> {
    let remaining = reader.num_bytes_left();
    if remaining != 0 {
        return Err(invalid_data(format!(
            "RLPR v12 {context} size does not match its fields ({remaining} trailing bytes)"
        )));
    }
    Ok(())
}

fn read_car_ext(bytes: &[u8], slot: usize) -> io::Result<CarExt> {
    let mut reader = DataReader::new(bytes);
    let field_mask = reader.read_u32()?;
    if field_mask & !CAR_KNOWN_FIELD_MASK != 0 {
        return Err(invalid_data(format!(
            "RLPR v12 car {slot} field mask has unknown bits: {field_mask:#x}"
        )));
    }

    let mut ext = CarExt {
        field_mask,
        ..CarExt::default()
    };

    if field_mask & CAR_INV_INERTIA_WORLD != 0 {
        ext.inv_inertia_world = Some(read_mat3(&mut reader, "car inv_inertia_world")?);
    }
    if field_mask & CAR_SUPERSONIC != 0 {
        ext.b_supersonic = Some(read_bool(&mut reader, "car b_supersonic")?);
        ext.time_below_supersonic_s =
            Some(read_finite_f32(&mut reader, "car time_below_supersonic_s")?);
    }
    if field_mask & CAR_AUTOFLIP != 0 {
        ext.autoflip_active = Some(read_bool(&mut reader, "car autoflip_active")?);
        ext.autoflip_right = Some(read_bool(&mut reader, "car autoflip_right")?);
        ext.autoflip_activity_time_s = Some(read_finite_f32(
            &mut reader,
            "car autoflip_activity_time_s",
        )?);
        ext.autoflip_active_time_s =
            Some(read_finite_f32(&mut reader, "car autoflip_active_time_s")?);
        ext.autoflip_inactive_time_s = Some(read_finite_f32(
            &mut reader,
            "car autoflip_inactive_time_s",
        )?);
    }
    if field_mask & CAR_JUMP != 0 {
        ext.jump_active = Some(read_bool(&mut reader, "car jump_active")?);
        ext.jump_activity_time_s = Some(read_finite_f32(&mut reader, "car jump_activity_time_s")?);
        ext.jump_inactive_time_s = Some(read_finite_f32(&mut reader, "car jump_inactive_time_s")?);
    }
    if field_mask & CAR_UPDATE_TAG != 0 {
        ext.update_tag = Some(reader.read_i32()?);
    }
    if field_mask & CAR_WHEEL_CONTACT != 0 {
        let mut wheels = [WheelContactExt::default(); 4];
        for wheel in &mut wheels {
            wheel.has_contact = read_bool(&mut reader, "wheel has_contact")?;
            wheel.has_contact_world_geometry =
                read_bool(&mut reader, "wheel has_contact_world_geometry")?;
            wheel.had_contact = read_bool(&mut reader, "wheel had_contact")?;
            wheel.has_contact_change_time =
                read_finite_f32(&mut reader, "wheel has_contact_change_time")?;
        }
        ext.wheel_contacts = Some(wheels);
    }

    require_consumed(&mut reader, "car extension")?;
    Ok(ext)
}

fn read_ball_ext(bytes: &[u8]) -> io::Result<Option<BallExt>> {
    let mut reader = DataReader::new(bytes);
    let field_mask = reader.read_u32()?;
    if field_mask & !BALL_KNOWN_FIELD_MASK != 0 {
        return Err(invalid_data(format!(
            "RLPR v12 ball field mask has unknown bits: {field_mask:#x}"
        )));
    }
    if field_mask == 0 {
        require_consumed(&mut reader, "ball extension")?;
        return Ok(None);
    }

    let mut ext = BallExt {
        field_mask,
        ..BallExt::default()
    };

    if field_mask & BALL_INV_INERTIA_WORLD != 0 {
        ext.inv_inertia_world = Some(read_mat3(&mut reader, "ball inv_inertia_world")?);
    }
    if field_mask & BALL_INTERP_BASIS != 0 {
        ext.interp_basis = Some(read_mat3(&mut reader, "ball interp_basis")?);
    }
    if field_mask & BALL_SLEEP != 0 {
        let activation_state = reader.read_u32()?;
        if !(1..=3).contains(&activation_state) {
            return Err(invalid_data(format!(
                "RLPR v12 ball activation_state must be 1, 2, or 3, got {activation_state}"
            )));
        }
        ext.activation_state = Some(activation_state);
        ext.deactivation_time_s = Some(read_finite_f32(&mut reader, "ball deactivation_time_s")?);
        ext.linear_sleeping_threshold = Some(read_finite_f32(
            &mut reader,
            "ball linear_sleeping_threshold",
        )?);
        ext.angular_sleeping_threshold = Some(read_finite_f32(
            &mut reader,
            "ball angular_sleeping_threshold",
        )?);
    }

    require_consumed(&mut reader, "ball extension")?;
    Ok(Some(ext))
}

fn read_pad_ext(bytes: &[u8], index: usize) -> io::Result<PadExt> {
    let mut reader = DataReader::new(bytes);
    let field_mask = reader.read_u32()?;
    if field_mask & !PAD_KNOWN_FIELD_MASK != 0 {
        return Err(invalid_data(format!(
            "RLPR v12 pad {index} field mask has unknown bits: {field_mask:#x}"
        )));
    }

    let mut ext = PadExt {
        field_mask,
        ..PadExt::default()
    };

    if field_mask & PAD_POSE_BOOST != 0 {
        let x = read_finite_f32(&mut reader, "pad x")?;
        let y = read_finite_f32(&mut reader, "pad y")?;
        let z = read_finite_f32(&mut reader, "pad z")?;
        ext.pos = Some([x, y, z]);
        ext.boost_amount = Some(read_finite_f32(&mut reader, "pad boost_amount")?);
        ext.boost_type = Some(reader.read_u8()?);
    }
    if field_mask & PAD_PICKUP_HISTORY != 0 {
        ext.pickup_counter = Some(reader.read_u8()?);
        ext.no_pickup = Some(read_bool(&mut reader, "pad no_pickup")?);
        ext.respawn_delay_s = Some(read_finite_f32(&mut reader, "pad respawn_delay_s")?);
    }
    if field_mask & PAD_TOUCH != 0 {
        ext.is_touching_vehicle = Some(read_bool(&mut reader, "pad is_touching_vehicle")?);
    }
    if field_mask & PAD_INSTIGATOR != 0 {
        ext.instigator_slot = Some(reader.read_i8()?);
    }

    require_consumed(&mut reader, "pad extension")?;
    Ok(ext)
}

fn read_global_ext(bytes: &[u8]) -> io::Result<Option<GlobalExt>> {
    let mut reader = DataReader::new(bytes);
    let field_mask = reader.read_u32()?;
    if field_mask & !GLOBAL_KNOWN_FIELD_MASK != 0 {
        return Err(invalid_data(format!(
            "RLPR v12 global field mask has unknown bits: {field_mask:#x}"
        )));
    }
    if field_mask == 0 {
        require_consumed(&mut reader, "global extension")?;
        return Ok(None);
    }

    let mut ext = GlobalExt {
        field_mask,
        ..GlobalExt::default()
    };

    if field_mask & GLOBAL_KICKOFF != 0 {
        ext.ball_has_been_hit = Some(read_bool(&mut reader, "global ball_has_been_hit")?);
        ext.round_active = Some(read_bool(&mut reader, "global round_active")?);
        ext.seconds_remaining_countdown = Some(reader.read_i32()?);
        ext.countdown_time = Some(reader.read_i32()?);
    }

    require_consumed(&mut reader, "global extension")?;
    Ok(Some(ext))
}

/// Read one extension payload.
///
/// `bytes` starts at `ext_version`; the caller has already consumed the
/// leading `u32 ext_size` with
/// [`DataReader::read_sized_bytes`](super::data_reader::DataReader::read_sized_bytes).
/// `num_cars` is the row car count. A known group must consume its declared
/// size exactly. An unknown group is skipped by its encoded size.
pub(crate) fn read_row_ext(bytes: &[u8], num_cars: usize) -> io::Result<RowExt> {
    if bytes.len() < EXT_HEADER_SIZE {
        return Err(invalid_data(format!(
            "RLPR v12 extension is {} bytes, below the {EXT_HEADER_SIZE}-byte header",
            bytes.len()
        )));
    }

    let mut block = DataReader::new(bytes);
    let version = block.read_u16()?;
    let reserved = block.read_u16()?;
    let group_mask = block.read_u32()?;

    if version != EXT_VERSION {
        return Err(invalid_data(format!(
            "RLPR v12 extension version must be {EXT_VERSION}, got {version}"
        )));
    }
    if reserved != 0 {
        return Err(invalid_data(format!(
            "RLPR v12 extension reserved word must be 0, got {reserved}"
        )));
    }

    let mut cars = Vec::new();
    let mut ball = None;
    let mut pads = Vec::new();
    let mut global = None;
    let mut collision_snapshot = None;

    for bit in 0..32u32 {
        if group_mask & (1u32 << bit) == 0 {
            continue;
        }

        let group_id = block.read_u32()?;
        let group_bytes = block.read_sized_bytes()?;
        if group_id != bit {
            return Err(invalid_data(format!(
                "RLPR v12 group id {group_id} does not match mask bit {bit}"
            )));
        }

        let mut group = DataReader::new(group_bytes);
        match group_id {
            GROUP_CARS => {
                let count = group.read_u32()? as usize;
                if count != num_cars {
                    return Err(invalid_data(format!(
                        "RLPR v12 car group count {count} does not match row car count {num_cars}"
                    )));
                }
                let mut parsed = Vec::with_capacity(count);
                for slot in 0..count {
                    parsed.push(read_car_ext(group.read_sized_bytes()?, slot)?);
                }
                cars = parsed;
            }
            GROUP_BALL => {
                ball = read_ball_ext(group.read_sized_bytes()?)?;
            }
            GROUP_PADS => {
                let count = group.read_u32()? as usize;
                if count > MAX_PADS {
                    return Err(invalid_data(format!(
                        "RLPR v12 pad count {count} is above {MAX_PADS}"
                    )));
                }
                let mut parsed = Vec::with_capacity(count);
                for index in 0..count {
                    parsed.push(read_pad_ext(group.read_sized_bytes()?, index)?);
                }
                pads = parsed;
            }
            GROUP_GLOBAL => {
                global = read_global_ext(group.read_sized_bytes()?)?;
            }
            GROUP_COLLISION => {
                // Group 4 is a raw payload. It has no inner size prefix.
                collision_snapshot = read_collision_ext(group_bytes)?;
                continue;
            }
            _ => continue,
        }

        require_consumed(&mut group, "group")?;
    }

    require_consumed(&mut block, "extension")?;

    Ok(RowExt {
        version,
        group_mask,
        cars,
        ball,
        pads,
        global,
        collision_snapshot,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn push_u16(out: &mut Vec<u8>, value: u16) {
        out.extend_from_slice(&value.to_le_bytes());
    }

    fn push_u32(out: &mut Vec<u8>, value: u32) {
        out.extend_from_slice(&value.to_le_bytes());
    }

    fn push_f32(out: &mut Vec<u8>, value: f32) {
        out.extend_from_slice(&value.to_le_bytes());
    }

    fn push_f32x9(out: &mut Vec<u8>, values: [f32; 9]) {
        for value in values {
            push_f32(out, value);
        }
    }

    /// One size-prefixed record: `u32 size`, `u32 mask`, then fields.
    fn record(mask: u32, fields: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        push_u32(&mut out, (4 + fields.len()) as u32);
        push_u32(&mut out, mask);
        out.extend_from_slice(fields);
        out
    }

    fn car_group(records: &[Vec<u8>]) -> Vec<u8> {
        let mut out = Vec::new();
        push_u32(&mut out, records.len() as u32);
        for record in records {
            out.extend_from_slice(record);
        }
        out
    }

    fn pad_group(records: &[Vec<u8>]) -> Vec<u8> {
        let mut out = Vec::new();
        push_u32(&mut out, records.len() as u32);
        for record in records {
            out.extend_from_slice(record);
        }
        out
    }

    fn payload_with_mask(mask: u32, groups: &[(u32, Vec<u8>)]) -> Vec<u8> {
        let mut out = Vec::new();
        push_u16(&mut out, EXT_VERSION);
        push_u16(&mut out, 0);
        push_u32(&mut out, mask);
        for (group_id, group) in groups {
            push_u32(&mut out, *group_id);
            push_u32(&mut out, group.len() as u32);
            out.extend_from_slice(group);
        }
        out
    }

    fn payload(groups: &[(u32, Vec<u8>)]) -> Vec<u8> {
        let mask = groups
            .iter()
            .fold(0u32, |mask, (group_id, _)| mask | (1u32 << group_id));
        payload_with_mask(mask, groups)
    }

    fn header(version: u16, reserved: u16, mask: u32) -> Vec<u8> {
        let mut out = Vec::new();
        push_u16(&mut out, version);
        push_u16(&mut out, reserved);
        push_u32(&mut out, mask);
        out
    }

    fn assert_rejects(expected: &str, bytes: &[u8], num_cars: usize) {
        let err = read_row_ext(bytes, num_cars)
            .expect_err(&format!("expected an error containing {expected:?}"));
        assert!(
            err.to_string().contains(expected),
            "expected {expected:?}, got {err}"
        );
    }

    #[test]
    fn pad_pickup_history_is_a_raw_counter() {
        let mut fields = vec![255, 0];
        fields.extend_from_slice(&0.0f32.to_le_bytes());
        let bytes = payload(&[(
            GROUP_PADS,
            pad_group(&[record(PAD_PICKUP_HISTORY, &fields)]),
        )]);
        let ext = read_row_ext(&bytes, 0).unwrap();
        let pad = &ext.pads[0];
        assert_eq!(pad.pickup_counter, Some(255));
        assert_eq!(pad.no_pickup, Some(false));
        assert_eq!(pad.respawn_delay_s, Some(0.0));
        assert_eq!(pad.instigator_slot, None);

        let bytes = payload(&[(GROUP_PADS, pad_group(&[record(PAD_INSTIGATOR, &[0xff])]))]);
        let ext = read_row_ext(&bytes, 0).unwrap();
        assert_eq!(ext.pads[0].instigator_slot, Some(-1));

        let bytes = payload(&[(GROUP_PADS, pad_group(&[record(PAD_INSTIGATOR, &[3])]))]);
        let ext = read_row_ext(&bytes, 0).unwrap();
        assert_eq!(ext.pads[0].instigator_slot, Some(3));
    }

    #[test]
    fn unknown_collision_schema_is_skipped() {
        // Schema 2 is unknown. The group is length-bounded, so the reader
        // must skip the whole payload, including these extra bytes.
        let bytes = payload(&[(GROUP_COLLISION, vec![2, 0, 1, 2, 3])]);
        let ext = read_row_ext(&bytes, 0).unwrap();
        assert_eq!(ext.group_mask, 1 << GROUP_COLLISION);
        assert!(ext.collision_snapshot.is_none());
    }

    #[test]
    fn empty_extension_has_no_fields() {
        let ext = read_row_ext(&header(EXT_VERSION, 0, 0), 1).unwrap();
        assert_eq!(
            ext,
            RowExt {
                version: EXT_VERSION,
                group_mask: 0,
                cars: Vec::new(),
                ball: None,
                pads: Vec::new(),
                global: None,
                collision_snapshot: None,
            }
        );
    }

    #[test]
    fn missing_fields_stay_none() {
        let mut fields = Vec::new();
        push_u32(&mut fields, (-7i32) as u32);
        let bytes = payload(&[(GROUP_CARS, car_group(&[record(CAR_UPDATE_TAG, &fields)]))]);
        let ext = read_row_ext(&bytes, 1).unwrap();

        let car = &ext.cars[0];
        assert_eq!(car.update_tag, Some(-7));
        assert_eq!(car.field_mask, CAR_UPDATE_TAG);
        assert_eq!(car.inv_inertia_world, None);
        assert_eq!(car.b_supersonic, None);
        assert_eq!(car.time_below_supersonic_s, None);
        assert_eq!(car.autoflip_active, None);
        assert_eq!(car.jump_active, None);
        assert_eq!(car.wheel_contacts, None);
        assert_eq!(ext.ball, None);
        assert_eq!(ext.global, None);
    }

    #[test]
    fn zero_masks_yield_empty_records() {
        let bytes = payload(&[
            (GROUP_CARS, car_group(&[record(0, &[])])),
            (GROUP_BALL, record(0, &[])),
            (GROUP_PADS, pad_group(&[record(0, &[])])),
            (GROUP_GLOBAL, record(0, &[])),
        ]);
        let ext = read_row_ext(&bytes, 1).unwrap();
        assert_eq!(ext.cars, vec![CarExt::default()]);
        assert_eq!(ext.ball, None);
        assert_eq!(ext.pads, vec![PadExt::default()]);
        assert_eq!(ext.global, None);
    }

    #[test]
    fn skips_unknown_group_by_encoded_size() {
        let bytes = payload(&[(7, vec![1, 2, 3, 4])]);
        let ext = read_row_ext(&bytes, 1).unwrap();
        assert_eq!(ext.group_mask, 1 << 7);
        assert!(ext.cars.is_empty());
        assert_eq!(ext.ball, None);
        assert!(ext.pads.is_empty());
        assert_eq!(ext.global, None);
    }

    #[test]
    fn rejects_bad_header() {
        assert_rejects("below the", &[0u8; EXT_HEADER_SIZE - 1], 1);
        assert_rejects("extension version", &header(EXT_VERSION + 1, 0, 0), 1);
        assert_rejects("reserved word", &header(EXT_VERSION, 1, 0), 1);
        assert_rejects(
            "trailing bytes",
            &[&header(EXT_VERSION, 0, 0)[..], &[0]].concat(),
            1,
        );
        assert!(
            read_row_ext(&header(EXT_VERSION, 0, 1), 1).is_err(),
            "a set group bit without a group payload must be rejected"
        );
    }

    #[test]
    fn rejects_group_id_that_differs_from_mask_bit() {
        let bytes = payload_with_mask(1, &[(1, Vec::new())]);
        assert_rejects("does not match mask bit", &bytes, 1);
    }

    #[test]
    fn rejects_unknown_known_field_bits() {
        let cases = [
            (
                "car 0 field mask",
                payload(&[(GROUP_CARS, car_group(&[record(0x80, &[])]))]),
            ),
            (
                "ball field mask",
                payload(&[(GROUP_BALL, record(0x08, &[]))]),
            ),
            (
                "pad 0 field mask",
                payload(&[(GROUP_PADS, pad_group(&[record(0x10, &[])]))]),
            ),
            (
                "global field mask",
                payload(&[(GROUP_GLOBAL, record(0x02, &[]))]),
            ),
        ];
        for (expected, bytes) in cases {
            assert_rejects(expected, &bytes, 1);
        }
    }

    #[test]
    fn rejects_bad_counts() {
        let car_mismatch = payload(&[(GROUP_CARS, car_group(&[record(0, &[]), record(0, &[])]))]);
        assert_rejects("car group count", &car_mismatch, 1);

        let mut too_many_pads = Vec::new();
        push_u32(&mut too_many_pads, (MAX_PADS + 1) as u32);
        let pads = payload(&[(GROUP_PADS, too_many_pads)]);
        assert_rejects("pad count", &pads, 1);
    }

    #[test]
    fn rejects_bad_boolean_bytes() {
        let supersonic = [2, 0, 0, 0, 0];
        let autoflip = [2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let jump = [2, 0, 0, 0, 0, 0, 0, 0, 0];
        let mut wheel_contact = Vec::new();
        for _ in 0..4 {
            wheel_contact.extend_from_slice(&[2, 0, 1, 0, 0, 0, 0]);
        }

        let cases = [
            ("car b_supersonic", CAR_SUPERSONIC, supersonic.to_vec()),
            ("car autoflip_active", CAR_AUTOFLIP, autoflip.to_vec()),
            ("car jump_active", CAR_JUMP, jump.to_vec()),
            ("wheel has_contact", CAR_WHEEL_CONTACT, wheel_contact),
        ];
        for (expected, mask, fields) in cases {
            let bytes = payload(&[(GROUP_CARS, car_group(&[record(mask, &fields)]))]);
            assert_rejects(expected, &bytes, 1);
        }

        let cases = [
            (
                "pad no_pickup",
                payload(&[(
                    GROUP_PADS,
                    pad_group(&[record(PAD_PICKUP_HISTORY, &[0, 2, 0, 0, 0, 0])]),
                )]),
            ),
            (
                "pad is_touching_vehicle",
                payload(&[(GROUP_PADS, pad_group(&[record(PAD_TOUCH, &[2])]))]),
            ),
            (
                "global ball_has_been_hit",
                payload(&[(
                    GROUP_GLOBAL,
                    record(GLOBAL_KICKOFF, &[2, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
                )]),
            ),
            (
                "global round_active",
                payload(&[(
                    GROUP_GLOBAL,
                    record(GLOBAL_KICKOFF, &[0, 2, 0, 0, 0, 0, 0, 0, 0, 0]),
                )]),
            ),
        ];
        for (expected, bytes) in cases {
            assert_rejects(expected, &bytes, 1);
        }
    }

    #[test]
    fn rejects_unsupported_ball_activation_state() {
        for activation in [0u32, 4, 5, u32::MAX] {
            let mut fields = Vec::new();
            fields.extend_from_slice(&activation.to_le_bytes());
            fields.extend_from_slice(&0.0f32.to_le_bytes());
            fields.extend_from_slice(&0.0f32.to_le_bytes());
            fields.extend_from_slice(&0.0f32.to_le_bytes());
            let bytes = payload(&[(GROUP_BALL, record(BALL_SLEEP, &fields))]);
            assert_rejects("activation_state", &bytes, 0);
        }

        for activation in [1u32, 2, 3] {
            let mut fields = Vec::new();
            fields.extend_from_slice(&activation.to_le_bytes());
            fields.extend_from_slice(&0.0f32.to_le_bytes());
            fields.extend_from_slice(&0.0f32.to_le_bytes());
            fields.extend_from_slice(&0.0f32.to_le_bytes());
            let bytes = payload(&[(GROUP_BALL, record(BALL_SLEEP, &fields))]);
            let ext = read_row_ext(&bytes, 0).unwrap();
            assert_eq!(ext.ball.unwrap().activation_state, Some(activation));
        }
    }

    #[test]
    fn rejects_nonfinite_floats() {
        let mut inertia = Vec::new();
        push_f32x9(
            &mut inertia,
            [1.0, 2.0, f32::NAN, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
        );
        let mut supersonic = vec![1];
        push_f32(&mut supersonic, f32::INFINITY);
        let mut autoflip = vec![1, 1];
        push_f32(&mut autoflip, 0.1);
        push_f32(&mut autoflip, f32::NEG_INFINITY);
        push_f32(&mut autoflip, 0.3);
        let mut jump = vec![1];
        push_f32(&mut jump, 0.4);
        push_f32(&mut jump, f32::NAN);
        let mut wheel_contact = Vec::new();
        for index in 0..4 {
            wheel_contact.extend_from_slice(&[1, 0, 1]);
            push_f32(&mut wheel_contact, if index == 0 { f32::NAN } else { 0.6 });
        }

        let cases = [
            ("car inv_inertia_world", CAR_INV_INERTIA_WORLD, inertia),
            ("car time_below_supersonic_s", CAR_SUPERSONIC, supersonic),
            ("car autoflip_active_time_s", CAR_AUTOFLIP, autoflip),
            ("car jump_inactive_time_s", CAR_JUMP, jump),
            (
                "wheel has_contact_change_time",
                CAR_WHEEL_CONTACT,
                wheel_contact,
            ),
        ];
        for (expected, mask, fields) in cases {
            let bytes = payload(&[(GROUP_CARS, car_group(&[record(mask, &fields)]))]);
            assert_rejects(expected, &bytes, 1);
        }

        let mut ball_inertia = Vec::new();
        push_f32x9(
            &mut ball_inertia,
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, f32::NAN],
        );
        let mut ball_interp = Vec::new();
        push_f32x9(
            &mut ball_interp,
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, f32::INFINITY],
        );
        let mut ball_sleep = Vec::new();
        push_u32(&mut ball_sleep, 2);
        push_f32(&mut ball_sleep, f32::NAN);
        push_f32(&mut ball_sleep, 0.8);
        push_f32(&mut ball_sleep, 1.0);

        let cases = [
            (
                "ball inv_inertia_world",
                BALL_INV_INERTIA_WORLD,
                ball_inertia,
            ),
            ("ball interp_basis", BALL_INTERP_BASIS, ball_interp),
            ("ball deactivation_time_s", BALL_SLEEP, ball_sleep),
        ];
        for (expected, mask, fields) in cases {
            let bytes = payload(&[(GROUP_BALL, record(mask, &fields))]);
            assert_rejects(expected, &bytes, 1);
        }

        let mut pad_pose = Vec::new();
        push_f32(&mut pad_pose, 100.0);
        push_f32(&mut pad_pose, f32::NAN);
        push_f32(&mut pad_pose, 300.0);
        push_f32(&mut pad_pose, 12.0);
        pad_pose.push(1);
        let mut pad_pickup = vec![1, 0];
        push_f32(&mut pad_pickup, f32::NAN);
        let cases = [
            ("pad y", PAD_POSE_BOOST, pad_pose),
            ("pad respawn_delay_s", PAD_PICKUP_HISTORY, pad_pickup),
        ];
        for (expected, mask, fields) in cases {
            let bytes = payload(&[(GROUP_PADS, pad_group(&[record(mask, &fields)]))]);
            assert_rejects(expected, &bytes, 1);
        }
    }

    #[test]
    fn rejects_trailing_payload() {
        let mut car_fields = Vec::new();
        push_u32(&mut car_fields, 7);
        car_fields.push(0);
        let cases = [
            (
                "car extension",
                payload(&[(
                    GROUP_CARS,
                    car_group(&[record(CAR_UPDATE_TAG, &car_fields)]),
                )]),
            ),
            ("ball extension", payload(&[(GROUP_BALL, record(0, &[0]))])),
            (
                "pad extension",
                payload(&[(GROUP_PADS, pad_group(&[record(PAD_TOUCH, &[1, 0])]))]),
            ),
            (
                "global extension",
                payload(&[(
                    GROUP_GLOBAL,
                    record(GLOBAL_KICKOFF, &[1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
                )]),
            ),
            (
                "group",
                payload(&[(GROUP_CARS, {
                    let mut group = car_group(&[record(0, &[])]);
                    group.push(0);
                    group
                })]),
            ),
        ];
        for (expected, bytes) in cases {
            assert_rejects(expected, &bytes, 1);
        }
    }
}
