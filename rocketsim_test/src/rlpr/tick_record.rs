use super::{CarRecord, PhysRecord, ROW_FLAGS_BALL_AVAILABLE, ROW_FLAGS_NO_PHYSICS, RowExt};

#[derive(Clone, Debug, PartialEq)]
pub struct TickRecord {
    /// RLPR v11 plugin level tick. Zero for v2-v10.
    pub level_tick: u64,
    /// RLPR v11 row flags. `ROW_FLAGS_BALL_AVAILABLE` for v2-v10.
    pub row_flags: u32,
    /// RLPR v11 reserved word. Zero for v2-v10.
    pub reserved0: u32,
    pub car_records: Vec<CarRecord>,
    pub ball_record: PhysRecord,
    /// RLPR v12 native hidden state. `None` for v2-v11.
    ///
    /// Each field is present only when its own mask bit is set. A mask bit
    /// never implies availability from the recording version.
    pub replay_state: Option<RowExt>,
}

impl TickRecord {
    /// True when the shared ball carries real physics for this row.
    #[inline]
    pub fn ball_available(&self) -> bool {
        self.row_flags & ROW_FLAGS_BALL_AVAILABLE != 0
    }

    /// True when the row carries any physics at all.
    ///
    /// A no-physics row is a real timestamp, not a physical state. It must
    /// never be scored.
    #[inline]
    pub fn has_physics(&self) -> bool {
        self.row_flags & ROW_FLAGS_NO_PHYSICS == 0
    }

    /// True when one frozen car slot carries real physics.
    #[inline]
    pub fn car_available(&self, slot: usize) -> bool {
        self.car_records
            .get(slot)
            .is_some_and(|car| car.is_available())
    }

    /// Availability bit mask, one bit per frozen car slot.
    pub fn availability_mask(&self) -> u32 {
        self.car_records
            .iter()
            .enumerate()
            .fold(0u32, |mask, (slot, car)| {
                if car.is_available() {
                    mask | (1 << slot)
                } else {
                    mask
                }
            })
    }

    /// True when every frozen car slot is available.
    pub fn all_cars_available(&self) -> bool {
        self.car_records.iter().all(CarRecord::is_available)
    }
}
