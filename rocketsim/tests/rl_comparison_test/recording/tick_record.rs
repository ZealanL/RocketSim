use super::{CarRecord, PhysRecord};

pub struct TickRecord {
    pub car_records: Vec<CarRecord>,
    pub ball_record: PhysRecord,
}
