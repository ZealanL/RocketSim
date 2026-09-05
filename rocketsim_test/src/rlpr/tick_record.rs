use super::{CarRecord, PhysRecord};

#[derive(Clone)]
pub struct TickRecord {
    pub car_records: Vec<CarRecord>,
    pub ball_record: PhysRecord,
}
