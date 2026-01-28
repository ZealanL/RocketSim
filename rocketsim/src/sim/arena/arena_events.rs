use glam::Vec3A;

#[derive(Debug, Copy, Clone)]
pub struct BallHitWorldEvent {
    pub contact_point: Vec3A,
    pub contact_normal: Vec3A,
}

#[derive(Debug, Copy, Clone)]
pub struct CarHitBallEvent {
    pub car_idx: usize,
    pub contact_point: Vec3A,
    pub extra_hit_vel: Vec3A,
}

#[derive(Debug, Copy, Clone)]
pub struct CarHitCarEvent {
    pub bumper_car_idx: usize,
    pub victim_car_idx: usize,
    pub contact_point: Vec3A,
    pub is_demo: bool,
}

#[derive(Debug, Copy, Clone)]
pub struct CarHitWorldEvent {
    pub car_idx: usize,
    pub contact_point: Vec3A,
    pub contact_normal: Vec3A,
}

#[derive(Debug, Copy, Clone)]
pub enum ArenaEvent {
    BallHitWorld(BallHitWorldEvent),
    CarHitBall(CarHitBallEvent),
    CarHitCar(CarHitCarEvent),
    CarHitWorld(CarHitWorldEvent),
}

//////////////////////////////////////

#[derive(Debug, Clone)]
pub(crate) struct ArenaEventList {
    events: Vec<ArenaEvent>,
}

impl ArenaEventList {
    const STARTING_CAPACITY: usize = 12;
    pub fn new() -> ArenaEventList {
        ArenaEventList {
            events: Vec::with_capacity(Self::STARTING_CAPACITY),
        }
    }

    pub fn push(&mut self, event: ArenaEvent) {
        self.events.push(event);
    }

    pub fn events(&self) -> &[ArenaEvent] {
        &self.events
    }

    pub fn clear(&mut self) {
        self.events.clear();
    }
}
