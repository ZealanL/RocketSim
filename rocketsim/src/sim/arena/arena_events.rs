use glam::Vec3A;

/// Ball touched a static surface this tick (wall, floor, ceiling, goal mesh).
///
/// `contact_point` is in uu, `contact_normal` points off the surface.
#[derive(Debug, Copy, Clone)]
pub struct BallHitWorldEvent {
    pub contact_point: Vec3A,
    pub contact_normal: Vec3A,
}

/// A car touched the ball this tick.
///
/// `extra_hit_vel` (uu/s) is the additional velocity from the car's hit
/// impulse (on top of the physics-solver bounce). Zero when the contact
/// added no extra impulse (e.g. repeated-contact guard).
#[derive(Debug, Copy, Clone)]
pub struct CarHitBallEvent {
    pub car_idx: usize,
    pub contact_point: Vec3A,
    pub extra_hit_vel: Vec3A,
}

/// Two cars collided this tick (tested both directions).
///
/// `is_demo == true` means the victim was demolished; otherwise a bump
/// impulse was cached for the victim. See [`crate::MutatorConfig`] demo rules.
#[derive(Debug, Copy, Clone)]
pub struct CarHitCarEvent {
    pub bumper_car_idx: usize,
    pub victim_car_idx: usize,
    pub contact_point: Vec3A,
    pub is_demo: bool,
}

/// A car touched a static surface; also stored as
/// `CarState::world_contact_normal` for that tick.
#[derive(Debug, Copy, Clone)]
pub struct CarHitWorldEvent {
    pub car_idx: usize,
    pub contact_point: Vec3A,
    pub contact_normal: Vec3A,
}

/// A car collected a boost pad (`boost_pad_idx` matches
/// `Arena::get_boost_pad_config` order).
#[derive(Debug, Copy, Clone)]
pub struct CarPickupBoostEvent {
    pub car_idx: usize,
    pub boost_pad_idx: usize,
}

/// One tick's contact/pickup notifications.
///
/// Returned by [`crate::Arena::step_tick`] and
/// [`crate::Arena::get_last_step_events`]; valid only until the next tick.
#[derive(Debug, Copy, Clone)]
pub enum ArenaEvent {
    BallHitWorld(BallHitWorldEvent),
    CarHitBall(CarHitBallEvent),
    CarHitCar(CarHitCarEvent),
    CarHitWorld(CarHitWorldEvent),
    CarPickupBoost(CarPickupBoostEvent),
}

//////////////////////////////////////

#[derive(Debug, Clone)]
pub(crate) struct ArenaEventList {
    events: Vec<ArenaEvent>,
}

impl ArenaEventList {
    const STARTING_CAPACITY: usize = 12;

    pub fn new() -> Self {
        Self {
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
