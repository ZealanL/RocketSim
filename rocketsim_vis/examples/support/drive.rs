use glam::Vec3A;
use rocketsim::{Arena, CarControls};
use rocketsim_vis::{ArenaVisExt, mutators::Mutators, proto::MUT_KNOWN_MASK};

/// Applies viewer controls and actions before each arena tick.
pub struct Driver {
    default_car: usize,
    previous_target: usize,
    mutators: u8,
}

impl Driver {
    pub fn new(default_car: usize) -> Self {
        Self {
            default_car,
            previous_target: default_car,
            mutators: 0,
        }
    }

    /// Apply the latest viewer state and advance the arena by one tick.
    pub fn tick(&mut self, arena: &mut Arena) {
        let message = arena.latest_vis_controls();
        let target = message.map_or(self.default_car, |message| {
            select_target(
                message.car_index as usize,
                arena.num_cars(),
                self.default_car,
            )
        });
        if self.previous_target != target {
            arena.set_car_controls(self.previous_target, CarControls::default());
            self.previous_target = target;
        }
        // Keep mutators on link loss. Driving inputs still become neutral.
        self.mutators = retain_mutators(self.mutators, message.map(|message| message.mutators));
        let controls = message.map_or_else(CarControls::default, |message| message.controls);
        let actions = arena.take_vis_actions();
        if actions & 1 != 0 {
            arena.reset_to_random_kickoff(None);
        }
        if actions & 2 != 0 {
            let car = arena.get_car_state(target);
            let pos = car.phys.pos + Vec3A::Z * 150.0;
            let vel = car.phys.vel;
            arena.edit_ball_state(|ball| {
                ball.phys.pos = pos;
                ball.phys.vel = vel;
            });
        }
        if actions & 4 != 0 {
            arena.edit_ball_state(|ball| ball.phys.vel += Vec3A::Z * 1000.0);
        }
        Mutators::from_mask(self.mutators).apply_pre_tick(arena, target);
        arena.set_car_controls(target, controls);
        arena.step_tick();
    }
}

fn select_target(car_index: usize, num_cars: usize, default_car: usize) -> usize {
    if car_index < num_cars {
        car_index
    } else {
        default_car
    }
}

fn retain_mutators(current: u8, update: Option<u8>) -> u8 {
    update.map_or(current, |mask| mask & MUT_KNOWN_MASK)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn invalid_target_uses_default_car() {
        assert_eq!(select_target(1, 2, 0), 1);
        assert_eq!(select_target(2, 2, 0), 0);
    }

    #[test]
    fn mutators_are_masked_and_retained_without_an_update() {
        let mask = retain_mutators(0, Some(MUT_KNOWN_MASK | 0x80));
        assert_eq!(mask, MUT_KNOWN_MASK);
        assert_eq!(retain_mutators(mask, None), mask);
    }
}
