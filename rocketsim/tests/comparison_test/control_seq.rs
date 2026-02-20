use rocketsim::CarControls;

#[derive(Clone, Debug)]
pub struct ControlSeq {
    car_controls: Vec<CarControls>,
}

impl ControlSeq {
    pub const fn new() -> Self {
        Self {
            car_controls: Vec::new(),
        }
    }

    pub fn new_single(car_controls: CarControls) -> Self {
        Self {
            car_controls: vec![car_controls],
        }
    }

    pub fn get_controls_at_tick(&self, tick: u64) -> CarControls {
        if self.car_controls.is_empty() {
            return CarControls::default();
        }

        if let Some(car_controls) = self.car_controls.get(tick as usize) {
            *car_controls
        } else {
            *self.car_controls.last().unwrap()
        }
    }

    pub fn add(mut self, controls: CarControls, duration: u64) -> Self {
        assert!(duration > 0);
        for _ in 0..duration {
            self.car_controls.push(controls);
        }
        self
    }
}
