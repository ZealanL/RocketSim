use std::{collections::HashMap, fmt::Debug};

use glam::Vec3A;
use rocketsim::{
    BallState, CarState, PhysState,
    consts::TICK_TIME,
    rlpr::{
        cpp_records::{CarRecord, PhysRecord},
        tick_record::TickRecord,
    },
};

#[derive(Copy, Clone, Debug)]
#[allow(dead_code)]
pub struct ComparisonData<T: Copy + Debug> {
    pub pred: T,
    pub real: T,

    pub raw_error: f32,
    pub rel_error: f32,
}
impl<T: Copy + Debug> ComparisonData<T> {
    pub fn new(pred: T, real: T, raw_error: f32, error_thresh: f32) -> Self {
        assert!(error_thresh > 0.0);
        Self {
            pred,
            real,
            raw_error,
            rel_error: raw_error / error_thresh,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Comparison {
    Float(ComparisonData<f32>),
    Bool(ComparisonData<bool>),
    Vec(ComparisonData<Vec3A>),
}
impl Comparison {
    pub fn new_float(pred: f32, real: f32, error_thresh: f32) -> Self {
        Comparison::Float(ComparisonData::new(
            pred,
            real,
            (pred - real).abs(),
            error_thresh,
        ))
    }

    pub fn new_bool(pred: bool, real: bool) -> Self {
        Comparison::Bool(ComparisonData::new(
            pred,
            real,
            (pred != real) as u8 as f32,
            1.0,
        ))
    }

    pub fn new_vec(pred: Vec3A, real: Vec3A, error_thresh: f32) -> Self {
        Comparison::Vec(ComparisonData::new(
            pred,
            real,
            (pred - real).length(),
            error_thresh,
        ))
    }

    pub fn rel_error(&self) -> f32 {
        use Comparison::*;
        match self {
            Float(d) => d.rel_error,
            Bool(d) => d.rel_error,
            Vec(d) => d.rel_error,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ComparisonSet {
    map: HashMap<String, Comparison>,
}
impl ComparisonSet {
    pub fn new() -> Self {
        Self {
            map: HashMap::new(),
        }
    }

    pub fn insert(&mut self, name: &str, comparison: Comparison) {
        let existing = self.map.insert(name.to_string(), comparison);
        assert!(existing.is_none());
    }

    pub fn append_with_prefix(&mut self, other: &ComparisonSet, prefix: &str) {
        for (k, v) in &other.map {
            self.insert(&format!("{prefix}{k}"), *v);
        }
    }

    pub fn map(&self) -> &HashMap<String, Comparison> {
        &self.map
    }

    pub fn calc_norm_error(&self) -> f32 {
        let square_sum: f32 = self.map.values().map(|v| v.rel_error().powi(2)).sum();
        square_sum.sqrt()
    }
}

fn compare_phys(phys_state: &PhysState, phys_record: &PhysRecord, comparisons: &mut ComparisonSet) {
    comparisons.insert(
        "pos",
        Comparison::new_vec(phys_state.pos, phys_record.pos.into(), 10.0),
    );
    comparisons.insert(
        "vel",
        Comparison::new_vec(phys_state.vel, phys_record.lin_vel.into(), 3.0),
    );
    comparisons.insert(
        "ang_vel",
        Comparison::new_vec(phys_state.ang_vel, phys_record.ang_vel.into(), 1.0),
    );

    comparisons.insert(
        "rot_forward",
        Comparison::new_vec(
            phys_state.get_forward_dir(),
            phys_record.rot.forward().into(),
            1.0,
        ),
    );
    comparisons.insert(
        "rot_up",
        Comparison::new_vec(phys_state.get_up_dir(), phys_record.rot.up().into(), 1.0),
    );
}

fn compare_car(car_state: &CarState, car_record: &CarRecord, comparisons: &mut ComparisonSet) {
    compare_phys(&car_state.phys, &car_record.phys, comparisons);

    comparisons.insert(
        "is_on_ground",
        Comparison::new_bool(car_state.is_on_ground, car_record.is_on_ground),
    );

    comparisons.insert(
        "is_jumping",
        Comparison::new_bool(car_state.is_jumping, car_record.is_jumping),
    );
    comparisons.insert(
        "has_jumped",
        Comparison::new_bool(car_state.has_jumped, car_record.has_jumped),
    );
    if car_state.has_jumped {
        comparisons.insert(
            "jump_time",
            Comparison::new_float(car_state.jump_time(), car_record.jump_time, TICK_TIME * 2.0),
        );
    }
    comparisons.insert(
        "is_flipping",
        Comparison::new_bool(car_state.is_flipping, car_record.is_flipping),
    );

    if car_state.is_flipping {
        comparisons.insert(
            "flip_time",
            Comparison::new_float(car_state.flip_time, car_record.flip_time, TICK_TIME * 2.0),
        );
        comparisons.insert(
            "flip_rel_torque",
            Comparison::new_vec(
                car_state.flip_rel_torque,
                car_record.flip_rel_torque.into(),
                0.05,
            ),
        );
    }
}

pub fn compare_states_to_tick(
    car_states: &[CarState],
    ball_state: &BallState,
    tick: &TickRecord,
) -> ComparisonSet {
    let mut car_comparisons_all: Vec<ComparisonSet> = Vec::new();
    for (car_state, car_record) in car_states.iter().zip(&tick.car_records) {
        let mut car_comparison_set = ComparisonSet::new();
        compare_car(car_state, car_record, &mut car_comparison_set);
        car_comparisons_all.push(car_comparison_set);
    }

    let mut ball_comparisons = ComparisonSet::new();
    compare_phys(&ball_state.phys, &tick.ball_record, &mut ball_comparisons);

    let mut all_comparisons = ComparisonSet::new();
    for (i, car_comparison_set) in car_comparisons_all.iter().enumerate() {
        all_comparisons.append_with_prefix(car_comparison_set, &format!("car_{i}_"));
    }
    all_comparisons.append_with_prefix(&ball_comparisons, "ball_");
    all_comparisons
}
