use std::collections::HashMap;

use rustc_hash::{FxBuildHasher, FxHashMap};

use crate::comparison_test::{TestResultState, state_compare::StateErrSet};

#[derive(Debug, Clone, Copy)]
pub struct ValueErrorStat {
    pub num_samples: usize,
    pub total: f32,
    pub max: f32,
}

impl Default for ValueErrorStat {
    fn default() -> Self {
        Self {
            num_samples: 0,
            total: 0.0,
            max: -f32::INFINITY,
        }
    }
}

impl ValueErrorStat {
    pub fn mean(&self) -> f32 {
        assert!(self.num_samples > 0);
        self.total / (self.num_samples as f32)
    }
}

#[derive(Debug, Clone)]
pub struct TestResult {
    pub ticks: Vec<TestResultState>,
    pub val_err_stats: FxHashMap<String, ValueErrorStat>,
}

impl TestResult {
    pub fn new(ticks: Vec<TestResultState>) -> Self {
        let mut val_err_stats: FxHashMap<String, ValueErrorStat> =
            HashMap::with_hasher(FxBuildHasher::default());
        for tick in &ticks {
            let ball_err = if let Some(ball_err) = &tick.comparison.ball_err {
                ball_err
            } else {
                &StateErrSet::new()
            };

            let all_errs_iter = tick
                .comparison
                .car_errs
                .iter()
                .flatten()
                .chain(ball_err.iter());
            for (name, err) in all_errs_iter {
                let entry = val_err_stats.entry(name.clone()).or_default();
                entry.num_samples += 1;
                entry.max = f32::max(entry.max, *err);
                entry.total += err;
            }
        }

        Self {
            ticks,
            val_err_stats,
        }
    }
}
