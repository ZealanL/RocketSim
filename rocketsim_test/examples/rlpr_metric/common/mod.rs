//! Backend-neutral multi-tick metric for RLPR recordings.
//!
//! The `v3` and `v2` adapters implement [`ReplayBackend`]. Each adapter steps
//! all available cars in one arena. [`evaluate`] compares each car and the
//! shared ball with the recorded state. Contact categories can overlap.
//!
//! Each segment starts from supported recorded state and reconstructed
//! history. Wheel raycasts refresh contact gates without advancing dynamics.
//! Reset mode can also restore state before each step. The v12 report lists
//! the applied, partial, and unused fields.

pub mod validation;

mod backend;
mod body;
mod coverage;
mod evaluation;
mod gating;
mod restore;
mod rows;
mod scoring;
mod transitions;

#[cfg(test)]
mod test_support;

// The metric and stress examples use different parts of this shared module.
#[cfg(test)]
pub use test_support::restore_test_tick;

#[allow(unused_imports)]
pub use self::{
    backend::*, body::*, coverage::*, evaluation::*, gating::*, restore::*, rows::*, scoring::*,
    transitions::*,
};
