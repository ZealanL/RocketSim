#![allow(
    clippy::suboptimal_flops,
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    clippy::cast_possible_truncation
)]

mod base;
mod bullet;
mod glam_inc;
mod logging;
#[doc(hidden)]
pub mod rlpr;
pub mod shared;
mod sim;
///////////

pub use base::*;
pub use glam_inc::*;

pub use crate::sim::*;
