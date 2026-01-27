#![allow(clippy::suboptimal_flops)]

mod base;
mod bullet;
mod logging;
pub mod shared;
mod sim;
#[cfg(feature = "vis")]
pub mod vis;
mod glam_inc;
///////////

pub use crate::sim::*;
pub use base::*;
pub use glam_inc::*;
