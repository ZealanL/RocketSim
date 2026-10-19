//! Versioned message types and codecs for the viewer protocol.
//!
//! This module contains no socket or network code. Use the multiplexed sim
//! codec for physics frames and retained-scene chunks.

mod codec;
mod messages;

pub use codec::*;
pub use messages::*;
