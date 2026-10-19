//! Socket transport for the viewer listener.
//!
//! Use [`viewer::ViewerListener`] to accept a simulation connection. The shared
//! socket-write implementation stays private to this module.

mod socket;
pub(crate) use socket::OutgoingBuffer;
pub mod simulation;
pub mod viewer;
