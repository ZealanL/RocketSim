use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
};

use crate::backend::WindowEvent;

/// FIFO queue of [`WindowEvent`]s shared between renderer and sim threads.
#[derive(Debug, Clone)]
pub struct WindowEventQueue {
    queue: VecDeque<WindowEvent>,
}

impl Default for WindowEventQueue {
    fn default() -> Self {
        Self::new()
    }
}

impl WindowEventQueue {
    /// Empty queue.
    pub fn new() -> Self {
        Self {
            queue: VecDeque::new(),
        }
    }

    /// Pushes one event from the renderer thread.
    pub fn push(&mut self, event: WindowEvent) {
        self.queue.push_back(event);
    }

    /// Drains all queued events (called once per tick by the sim thread).
    pub fn pop_all(&mut self) -> Vec<WindowEvent> {
        self.queue.drain(..).collect()
    }
}

/// Thread-shared [`WindowEventQueue`] (`Arc<Mutex<..>>`).
pub type SharedWindowEvents = Arc<Mutex<WindowEventQueue>>;
