use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
};

use crate::vis::backend::WindowEvent;

#[derive(Debug, Clone)]
pub struct WindowEventQueue {
    queue: VecDeque<WindowEvent>,
}

impl WindowEventQueue {
    pub fn new() -> Self {
        Self {
            queue: VecDeque::new(),
        }
    }

    pub fn push(&mut self, event: WindowEvent) {
        self.queue.push_back(event);
    }

    pub fn pop_all(&mut self) -> Vec<WindowEvent> {
        self.queue.drain(..).collect()
    }
}

pub type SharedWindowEvents = Arc<Mutex<WindowEventQueue>>;
