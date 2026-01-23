use miniquad::{KeyCode, MouseButton};

#[derive(Debug, Copy, Clone)]
pub enum WindowEvent {
    Resize { width: f32, height: f32 },
    MouseButtonDown { button: MouseButton },
    KeyDown { key: KeyCode },
    Quit
}