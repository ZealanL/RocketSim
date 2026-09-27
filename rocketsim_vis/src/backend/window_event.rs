use miniquad::{KeyCode, MouseButton};

/// Input/window event forwarded from the renderer thread to the sim thread.
///
/// [`crate::VisInst`] drains these each tick; `C` and `Space` drive the camera.
#[derive(Debug, Copy, Clone)]
pub enum WindowEvent {
    /// Window was resized to `width` x `height` pixels.
    Resize { width: f32, height: f32 },
    /// Mouse button pressed.
    MouseButtonDown { button: MouseButton },
    /// Key pressed.
    KeyDown { key: KeyCode },
    /// Window close requested.
    Quit,
}
