use std::sync::Arc;

use rocketsim_vis::{
    net_client::FrameClient,
    proto::{ControlMsg, SceneMetadata, VisFrame},
};

#[derive(Debug, Clone)]
pub struct StreamSnapshot {
    pub generation: u64,
    pub frame: Option<VisFrame>,
    pub scene: Option<Arc<SceneMetadata>>,
}

pub fn snapshot(client: &FrameClient) -> StreamSnapshot {
    let (generation, frame, scene) = client.snapshot();
    StreamSnapshot {
        generation,
        frame,
        scene,
    }
}

pub fn send_controls(client: &FrameClient, generation: u64, msg: ControlMsg) -> bool {
    send_controls_at_generation(generation, msg, |generation, msg| {
        client.send_controls_for_generation(generation, msg)
    })
}

pub fn send_controls_at_generation(
    generation: u64,
    msg: ControlMsg,
    send: impl FnOnce(u64, ControlMsg) -> bool,
) -> bool {
    send(generation, msg)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TickAdvance {
    pub steps: usize,
    pub reset: bool,
}

pub fn tick_advance(previous: Option<u64>, tick: u64) -> TickAdvance {
    match previous {
        None => TickAdvance {
            steps: 1,
            reset: false,
        },
        Some(last) if tick < last => TickAdvance {
            steps: 0,
            reset: true,
        },
        Some(last) => TickAdvance {
            steps: tick.saturating_sub(last).min(10) as usize,
            reset: false,
        },
    }
}

pub fn choose_active_pad<T: Copy + Eq>(
    current: Option<T>,
    connected_event: Option<T>,
    disconnected_event: Option<T>,
    connected: &[T],
) -> Option<T> {
    if current.is_some() && current == disconnected_event {
        connected.first().copied().or(connected_event)
    } else {
        current
            .or(connected_event)
            .or_else(|| connected.first().copied())
    }
}

pub fn generation_changed(previous: Option<u64>, current: u64) -> bool {
    previous.is_some_and(|generation| generation != current)
}

#[cfg(test)]
pub fn pad_edge_allowed(focused: bool, device_changed: bool, held: bool, was_held: bool) -> bool {
    focused && !device_changed && held && !was_held
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repeated_ticks_do_not_advance_and_rewind_resets() {
        assert_eq!(
            tick_advance(Some(14), 14),
            TickAdvance {
                steps: 0,
                reset: false
            }
        );
        assert_eq!(
            tick_advance(Some(14), 15),
            TickAdvance {
                steps: 1,
                reset: false
            }
        );
        assert_eq!(
            tick_advance(Some(14), 8),
            TickAdvance {
                steps: 0,
                reset: true
            }
        );
    }

    #[test]
    fn active_pad_stays_stable_and_moves_on_disconnect() {
        assert_eq!(choose_active_pad(Some(1), None, None, &[1, 2]), Some(1));
        assert_eq!(choose_active_pad(Some(1), None, Some(1), &[2]), Some(2));
        assert_eq!(choose_active_pad(None, Some(3), None, &[3]), Some(3));
    }

    #[test]
    fn generation_change_resets_independent_of_tick_direction() {
        assert!(generation_changed(Some(4), 5));
        assert!(generation_changed(Some(5), 4));
        assert!(!generation_changed(Some(5), 5));
        assert!(!generation_changed(None, 5));
    }

    #[test]
    fn controls_use_the_generation_from_the_observed_snapshot() {
        let mut sent_generation = None;
        let accepted = send_controls_at_generation(
            42,
            ControlMsg::new(0, rocketsim::CarControls::default(), 0, 0),
            |generation, _| {
                sent_generation = Some(generation);
                generation == 42
            },
        );
        assert!(accepted);
        assert_eq!(sent_generation, Some(42));
    }

    #[test]
    fn focus_and_device_changes_suppress_edges_but_allow_tracking() {
        assert!(!pad_edge_allowed(false, false, true, false));
        assert!(!pad_edge_allowed(true, true, true, false));
        assert!(!pad_edge_allowed(true, false, true, true));
        assert!(pad_edge_allowed(true, false, true, false));
    }
}
