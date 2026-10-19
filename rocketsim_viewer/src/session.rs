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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tick_advancement_handles_repeats_jumps_and_rewinds() {
        for (previous, tick, expected) in [
            (
                None,
                9,
                TickAdvance {
                    steps: 1,
                    reset: false,
                },
            ),
            (
                Some(9),
                9,
                TickAdvance {
                    steps: 0,
                    reset: false,
                },
            ),
            (
                Some(9),
                12,
                TickAdvance {
                    steps: 3,
                    reset: false,
                },
            ),
            (
                Some(9),
                30,
                TickAdvance {
                    steps: 10,
                    reset: false,
                },
            ),
            (
                Some(9),
                4,
                TickAdvance {
                    steps: 0,
                    reset: true,
                },
            ),
        ] {
            assert_eq!(tick_advance(previous, tick), expected);
        }
    }

    #[test]
    fn controller_selection_stays_stable_and_recovers_on_disconnect() {
        let cases = [
            (Some(1), None, None, &[1, 2][..], Some(1)),
            (Some(1), None, Some(1), &[2][..], Some(2)),
            (None, Some(3), None, &[3][..], Some(3)),
            (None, None, None, &[][..], None),
        ];
        for (current, connected_event, disconnected_event, connected, expected) in cases {
            assert_eq!(
                choose_active_pad(current, connected_event, disconnected_event, connected),
                expected
            );
        }
    }
}
