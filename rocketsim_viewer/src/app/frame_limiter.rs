use std::time::{Duration, Instant};

#[derive(Default)]
pub(super) struct FrameLimiter {
    last_start: Option<Instant>,
}

impl FrameLimiter {
    fn remaining(&self, now: Instant, fps: u32) -> Duration {
        if fps == 0 {
            return Duration::ZERO;
        }
        let interval = Duration::from_secs_f64(1.0 / f64::from(fps));
        self.last_start.map_or(Duration::ZERO, |last| {
            interval.saturating_sub(now.saturating_duration_since(last))
        })
    }

    /// Pace each native frame, including frames requested by input events.
    /// Return the elapsed time so the caller can update the input timestamp.
    pub(super) fn wait(&mut self, fps: u32) -> Duration {
        let before = Instant::now();
        let remaining = self.remaining(before, fps);
        if !remaining.is_zero() {
            std::thread::sleep(remaining);
        }
        let started = Instant::now();
        // Use the actual start. A late frame must not cause a catch-up burst.
        self.last_start = Some(started);
        started.duration_since(before)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deadlines_cover_early_input_slow_frames_and_live_limit_changes() {
        let start = Instant::now();
        let mut limiter = FrameLimiter::default();
        assert_eq!(limiter.remaining(start, 60), Duration::ZERO);
        limiter.last_start = Some(start);

        for (elapsed_ms, fps, expected) in [
            (0, 60, Duration::from_secs_f64(1.0 / 60.0)),
            (
                5,
                60,
                Duration::from_secs_f64(1.0 / 60.0) - Duration::from_millis(5),
            ),
            (20, 60, Duration::ZERO),
            (1000, 60, Duration::ZERO),
            (0, 0, Duration::ZERO),
            (5, 0, Duration::ZERO),
            (
                20,
                30,
                Duration::from_secs_f64(1.0 / 30.0) - Duration::from_millis(20),
            ),
            (10, 120, Duration::ZERO),
        ] {
            assert_eq!(
                limiter.remaining(start + Duration::from_millis(elapsed_ms), fps),
                expected,
                "elapsed_ms={elapsed_ms}, fps={fps}"
            );
        }

        limiter.last_start = Some(start + Duration::from_secs(1));
        assert_eq!(
            limiter.remaining(start + Duration::from_millis(1001), 60),
            Duration::from_secs_f64(1.0 / 60.0) - Duration::from_millis(1)
        );
    }
}
