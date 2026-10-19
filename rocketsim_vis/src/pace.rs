//! Wait between simulation ticks to run near real time.
//!
//! [`TickPacer::wait`] returns the number of simulation ticks to run. It
//! sleeps if the next deadline is in the future. If the application is late,
//! it returns extra ticks up to a configured limit. After a longer delay, it
//! resets the next deadline instead of requesting an unlimited number of ticks.
//!
//! Call [`rocketsim::Arena::step_tick`] once for each returned tick. The pacer
//! does not step an arena, send input, or change simulation time by itself.
//! Missed deadlines in [`PaceStats::dropped`] are not skipped physics ticks.
//! Your application runs fewer simulation ticks relative to elapsed real time.
//!
//! ```no_run
//! use rocketsim::{Arena, GameMode};
//! use rocketsim_vis::pace::TickPacer;
//!
//! let mut arena = Arena::new(GameMode::TheVoid);
//! let mut pacer = TickPacer::at_120hz();
//! for _ in 0..120 {
//!     for _ in 0..pacer.wait() {
//!         arena.step_tick();
//!     }
//! }
//! println!("Missed deadlines: {}", pacer.stats().dropped);
//! ```
//!
//! Use [`steps_for_lateness`] to test timing calculations without sleeping.
//! Omit the pacer when a batch simulation must run as fast as possible.

use std::time::{Duration, Instant};

/// Simulation rate in ticks per second: `120.0`.
pub const TICK_HZ: f64 = 120.0;

/// Nominal interval between ticks: `8_333_333` nanoseconds.
pub const TICK_INTERVAL: Duration = Duration::from_nanos(8_333_333);

/// Simulated time per tick in seconds: `1.0 / 120.0`.
pub const TICK_DT: f32 = 1.0 / 120.0;

/// Default maximum number of extra ticks returned after a delay.
///
/// The value `5` permits a batch of at most six ticks from one wait.
pub const MAX_CATCH_UP: u32 = 5;

/// Cumulative counts returned by [`TickPacer::stats`].
///
/// These counts describe requests from the pacer. They do not measure how
/// many physics ticks the application actually completed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PaceStats {
    /// Total ticks returned by [`TickPacer::wait`], including extra ticks.
    pub ticks: u64,
    /// Missed real-time deadlines beyond the configured extra-tick limit.
    pub dropped: u64,
    /// Largest number of extra ticks returned by one wait. Initially `0`.
    pub max_backlog: u32,
}

/// A timer that requests bounded batches of simulation ticks.
///
/// [`Self::wait`] returns one tick when on time. It returns more ticks after
/// a delay, up to `1 + max_catch_up`. Longer delays reset the schedule.
/// The timer does not guarantee exact sleep duration or simulation speed.
///
/// ```rust
/// use std::time::Duration;
///
/// use rocketsim_vis::pace::TickPacer;
///
/// // Drive the sim at 120 Hz: wait, then step that many ticks.
/// let mut pacer = TickPacer::at_120hz();
/// let steps = pacer.wait();
/// assert!(steps >= 1);
/// assert!(pacer.stats().ticks >= 1);
/// ```
pub struct TickPacer {
    interval: Duration,
    max_catch_up: u32,
    next: Instant,
    stats: PaceStats,
}

impl TickPacer {
    /// Create a timer with a fixed interval and an extra-tick limit.
    ///
    /// Schedule the first deadline one `interval` after construction. Clamp
    /// `max_catch_up` to `u32::MAX - 1` so the total tick count fits in `u32`.
    /// The first wait sleeps only for time that remains before that deadline.
    ///
    /// # Panics
    ///
    /// Panic if `interval` is zero or the system clock cannot represent the
    /// first deadline.
    pub fn new(interval: Duration, max_catch_up: u32) -> Self {
        assert!(!interval.is_zero(), "tick interval must be non-zero");
        let max_catch_up = max_catch_up.min(u32::MAX - 1);
        Self {
            interval,
            max_catch_up,
            next: Instant::now()
                .checked_add(interval)
                .expect("tick interval is too large for the system clock"),
            stats: PaceStats::default(),
        }
    }

    /// Create a 120 Hz timer with at most five extra ticks per wait.
    ///
    /// Use [`TICK_INTERVAL`] and [`MAX_CATCH_UP`] as the constructor values.
    pub fn at_120hz() -> Self {
        Self::new(TICK_INTERVAL, MAX_CATCH_UP)
    }

    /// Return the configured interval between deadlines.
    pub fn interval(&self) -> Duration {
        self.interval
    }

    /// Copy the accumulated timing counts without resetting them.
    pub fn stats(&self) -> PaceStats {
        self.stats
    }

    /// Wait for the next deadline and return the number of ticks to run.
    ///
    /// Return at least `1` and at most `1 + max_catch_up`. A late call can
    /// return immediately. Update [`PaceStats`] before returning. Run one
    /// physics tick for each returned tick to follow the timer's schedule.
    pub fn wait(&mut self) -> u32 {
        let now = Instant::now();
        if now < self.next {
            std::thread::sleep(self.next - now);
        }
        let now = Instant::now();
        let (steps, dropped) = steps_for_lateness(now, self.next, self.interval, self.max_catch_up);
        self.next = advance_deadline(self.next, now, self.interval, steps);
        record_steps(&mut self.stats, steps, dropped);
        steps
    }
}

/// Calculate a tick batch and missed deadlines without waiting.
///
/// Return `(steps, dropped)`. `steps` includes the next tick and at most
/// `max_catch_up` extra ticks. `dropped` counts additional elapsed deadlines.
/// An early call or a zero interval returns `(1, 0)`. This function does not
/// modify a deadline or the clock. See [`TickPacer::wait`] for actual pacing.
///
/// ```
/// use std::time::{Duration, Instant};
///
/// use rocketsim_vis::pace::steps_for_lateness;
///
/// let deadline = Instant::now();
/// let interval = Duration::from_millis(10);
/// let now = deadline + Duration::from_millis(100);
/// assert_eq!(steps_for_lateness(now, deadline, interval, 5), (6, 5));
/// ```
pub fn steps_for_lateness(
    now: Instant,
    next: Instant,
    interval: Duration,
    max_catch_up: u32,
) -> (u32, u32) {
    if now < next || interval.is_zero() {
        return (1, 0);
    }
    let missed = steps_for_lateness_missed(now, next, interval);
    let catch_up = missed.min(max_catch_up.min(u32::MAX - 1));
    (1 + catch_up, missed - catch_up)
}

fn advance_deadline(next: Instant, now: Instant, interval: Duration, steps: u32) -> Instant {
    let next = next
        .checked_add(interval.saturating_mul(steps))
        .unwrap_or(now);
    if now >= next {
        // Re-sync after a long stall. Excess lateness was counted above.
        now.checked_add(interval)
            .expect("tick interval is too large for the system clock")
    } else {
        next
    }
}

fn record_steps(stats: &mut PaceStats, steps: u32, dropped: u32) {
    stats.ticks = stats.ticks.saturating_add(u64::from(steps));
    stats.dropped = stats.dropped.saturating_add(u64::from(dropped));
    stats.max_backlog = stats.max_backlog.max(steps.saturating_sub(1));
}

/// Whole extra intervals elapsed between `next` and `now` (0 if early).
/// Saturates instead of overflowing on hour-long stalls.
fn steps_for_lateness_missed(now: Instant, next: Instant, interval: Duration) -> u32 {
    if now < next || interval.is_zero() {
        return 0;
    }
    let late_ns = (now - next).as_nanos();
    let step_ns = interval.as_nanos().max(1);
    u32::try_from(late_ns / step_ns).unwrap_or(u32::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t0() -> Instant {
        Instant::now()
    }

    #[test]
    fn catch_up_math() {
        let base = t0();
        // Early and exactly-on-deadline both yield one step, zero drops.
        assert_eq!(
            steps_for_lateness(base, base + Duration::from_millis(8), TICK_INTERVAL, 5),
            (1, 0)
        );
        assert_eq!(steps_for_lateness(base, base, TICK_INTERVAL, 5), (1, 0));
        // 2 intervals late -> this tick + 2 catch-up, nothing dropped.
        let now = base + 2 * TICK_INTERVAL + Duration::from_micros(100);
        assert_eq!(
            steps_for_lateness(now, base, TICK_INTERVAL, MAX_CATCH_UP),
            (3, 0)
        );
        // 10 intervals late with cap 5 -> 6 steps, 5 dropped.
        let now = base + 10 * TICK_INTERVAL + Duration::from_micros(100);
        assert_eq!(
            steps_for_lateness(now, base, TICK_INTERVAL, MAX_CATCH_UP),
            (1 + MAX_CATCH_UP, 10 - MAX_CATCH_UP)
        );
        // The returned drop count includes all excess intervals once, even
        // when the caller re-syncs its deadline after this result.
        assert_eq!(steps_for_lateness(now, base, TICK_INTERVAL, 0), (1, 10));
    }

    #[test]
    fn dropped_ticks_are_recorded_once_after_resync() {
        let base = t0();
        let now = base + 10 * TICK_INTERVAL;
        let (steps, dropped) = steps_for_lateness(now, base, TICK_INTERVAL, MAX_CATCH_UP);
        let mut stats = PaceStats::default();
        record_steps(&mut stats, steps, dropped);

        assert_eq!(stats.ticks, 1 + u64::from(MAX_CATCH_UP));
        assert_eq!(stats.dropped, 10 - u64::from(MAX_CATCH_UP));
        assert_eq!(stats.max_backlog, MAX_CATCH_UP);
    }

    #[test]
    fn zero_interval_math_is_safe() {
        let base = t0();
        assert_eq!(steps_for_lateness(base, base, Duration::ZERO, 5), (1, 0));
    }

    #[test]
    fn stats_accumulate_over_waits() {
        // Generous interval: under parallel `cargo test` load a 1ms sleep
        // can overshoot into catch-up, so assert accounting (not exact
        // steps) — determinism of the step math is covered above.
        let mut pacer = TickPacer::new(Duration::from_millis(20), 5);
        let mut total = 0u64;
        let mut max_bl = 0u32;
        for _ in 0..2 {
            let steps = pacer.wait();
            assert!(steps >= 1, "wait must always run at least one tick");
            total += u64::from(steps);
            max_bl = max_bl.max(steps - 1);
        }
        let stats = pacer.stats();
        assert_eq!(stats.ticks, total);
        assert_eq!(stats.max_backlog, max_bl);
    }

    #[test]
    fn stall_resyncs_without_spiral() {
        let interval = Duration::from_millis(1);
        let first_deadline = t0() + interval;
        let stalled_at = first_deadline + Duration::from_millis(50);
        let (steps, dropped) = steps_for_lateness(stalled_at, first_deadline, interval, 2);

        assert_eq!(steps, 3, "capped at 1 + max_catch_up");
        assert_eq!(dropped, 48, "count excess deadlines once");

        let next = advance_deadline(first_deadline, stalled_at, interval, steps);
        assert_eq!(next, stalled_at + interval);
        assert_eq!(steps_for_lateness(next, next, interval, 2), (1, 0));
    }
}
