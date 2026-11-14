//! Parallel stepping for many arenas.

use super::Arena;

/// Number of arenas for one parallel work item.
///
/// The call aims for about four chunks on each worker. The result is at least
/// one arena.
fn batch_grain(len: usize) -> usize {
    let workers = rayon::current_num_threads().max(1);
    (len / (workers * 4)).max(1)
}

impl Arena {
    /// Advances each arena in `arenas` by one tick.
    ///
    /// Use [`Arena::get_last_step_events`] on each arena after the call to
    /// read that arena's events. The method returns no events.
    ///
    /// # Execution
    ///
    /// An empty slice or a slice with one arena runs on the calling thread.
    /// This path does not use the Rayon pool. It does not create worker
    /// threads.
    ///
    /// A slice with two or more arenas uses the current Rayon pool. The call
    /// joins the full batch before it returns. By default, the current pool is
    /// the process-global pool. A caller can select a local pool with
    /// `pool.install`.
    ///
    /// # Determinism
    ///
    /// Each arena keeps its own physics state and random generator. An arena
    /// tick does not depend on the worker that runs it. The arena order across
    /// workers is not specified.
    ///
    /// # Thread pool
    ///
    /// The default pool size comes from Rayon. Set `RAYON_NUM_THREADS` before
    /// the first parallel call to choose the size. To use a local pool, call
    /// `pool.install(|| Arena::step_batch(&mut arenas))`.
    ///
    /// # Panics
    ///
    /// A panic in an arena propagates to the caller. The call does not roll
    /// back the batch. Other arenas can continue, and each arena that finishes
    /// keeps its new state. The batch can therefore make partial progress.
    ///
    /// # Example
    ///
    /// ```
    /// use rocketsim::{Arena, GameMode};
    ///
    /// let mut arenas = [Arena::new(GameMode::TheVoid), Arena::new(GameMode::TheVoid)];
    /// Arena::step_batch(&mut arenas);
    /// assert_eq!(arenas[0].tick_count(), 1);
    /// assert_eq!(arenas[1].tick_count(), 1);
    /// ```
    pub fn step_batch(arenas: &mut [Arena]) {
        Self::step_batch_with(arenas, |_, _| {});
    }

    /// Advances each arena in `arenas` by one tick after a control callback.
    ///
    /// `before_step` runs for each arena before that arena's tick. The
    /// callback receives the arena index and the arena. Write the controls
    /// for the arena in the callback. Do not write controls for another
    /// arena.
    ///
    /// Use [`Arena::get_last_step_events`] on each arena after the call to
    /// read that arena's events. The method returns no events.
    ///
    /// Execution, determinism, thread-pool selection, and panic behavior match
    /// [`Arena::step_batch`]. A panic in the callback has the same effect as a
    /// panic in an arena. The callback can read shared immutable data. It must
    /// write only to its arena. Do not use a shared random source. A callback
    /// result must not depend on another callback invocation.
    ///
    /// # Example
    ///
    /// ```
    /// use rocketsim::{Arena, CarBodyConfig, CarControls, GameMode, Team};
    ///
    /// let mut arenas = [Arena::new(GameMode::TheVoid), Arena::new(GameMode::TheVoid)];
    /// let car = arenas[0].add_car(Team::Blue, CarBodyConfig::OCTANE);
    /// let controls = CarControls::default().with_throttle(1.0);
    /// Arena::step_batch_with(&mut arenas, |arena_idx, arena| {
    ///     if arena_idx == 0 {
    ///         arena.set_car_controls(car, controls);
    ///     }
    /// });
    /// ```
    pub fn step_batch_with(arenas: &mut [Arena], before_step: impl Fn(usize, &mut Arena) + Sync) {
        if arenas.len() < 2 {
            for (arena_idx, arena) in arenas.iter_mut().enumerate() {
                before_step(arena_idx, arena);
                arena.step_tick();
            }
            return;
        }

        use rayon::prelude::*;

        let grain = batch_grain(arenas.len());
        arenas
            .par_iter_mut()
            .enumerate()
            .with_min_len(grain)
            .for_each(|(arena_idx, arena)| {
                before_step(arena_idx, arena);
                arena.step_tick();
            });
    }
}
