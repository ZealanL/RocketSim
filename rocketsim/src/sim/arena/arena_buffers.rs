use super::ArenaContactTracker;
use crate::bullet::dynamics::SeqImpulseConstraintSolver;

/// Reusable temporary storage for one arena step.
///
/// Pass one value to [`crate::Arena::step_tick_with_buffers`]. The arena borrows the
/// storage for exactly one tick. Reuse the same value for each arena that a
/// worker thread steps.
///
/// Use this storage when one worker steps many arenas. Use
/// [`crate::Arena::step_tick`] for a single arena to avoid the storage exchange.
/// Speed depends on the workload. Measure both methods before you choose one.
///
/// The storage holds only temporary step data. It does not hold world state,
/// persistent manifolds, output events, contact history, or random-generator
/// state. All of those stay in the arena.
///
/// # Lifetime
///
/// The arena keeps its own storage object. The step exchanges the two storage
/// objects for one tick and then exchanges them back. This value keeps the
/// buffers used by that step. A successful step clears their contents but
/// retains their capacity. An arena that also uses [`crate::Arena::step_tick`]
/// retains its own grown buffers.
///
/// Use one buffer set for one worker thread at a time. Do not use the same
/// value from two threads at the same time.
///
/// # Panic safety
///
/// A panic inside the step cannot move temporary records into another arena.
/// The restore guard runs during unwinding. It puts the arena storage back
/// first. It then clears the records and temporary rows of the reuse storage.
/// The next call starts from empty storage. A build with `panic = "abort"`
/// stops the process, so no arena can reuse the storage.
///
/// # Example
///
/// ```no_run
/// use rocketsim::{Arena, ArenaStepBuffers, GameMode};
///
/// let mut arenas = [Arena::new(GameMode::TheVoid), Arena::new(GameMode::TheVoid)];
/// let mut buffers = ArenaStepBuffers::new();
/// for _ in 0..120 {
///     for arena in &mut arenas {
///         arena.step_tick_with_buffers(&mut buffers);
///     }
/// }
/// ```
pub struct ArenaStepBuffers {
    pub(super) solver: SeqImpulseConstraintSolver,
    pub(super) contact_tracker: ArenaContactTracker,
}

impl ArenaStepBuffers {
    /// Create empty step buffers.
    ///
    /// This does not allocate a step buffer. The first step allocates the
    /// buffers that it needs.
    #[must_use]
    pub fn new() -> Self {
        Self {
            solver: SeqImpulseConstraintSolver::default(),
            contact_tracker: ArenaContactTracker::new_empty(),
        }
    }

    /// Remove temporary records and reset per-step flags. Keep the capacity.
    pub(super) fn reset(&mut self) {
        self.solver.reset_transient();
        self.contact_tracker.reset_transient();
    }
}

impl Default for ArenaStepBuffers {
    fn default() -> Self {
        Self::new()
    }
}
