//! Map an RLPR v12 group 4 collision snapshot to a replay restore state.
//!
//! The reader in `rlpr::collision_snapshot` gives a portable view of the native
//! Bullet collision state. This module selects the part of that view that maps
//! to RocketSim bodies without a guess:
//!
//! - a manifold whose two bodies are the ball and a car, or two cars;
//! - the retained points of that manifold, in stored dispatcher order.
//!
//! The other manifolds touch at least one static world body. A capture has no
//! static shape identity, and it has no triangle part or index. These
//! manifolds are counted and skipped. They are never rebuilt.
//!
//! # Fields that the restore does not carry
//!
//! - Solver warm-start impulses. The verified point field mask `0x013f` does
//!   not contain them.
//! - Contact friction and restitution. RocketSim rebuilds them from its own
//!   car contact constants.
//! - Broadphase handles, edges, and pairs. The RocketSim broadphase keeps its
//!   own allocation order.
//!
//! A returned state is a partial restore. Do not report it as a complete
//! native collision checkpoint.
//!
//! # Row phase
//!
//! Use the collision group of the state row itself.
//!
//! The recorder runs `core::on_tick_start` before it calls the game level
//! tick. The row body state and the collision group are both read in that
//! function (`rl_phys_record/src/core/hooks.cpp` and `core/core.cpp`). The row
//! body state is the post-step state of the previous tick, and the recorded
//! manifolds are the ones that the previous tick left. Row `N` therefore holds
//! the collision state that exists at the start of the step from row `N` to
//! row `N + 1`.
//!
//! Do not use row `N + 1`. Its manifolds have already run the dispatch for the
//! step out of row `N`, so they are the result of the transition and not its
//! input.
//!
//! A contact world witness is only recomputed during a collision dispatch. A
//! witness in row `N` therefore describes the body pose of row `N - 1`. This
//! is correct state, not an error: the next dispatch refreshes the witness
//! from the local anchor. The local anchor and the normal are the durable
//! values.
//!
//! Measured on the 2026-11-03 corpus, the row `N - 1` fit has a mean residual
//! of 2.0e-4 bt, and the row `N` fit reaches 0.73 bt.
//! [`classify_witness_phase`] reports that difference. It is a diagnostic for
//! a recorder change. The restore does not reject a row from it: a turning
//! body can make a valid stale witness fit the current pose by a hair.

// Standalone extraction and witness diagnostics are not all used by the CLI.
#![allow(dead_code)]

use std::fmt::{self, Display, Formatter};

use rocketsim_test::{
    replay::collision::{
        CollisionReplayManifold, CollisionReplayPoint, CollisionReplayState, ReplayBodyRef,
    },
    rlpr::{
        CollisionBody, CollisionBodyRole, CollisionLayoutId, CollisionManifoldOrder,
        CollisionSectionStatus, CollisionSnapshot,
        collision_snapshot::{
            POINT_FIELD_CORE_MASK, POINT_FIELD_DISTANCE, POINT_FIELD_LOCAL_A, POINT_FIELD_LOCAL_B,
            POINT_FIELD_NORMAL_B, POINT_FIELD_WORLD_A, POINT_FIELD_WORLD_B, SCHEMA_VERSION,
            SECTION_BODIES, SECTION_MANIFOLDS,
        },
        tick_record::TickRecord,
    },
};

/// Point fields that a restore needs.
///
/// The verified build has all of them. A capture without one of them cannot
/// place a contact point, so the whole snapshot is refused.
pub const REQUIRED_POINT_FIELDS: u32 = POINT_FIELD_LOCAL_A
    | POINT_FIELD_LOCAL_B
    | POINT_FIELD_WORLD_A
    | POINT_FIELD_WORLD_B
    | POINT_FIELD_NORMAL_B
    | POINT_FIELD_DISTANCE;

/// Sections that a restore needs.
pub const REQUIRED_SECTIONS: u16 = SECTION_BODIES | SECTION_MANIFOLDS;

/// Highest accepted recorded car slot.
const MAX_CAR_SLOT: u8 = 7;
/// Number of car slots from zero to [`MAX_CAR_SLOT`].
const MAX_CAR_SLOTS: usize = MAX_CAR_SLOT as usize + 1;

/// Coverage of one extraction.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CollisionReplaySummary {
    /// Manifold records in the capture.
    pub native_manifolds: usize,
    /// Manifolds that map to RocketSim bodies.
    pub covered_manifolds: usize,
    /// Manifolds that the restore does not cover.
    pub skipped_manifolds: usize,
    /// Skipped manifolds with a static or unknown body.
    pub skipped_unmappable_manifolds: usize,
    /// Skipped manifolds that name an unavailable car slot.
    pub skipped_unavailable_manifolds: usize,
    /// Contact points in the capture.
    pub native_points: usize,
    /// Contact points in the covered manifolds.
    pub covered_points: usize,
    /// Verified point field mask of the capture.
    pub point_field_mask: u32,
}

impl CollisionReplaySummary {
    /// True when every captured manifold maps to RocketSim bodies.
    #[must_use]
    pub fn is_complete_manifold_set(&self) -> bool {
        self.skipped_manifolds == 0
    }
}

impl Display for CollisionReplaySummary {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "covered {}/{} manifolds ({} static, {} unavailable) and {}/{} points, point mask {:#06x}",
            self.covered_manifolds,
            self.native_manifolds,
            self.skipped_unmappable_manifolds,
            self.skipped_unavailable_manifolds,
            self.covered_points,
            self.native_points,
            self.point_field_mask
        )
    }
}

/// One successful extraction.
#[derive(Clone, Debug, PartialEq)]
pub struct CollisionReplayExtract {
    /// The covered manifolds, in stored dispatcher order.
    pub state: CollisionReplayState,
    /// Coverage of the capture.
    pub summary: CollisionReplaySummary,
}

/// Why a capture was not restored.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionReplaySkip {
    /// The row has no collision snapshot.
    NoSnapshot,
    /// The state row index is outside the recording.
    MissingRow,
    /// The schema version is not the verified version.
    UnknownSchema {
        /// Version from the capture.
        version: u16,
    },
    /// The capture does not identify the build.
    UnidentifiedBuild,
    /// The capture layout is not the verified layout.
    UnverifiedLayout {
        /// Layout from the capture.
        layout: CollisionLayoutId,
    },
    /// The capture sets the limit-clamped flag.
    LimitsClamped,
    /// A required section is not verified for the build.
    SectionNotVerified {
        /// Verified section mask.
        verified: u16,
    },
    /// The bodies section is absent or incomplete.
    BodiesIncomplete,
    /// The manifolds section is absent or incomplete.
    ManifoldsIncomplete,
    /// The manifold order is not the stored dispatcher order.
    NotDispatcherOrder {
        /// Order from the capture.
        order: CollisionManifoldOrder,
    },
    /// A point field mask misses a required field or holds an unverified
    /// field.
    PointFieldsUnverified {
        /// Point field mask of the manifold.
        mask: u32,
    },
    /// Two body records use the same catalog index.
    DuplicateCatalogIndex {
        /// The repeated catalog index.
        catalog_index: i32,
    },
    /// The capture holds more than one ball body.
    AmbiguousBall,
    /// Two car bodies use the same car slot.
    DuplicateCarSlot {
        /// The repeated car slot.
        car_slot: u8,
    },
    /// A manifold names a catalog index without a body record.
    UnknownCatalogIndex {
        /// The missing catalog index.
        catalog_index: i32,
    },
    /// A manifold point count contradicts its record.
    PointCountMismatch {
        /// Index of the manifold in the stored order.
        manifold: usize,
    },
    /// A required point field is absent from a point.
    PointFieldMissing {
        /// Index of the manifold in the stored order.
        manifold: usize,
        /// Index of the point in the manifold.
        point: usize,
    },
    /// The capture holds the same covered body pair twice.
    DuplicatePair {
        /// Index of the second manifold in the stored order.
        manifold: usize,
    },
}

impl Display for CollisionReplaySkip {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoSnapshot => formatter.write_str("the row has no collision snapshot"),
            Self::MissingRow => formatter.write_str("the state row index is outside the recording"),
            Self::UnknownSchema { version } => {
                write!(
                    formatter,
                    "collision schema {version} is not {SCHEMA_VERSION}"
                )
            }
            Self::UnidentifiedBuild => formatter.write_str("the collision capture has no build"),
            Self::UnverifiedLayout { layout } => {
                write!(
                    formatter,
                    "collision layout {layout:?} is not the verified layout"
                )
            }
            Self::LimitsClamped => formatter.write_str("a collision limit clamped a native count"),
            Self::SectionNotVerified { verified } => write!(
                formatter,
                "collision section mask {verified:#06x} misses {REQUIRED_SECTIONS:#06x}"
            ),
            Self::BodiesIncomplete => {
                formatter.write_str("the collision body section is incomplete")
            }
            Self::ManifoldsIncomplete => {
                formatter.write_str("the collision manifold section is incomplete")
            }
            Self::NotDispatcherOrder { order } => {
                write!(
                    formatter,
                    "collision manifold order {order:?} is not stored order"
                )
            }
            Self::PointFieldsUnverified { mask } => write!(
                formatter,
                "collision point mask {mask:#06x} is not verified"
            ),
            Self::DuplicateCatalogIndex { catalog_index } => {
                write!(formatter, "catalog index {catalog_index} is not unique")
            }
            Self::AmbiguousBall => formatter.write_str("the capture holds more than one ball"),
            Self::DuplicateCarSlot { car_slot } => {
                write!(formatter, "car slot {car_slot} is not unique")
            }
            Self::UnknownCatalogIndex { catalog_index } => {
                write!(formatter, "catalog index {catalog_index} has no body")
            }
            Self::PointCountMismatch { manifold } => {
                write!(
                    formatter,
                    "manifold {manifold} point count contradicts its record"
                )
            }
            Self::PointFieldMissing { manifold, point } => {
                write!(
                    formatter,
                    "manifold {manifold} point {point} misses a required field"
                )
            }
            Self::DuplicatePair { manifold } => {
                write!(formatter, "manifold {manifold} repeats a covered body pair")
            }
        }
    }
}

impl std::error::Error for CollisionReplaySkip {}

/// One body record that a manifold can name.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CatalogBody {
    Ball,
    Car {
        slot: u8,
    },
    /// A static body, an unknown body, or a car without a valid slot.
    Other,
}

/// The result of a catalog body lookup.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MapResult {
    /// The body maps to an arena body.
    Mapped(ReplayBodyRef),
    /// The body is a car slot without an arena car.
    Unavailable,
    /// The body cannot map to an arena body.
    Unmappable,
}

fn map_body(body: CatalogBody, car_available: &[bool]) -> MapResult {
    match body {
        CatalogBody::Ball => MapResult::Mapped(ReplayBodyRef::Ball),
        CatalogBody::Car { slot }
            if car_available
                .get(usize::from(slot))
                .copied()
                .unwrap_or(false) =>
        {
            MapResult::Mapped(ReplayBodyRef::Car { slot })
        }
        CatalogBody::Car { .. } => MapResult::Unavailable,
        CatalogBody::Other => MapResult::Unmappable,
    }
}

/// Map a car slot or the ball to a small key for pair de-duplication.
fn pair_key(body: ReplayBodyRef) -> u8 {
    match body {
        ReplayBodyRef::Ball => u8::MAX,
        ReplayBodyRef::Car { slot } => slot,
    }
}

/// Build the catalog lookup table from the body records.
fn catalog(records: &[CollisionBody]) -> Result<Vec<(i32, CatalogBody)>, CollisionReplaySkip> {
    let mut entries: Vec<(i32, CatalogBody)> = Vec::with_capacity(records.len());
    let mut has_ball = false;
    let mut car_slots = [false; MAX_CAR_SLOTS];

    for record in records {
        // An uncatalogued body has a negative catalog index. The Soccar world
        // statics all share `-1`. Such a body can never map to an arena body,
        // so it has no lookup entry and no uniqueness requirement.
        if record.catalog_index < 0 {
            continue;
        }

        if entries
            .iter()
            .any(|&(index, _)| index == record.catalog_index)
        {
            return Err(CollisionReplaySkip::DuplicateCatalogIndex {
                catalog_index: record.catalog_index,
            });
        }

        if record.role == CollisionBodyRole::Ball {
            if has_ball {
                return Err(CollisionReplaySkip::AmbiguousBall);
            }
            has_ball = true;
            entries.push((record.catalog_index, CatalogBody::Ball));
            continue;
        }

        if record.role == CollisionBodyRole::Car {
            let slot = record.car_slot;
            if slot > MAX_CAR_SLOT {
                entries.push((record.catalog_index, CatalogBody::Other));
                continue;
            }
            if car_slots[usize::from(slot)] {
                return Err(CollisionReplaySkip::DuplicateCarSlot { car_slot: slot });
            }
            car_slots[usize::from(slot)] = true;
            entries.push((record.catalog_index, CatalogBody::Car { slot }));
            continue;
        }

        entries.push((record.catalog_index, CatalogBody::Other));
    }

    Ok(entries)
}

fn lookup(
    entries: &[(i32, CatalogBody)],
    catalog_index: i32,
) -> Result<CatalogBody, CollisionReplaySkip> {
    if catalog_index < 0 {
        return Ok(CatalogBody::Other);
    }
    entries
        .iter()
        .find(|&&(index, _)| index == catalog_index)
        .map(|&(_, body)| body)
        .ok_or(CollisionReplaySkip::UnknownCatalogIndex { catalog_index })
}

/// Read one point.
///
/// The caller has already checked the point field mask.
fn point(
    manifold: usize,
    index: usize,
    source: &rocketsim_test::rlpr::CollisionManifoldPoint,
) -> Result<CollisionReplayPoint, CollisionReplaySkip> {
    let field_missing = CollisionReplaySkip::PointFieldMissing {
        manifold,
        point: index,
    };
    Ok(CollisionReplayPoint {
        local_point_a: source
            .local_point_a
            .map(glam::Vec3A::from_array)
            .ok_or(field_missing)?,
        local_point_b: source
            .local_point_b
            .map(glam::Vec3A::from_array)
            .ok_or(field_missing)?,
        position_world_on_a: source
            .position_world_on_a
            .map(glam::Vec3A::from_array)
            .ok_or(field_missing)?,
        position_world_on_b: source
            .position_world_on_b
            .map(glam::Vec3A::from_array)
            .ok_or(field_missing)?,
        normal_world_on_b: source
            .normal_world_on_b
            .map(glam::Vec3A::from_array)
            .ok_or(field_missing)?,
        distance: source.distance1.ok_or(field_missing)?,
    })
}

/// Select the covered manifolds of one capture.
///
/// `car_available` holds one flag per recorded car slot. A manifold that
/// names an unavailable slot is counted and skipped, because the arena has no
/// body to attach it to.
///
/// # Errors
///
/// Returns [`CollisionReplaySkip`] when the capture cannot be restored
/// without a guess. A skip covers the whole row. It never returns a partial
/// state for a partially readable snapshot.
pub fn extract(
    snapshot: Option<&CollisionSnapshot>,
    car_available: &[bool],
) -> Result<CollisionReplayExtract, CollisionReplaySkip> {
    let snapshot = snapshot.ok_or(CollisionReplaySkip::NoSnapshot)?;

    if snapshot.schema_version != SCHEMA_VERSION {
        return Err(CollisionReplaySkip::UnknownSchema {
            version: snapshot.schema_version,
        });
    }
    if !snapshot.build_identified {
        return Err(CollisionReplaySkip::UnidentifiedBuild);
    }
    if snapshot.layout_id != CollisionLayoutId::Current20261102 {
        return Err(CollisionReplaySkip::UnverifiedLayout {
            layout: snapshot.layout_id,
        });
    }
    if snapshot.limits_clamped {
        return Err(CollisionReplaySkip::LimitsClamped);
    }
    if snapshot.verified_sections & REQUIRED_SECTIONS != REQUIRED_SECTIONS {
        return Err(CollisionReplaySkip::SectionNotVerified {
            verified: snapshot.verified_sections,
        });
    }

    let bodies = snapshot
        .bodies
        .as_ref()
        .filter(|section| {
            section.report.status == CollisionSectionStatus::Ok
                && !section.report.truncated
                && section.report.dropped_count == 0
                && section.report.skipped_count == 0
        })
        .ok_or(CollisionReplaySkip::BodiesIncomplete)?;

    let manifolds = snapshot
        .manifolds
        .as_ref()
        .filter(|section| {
            section.report.status == CollisionSectionStatus::Ok
                && !section.report.truncated
                && section.report.dropped_count == 0
                && section.report.skipped_count == 0
        })
        .ok_or(CollisionReplaySkip::ManifoldsIncomplete)?;

    if manifolds.order != CollisionManifoldOrder::DispatcherStored {
        return Err(CollisionReplaySkip::NotDispatcherOrder {
            order: manifolds.order,
        });
    }

    let entries = catalog(&bodies.records)?;
    let mut summary = CollisionReplaySummary {
        native_manifolds: manifolds.records.len(),
        ..CollisionReplaySummary::default()
    };
    let mut state = CollisionReplayState::default();
    let mut pairs: Vec<(u8, u8)> = Vec::with_capacity(manifolds.records.len());

    for (manifold_index, record) in manifolds.records.iter().enumerate() {
        let mask = record.point_field_mask;
        if usize::from(record.point_count) != record.points.len() {
            return Err(CollisionReplaySkip::PointCountMismatch {
                manifold: manifold_index,
            });
        }

        summary.point_field_mask |= mask;
        summary.native_points += record.points.len();

        let mapped0 = map_body(lookup(&entries, record.body0_catalog_index)?, car_available);
        let mapped1 = map_body(lookup(&entries, record.body1_catalog_index)?, car_available);
        let (MapResult::Mapped(body0), MapResult::Mapped(body1)) = (mapped0, mapped1) else {
            // A static, unknown, or unavailable body cannot map to an arena
            // body. Do not rebuild this manifold.
            summary.skipped_manifolds += 1;
            if mapped0 == MapResult::Unavailable || mapped1 == MapResult::Unavailable {
                summary.skipped_unavailable_manifolds += 1;
            } else {
                summary.skipped_unmappable_manifolds += 1;
            }
            continue;
        };

        // The mask matters only for a manifold that this restore can build.
        // A skipped static manifold cannot refuse the row.
        if mask & REQUIRED_POINT_FIELDS != REQUIRED_POINT_FIELDS
            || mask & !POINT_FIELD_CORE_MASK != 0
        {
            return Err(CollisionReplaySkip::PointFieldsUnverified { mask });
        }

        let key = (
            pair_key(body0).min(pair_key(body1)),
            pair_key(body0).max(pair_key(body1)),
        );
        if pairs.contains(&key) {
            return Err(CollisionReplaySkip::DuplicatePair {
                manifold: manifold_index,
            });
        }
        pairs.push(key);

        let mut points = Vec::with_capacity(record.points.len());
        for (point_index, source) in record.points.iter().enumerate() {
            points.push(point(manifold_index, point_index, source)?);
        }

        summary.covered_manifolds += 1;
        summary.covered_points += points.len();
        state.manifolds.push(CollisionReplayManifold {
            body0,
            body1,
            contact_breaking_threshold: record.contact_breaking_threshold,
            contact_processing_threshold: record.contact_processing_threshold,
            points,
        });
    }

    Ok(CollisionReplayExtract { state, summary })
}

/// Read the car availability flags of one row.
fn car_available(tick: &TickRecord) -> Vec<bool> {
    tick.car_records
        .iter()
        .take(MAX_CAR_SLOTS)
        .map(|car| car.is_available())
        .collect()
}

/// Select the covered manifolds of one row.
///
/// # Errors
///
/// Returns [`CollisionReplaySkip`] when the row has no usable capture.
pub fn extract_for_row(tick: &TickRecord) -> Result<CollisionReplayExtract, CollisionReplaySkip> {
    let snapshot = tick
        .replay_state
        .as_ref()
        .and_then(|ext| ext.collision_snapshot.as_ref());
    extract(snapshot, &car_available(tick))
}

/// Select the covered manifolds of one row with an explicit car availability
/// mask.
///
/// Pass the arena slot map when it can be more restrictive than the row, for
/// example a fixed partial roster. A manifold that names an unavailable slot
/// is then counted and skipped, so the row stays atomic.
///
/// # Errors
///
/// Returns the same values as [`extract`].
pub fn extract_for_row_with_availability(
    tick: &TickRecord,
    car_available: &[bool],
) -> Result<CollisionReplayExtract, CollisionReplaySkip> {
    let snapshot = tick
        .replay_state
        .as_ref()
        .and_then(|ext| ext.collision_snapshot.as_ref());
    extract(snapshot, car_available)
}

/// Select the manifold state that belongs to one recorded state row.
///
/// The collision group of row `N` is the state at the start of the step from
/// row `N` to row `N + 1`. Use row `N`, not row `N + 1`. The function is a
/// bounds-checked alias of `extract_for_row(&ticks[state_index])`.
///
/// # Errors
///
/// Returns [`CollisionReplaySkip::MissingRow`] when `state_index` is outside
/// the recording, and the other [`CollisionReplaySkip`] values from
/// [`extract_for_row`].
pub fn extract_for_state(
    ticks: &[TickRecord],
    state_index: usize,
) -> Result<CollisionReplayExtract, CollisionReplaySkip> {
    let tick = ticks
        .get(state_index)
        .ok_or(CollisionReplaySkip::MissingRow)?;
    extract_for_row(tick)
}

/// Residual below which two body poses cannot be separated, in Bullet units.
///
/// Two percent of the ball radius. A body that moves less than this between
/// rows gives no phase evidence.
const WITNESS_TIE_BT: f32 = 1.0e-2;

/// Relative margin below which two witness fits are indistinguishable.
///
/// A rotating body can make the current pose fit a stale witness slightly
/// better than the previous pose. Measured on the 2026-11-03 corpus, the
/// largest one-step turning case differs by one percent. Ten percent keeps
/// that case as ambiguous and still separates a real phase flip, which differs
/// by one order of magnitude.
const WITNESS_PHASE_RELATIVE_MARGIN: f32 = 0.1;

/// Which body pose a captured contact witness describes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WitnessPhase {
    /// The witness fits the previous row pose. This is the expected phase.
    ///
    /// The recorder samples before the game step, and a witness is only
    /// recomputed during a collision dispatch. The next dispatch refreshes the
    /// witness from the local anchor.
    PreviousStep,
    /// The witness fits the current row pose materially better than the
    /// previous row.
    ///
    /// This is a recorder-change signal: a capture after the step dispatch
    /// would look like this. It is a diagnostic, not a gate.
    CurrentStep,
    /// Both poses fit within [`WITNESS_TIE_BT`].
    Ambiguous,
    /// No point had both body poses.
    Unmeasurable,
}

impl Display for WitnessPhase {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::PreviousStep => "previous step",
            Self::CurrentStep => "current step",
            Self::Ambiguous => "ambiguous",
            Self::Unmeasurable => "unmeasurable",
        })
    }
}

/// Phase measurement over a set of rows.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WitnessPhaseReport {
    /// Rows with at least one measurable point.
    pub rows_checked: usize,
    /// Rows whose witness fits the previous row pose.
    pub previous_step: usize,
    /// Rows whose witness fits the current row pose better.
    pub current_step: usize,
    /// Rows that both poses fit.
    pub ambiguous: usize,
    /// Rows with no measurable point.
    pub unmeasurable: usize,
    /// Largest previous-row residual, in Bullet units.
    pub max_previous_residual_bt: f32,
    /// Largest current-row residual, in Bullet units.
    pub max_current_residual_bt: f32,
}

impl WitnessPhaseReport {
    /// True when no checked row fits the current pose materially better than
    /// the previous pose.
    ///
    /// Use it to report the recorder phase. Do not use it to reject a row.
    #[must_use]
    pub fn is_pre_transition(&self) -> bool {
        self.current_step == 0 && self.previous_step > 0
    }
}

impl Display for WitnessPhaseReport {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "phase previous {} current {} ambiguous {} unmeasurable {} of {}, residual previous {} current {}",
            self.previous_step,
            self.current_step,
            self.ambiguous,
            self.unmeasurable,
            self.rows_checked,
            self.max_previous_residual_bt,
            self.max_current_residual_bt
        )
    }
}

/// Body transform of one row, in Bullet units.
fn record_transform(tick: &TickRecord, body: ReplayBodyRef) -> Option<glam::Affine3A> {
    let phys: rocketsim::PhysState = match body {
        ReplayBodyRef::Ball => {
            if !tick.ball_available() {
                return None;
            }
            tick.ball_record.into()
        }
        ReplayBodyRef::Car { slot } => {
            let car = tick.car_records.get(usize::from(slot))?;
            if !car.is_available() {
                return None;
            }
            car.phys.into()
        }
    };
    Some(glam::Affine3A {
        matrix3: phys.rot_mat,
        translation: phys.pos * rocketsim::consts::UU_TO_BT,
    })
}

/// Mean witness residual of one state against two body pose rows.
///
/// Both values use the same point set: a point counts only when both pose rows
/// have a transform for both bodies.
fn witness_residuals(
    previous: &TickRecord,
    current: &TickRecord,
    state: &CollisionReplayState,
) -> Option<(f32, f32)> {
    let mut previous_sum = 0.0f32;
    let mut current_sum = 0.0f32;
    let mut count = 0usize;
    for manifold in &state.manifolds {
        let (Some(previous0), Some(previous1)) = (
            record_transform(previous, manifold.body0),
            record_transform(previous, manifold.body1),
        ) else {
            continue;
        };
        let (Some(current0), Some(current1)) = (
            record_transform(current, manifold.body0),
            record_transform(current, manifold.body1),
        ) else {
            continue;
        };
        for point in &manifold.points {
            previous_sum += previous0
                .transform_point3a(point.local_point_a)
                .distance(point.position_world_on_a)
                .max(
                    previous1
                        .transform_point3a(point.local_point_b)
                        .distance(point.position_world_on_b),
                );
            current_sum += current0
                .transform_point3a(point.local_point_a)
                .distance(point.position_world_on_a)
                .max(
                    current1
                        .transform_point3a(point.local_point_b)
                        .distance(point.position_world_on_b),
                );
            count += 1;
        }
    }
    (count > 0).then(|| (previous_sum / count as f32, current_sum / count as f32))
}

/// Measure which body pose the captured contact witnesses describe.
///
/// Run this once for a recording before you restore any row of it. The
/// recorder phase is a property of the recorder build, not of one row.
///
/// `max_rows` bounds the scan. Pass `usize::MAX` to check every row.
#[must_use]
pub fn classify_witness_phase(ticks: &[TickRecord], max_rows: usize) -> WitnessPhaseReport {
    let mut report = WitnessPhaseReport::default();
    for index in 1..ticks.len().min(max_rows) {
        let has_snapshot = ticks[index]
            .replay_state
            .as_ref()
            .and_then(|ext| ext.collision_snapshot.as_ref())
            .is_some();
        if !has_snapshot {
            continue;
        }
        let Ok(extracted) = extract_for_row(&ticks[index]) else {
            continue;
        };
        if extracted.summary.covered_points == 0 {
            continue;
        }

        let residuals = witness_residuals(&ticks[index - 1], &ticks[index], &extracted.state);
        let Some((previous, current)) = residuals else {
            report.unmeasurable += 1;
            report.rows_checked += 1;
            continue;
        };

        report.rows_checked += 1;
        report.max_previous_residual_bt = report.max_previous_residual_bt.max(previous);
        report.max_current_residual_bt = report.max_current_residual_bt.max(current);

        match classify_witness(previous, current) {
            WitnessPhase::PreviousStep => report.previous_step += 1,
            WitnessPhase::CurrentStep => report.current_step += 1,
            WitnessPhase::Ambiguous => report.ambiguous += 1,
            WitnessPhase::Unmeasurable => report.unmeasurable += 1,
        }
    }
    report
}

/// Classify one row from its two pose residuals.
fn classify_witness(previous: f32, current: f32) -> WitnessPhase {
    let small = previous <= WITNESS_TIE_BT && current <= WITNESS_TIE_BT;
    let indistinguishable =
        (previous - current).abs() <= previous.max(current) * WITNESS_PHASE_RELATIVE_MARGIN;
    if small || indistinguishable {
        WitnessPhase::Ambiguous
    } else if previous < current {
        WitnessPhase::PreviousStep
    } else {
        WitnessPhase::CurrentStep
    }
}

#[cfg(test)]
mod tests {
    use rocketsim_test::rlpr::{
        CollisionManifold, CollisionManifoldPoint, CollisionManifoldsSection,
        CollisionResolverTrace, CollisionSourcePath,
        collision_snapshot::{CollisionBodiesSection, CollisionSectionReport},
    };

    use super::*;

    /// Extract with every car slot available. Specific tests pass their own
    /// availability flags to [`extract`].
    fn extract(
        snapshot: Option<&CollisionSnapshot>,
    ) -> Result<CollisionReplayExtract, CollisionReplaySkip> {
        super::extract(snapshot, &[true; MAX_CAR_SLOTS])
    }

    fn body(catalog_index: i32, role: CollisionBodyRole, car_slot: u8) -> CollisionBody {
        CollisionBody {
            catalog_index,
            role,
            car_slot,
            ..CollisionBody::default()
        }
    }

    fn point(offset: f32) -> CollisionManifoldPoint {
        CollisionManifoldPoint {
            local_point_a: Some([offset, 0.0, 0.0]),
            local_point_b: Some([offset, 0.0, 0.0]),
            position_world_on_a: Some([offset, 0.0, 1.0]),
            position_world_on_b: Some([offset, 0.0, 0.9]),
            normal_world_on_b: Some([0.0, 0.0, 1.0]),
            distance1: Some(-0.1),
            lifetime: Some(3),
            ..CollisionManifoldPoint::default()
        }
    }

    fn manifold(body0: i32, body1: i32, points: Vec<CollisionManifoldPoint>) -> CollisionManifold {
        CollisionManifold {
            body0_catalog_index: body0,
            body1_catalog_index: body1,
            contact_breaking_threshold: 0.02,
            contact_processing_threshold: 1.0e30,
            point_count: points.len() as u8,
            non_positive_distance_point_count: points.len() as u8,
            point_field_mask: POINT_FIELD_CORE_MASK,
            points,
            ..CollisionManifold::default()
        }
    }

    fn snapshot(
        bodies: Vec<CollisionBody>,
        manifolds: Vec<CollisionManifold>,
        order: CollisionManifoldOrder,
    ) -> CollisionSnapshot {
        CollisionSnapshot {
            schema_version: SCHEMA_VERSION,
            requested_sections: REQUIRED_SECTIONS,
            verified_sections: REQUIRED_SECTIONS,
            build_identified: true,
            layout_id: CollisionLayoutId::Current20261102,
            source_path: CollisionSourcePath::EngineSceneChain,
            section_count: 2,
            resolver: CollisionResolverTrace::default(),
            bodies: Some(CollisionBodiesSection {
                report: CollisionSectionReport {
                    status: CollisionSectionStatus::Ok,
                    ..CollisionSectionReport::default()
                },
                records: bodies,
            }),
            manifolds: Some(CollisionManifoldsSection {
                report: CollisionSectionReport {
                    status: CollisionSectionStatus::Ok,
                    ..CollisionSectionReport::default()
                },
                order,
                records: manifolds,
                ..CollisionManifoldsSection::default()
            }),
            ..CollisionSnapshot::default()
        }
    }

    fn roster() -> Vec<CollisionBody> {
        vec![
            body(0, CollisionBodyRole::Ball, 0xFF),
            body(1, CollisionBodyRole::Car, 0),
            body(2, CollisionBodyRole::Car, 1),
            body(3, CollisionBodyRole::WorldStatic, 0xFF),
        ]
    }

    #[test]
    fn covered_manifolds_keep_stored_order() {
        let capture = snapshot(
            roster(),
            vec![
                manifold(1, 0, vec![point(0.0), point(0.5)]),
                manifold(2, 1, vec![point(0.0)]),
            ],
            CollisionManifoldOrder::DispatcherStored,
        );

        let extract = extract(Some(&capture)).unwrap();

        assert_eq!(extract.state.manifolds.len(), 2);
        assert_eq!(
            extract.state.manifolds[0].body0,
            ReplayBodyRef::Car { slot: 0 }
        );
        assert_eq!(extract.state.manifolds[0].body1, ReplayBodyRef::Ball);
        assert_eq!(extract.state.manifolds[0].points.len(), 2);
        assert_eq!(
            extract.state.manifolds[1].body0,
            ReplayBodyRef::Car { slot: 1 }
        );
        assert_eq!(
            extract.state.manifolds[1].body1,
            ReplayBodyRef::Car { slot: 0 }
        );
        assert_eq!(extract.summary.native_manifolds, 2);
        assert_eq!(extract.summary.covered_manifolds, 2);
        assert_eq!(extract.summary.native_points, 3);
        assert_eq!(extract.summary.covered_points, 3);
        assert!(extract.summary.is_complete_manifold_set());
    }

    #[test]
    fn static_manifolds_are_counted_not_rebuilt() {
        let capture = snapshot(
            roster(),
            vec![
                manifold(1, 3, vec![point(0.0)]),
                manifold(0, 3, vec![]),
                manifold(1, 0, vec![point(0.0)]),
            ],
            CollisionManifoldOrder::DispatcherStored,
        );

        let extract = extract(Some(&capture)).unwrap();

        assert_eq!(extract.state.manifolds.len(), 1);
        assert_eq!(extract.summary.native_manifolds, 3);
        assert_eq!(extract.summary.covered_manifolds, 1);
        assert_eq!(extract.summary.skipped_manifolds, 2);
        assert!(!extract.summary.is_complete_manifold_set());
    }

    #[test]
    fn incomplete_or_ambiguous_captures_are_refused() {
        let good = snapshot(
            roster(),
            vec![manifold(1, 0, vec![point(0.0)])],
            CollisionManifoldOrder::DispatcherStored,
        );
        assert!(extract(Some(&good)).is_ok());
        assert_eq!(extract(None), Err(CollisionReplaySkip::NoSnapshot));

        let mut unknown_schema = good.clone();
        unknown_schema.schema_version = 2;
        assert_eq!(
            extract(Some(&unknown_schema)),
            Err(CollisionReplaySkip::UnknownSchema { version: 2 })
        );

        let mut other_layout = good.clone();
        other_layout.layout_id = CollisionLayoutId::Ecbc20261019;
        assert_eq!(
            extract(Some(&other_layout)),
            Err(CollisionReplaySkip::UnverifiedLayout {
                layout: CollisionLayoutId::Ecbc20261019
            })
        );

        let mut unresolved = good.clone();
        unresolved.build_identified = false;
        assert_eq!(
            extract(Some(&unresolved)),
            Err(CollisionReplaySkip::UnidentifiedBuild)
        );

        let mut partial = good.clone();
        if let Some(section) = partial.manifolds.as_mut() {
            section.report.status = CollisionSectionStatus::Partial;
        }
        assert_eq!(
            extract(Some(&partial)),
            Err(CollisionReplaySkip::ManifoldsIncomplete)
        );

        let mut solve_order = good.clone();
        if let Some(section) = solve_order.manifolds.as_mut() {
            section.order = CollisionManifoldOrder::SolveInput;
        }
        assert_eq!(
            extract(Some(&solve_order)),
            Err(CollisionReplaySkip::NotDispatcherOrder {
                order: CollisionManifoldOrder::SolveInput
            })
        );

        let mut missing_field = good.clone();
        if let Some(section) = missing_field.manifolds.as_mut() {
            section.records[0].point_field_mask = POINT_FIELD_LOCAL_A;
        }
        assert_eq!(
            extract(Some(&missing_field)),
            Err(CollisionReplaySkip::PointFieldsUnverified {
                mask: POINT_FIELD_LOCAL_A
            })
        );

        let mut unverified_field = good.clone();
        if let Some(section) = unverified_field.manifolds.as_mut() {
            section.records[0].point_field_mask = POINT_FIELD_CORE_MASK | (1 << 9);
        }
        assert_eq!(
            extract(Some(&unverified_field)),
            Err(CollisionReplaySkip::PointFieldsUnverified {
                mask: POINT_FIELD_CORE_MASK | (1 << 9)
            })
        );

        let mut duplicate_pair = good.clone();
        if let Some(section) = duplicate_pair.manifolds.as_mut() {
            section.records.push(manifold(0, 1, vec![]));
        }
        assert_eq!(
            extract(Some(&duplicate_pair)),
            Err(CollisionReplaySkip::DuplicatePair { manifold: 1 })
        );

        let mut two_balls = good.clone();
        if let Some(section) = two_balls.bodies.as_mut() {
            section.records.push(body(4, CollisionBodyRole::Ball, 0xFF));
        }
        assert_eq!(
            extract(Some(&two_balls)),
            Err(CollisionReplaySkip::AmbiguousBall)
        );

        let mut unknown_body = good.clone();
        if let Some(section) = unknown_body.manifolds.as_mut() {
            section.records[0].body1_catalog_index = 9;
        }
        assert_eq!(
            extract(Some(&unknown_body)),
            Err(CollisionReplaySkip::UnknownCatalogIndex { catalog_index: 9 })
        );

        let mut clamped = good.clone();
        clamped.limits_clamped = true;
        assert_eq!(
            extract(Some(&clamped)),
            Err(CollisionReplaySkip::LimitsClamped)
        );

        let mut unverified_section = good.clone();
        unverified_section.verified_sections = SECTION_BODIES;
        assert_eq!(
            extract(Some(&unverified_section)),
            Err(CollisionReplaySkip::SectionNotVerified {
                verified: SECTION_BODIES
            })
        );
    }

    #[test]
    fn unavailable_car_slot_is_counted_not_refused() {
        let capture = snapshot(
            roster(),
            vec![manifold(1, 0, vec![point(0.0)]), manifold(2, 1, vec![])],
            CollisionManifoldOrder::DispatcherStored,
        );
        let mut available = vec![true; MAX_CAR_SLOTS];
        available[1] = false;
        let extract = super::extract(Some(&capture), &available).unwrap();
        assert_eq!(extract.state.manifolds.len(), 1);
        assert_eq!(extract.state.manifolds[0].body1, ReplayBodyRef::Ball);
        assert_eq!(extract.summary.skipped_manifolds, 1);
        assert_eq!(extract.summary.skipped_unavailable_manifolds, 1);
        assert_eq!(extract.summary.skipped_unmappable_manifolds, 0);
    }

    #[test]
    fn shared_uncatalogued_bodies_are_not_duplicates() {
        // Soccar world statics share catalog index -1. A car-world manifold
        // must be skipped, not refused.
        let mut bodies = roster();
        bodies.push(body(-1, CollisionBodyRole::WorldStatic, 0xFF));
        bodies.push(body(-1, CollisionBodyRole::WorldStatic, 0xFF));
        let capture = snapshot(
            bodies,
            vec![manifold(1, -1, vec![]), manifold(2, -1, vec![])],
            CollisionManifoldOrder::DispatcherStored,
        );
        let recovered = extract(Some(&capture)).unwrap();
        assert!(recovered.state.manifolds.is_empty());
        assert_eq!(recovered.summary.skipped_manifolds, 2);

        // A repeated catalogued index is still refused.
        let mut duplicate = roster();
        duplicate.push(body(1, CollisionBodyRole::Car, 2));
        let capture = snapshot(duplicate, vec![], CollisionManifoldOrder::DispatcherStored);
        assert_eq!(
            extract(Some(&capture)),
            Err(CollisionReplaySkip::DuplicateCatalogIndex { catalog_index: 1 })
        );
    }

    #[test]
    fn point_count_and_field_checks_are_per_manifold() {
        let mut capture = snapshot(
            roster(),
            vec![manifold(1, 0, vec![point(0.0)])],
            CollisionManifoldOrder::DispatcherStored,
        );
        if let Some(section) = capture.manifolds.as_mut() {
            section.records[0].point_count = 2;
        }
        assert_eq!(
            extract(Some(&capture)),
            Err(CollisionReplaySkip::PointCountMismatch { manifold: 0 })
        );

        let mut capture = snapshot(
            roster(),
            vec![manifold(1, 0, vec![point(0.0)])],
            CollisionManifoldOrder::DispatcherStored,
        );
        if let Some(section) = capture.manifolds.as_mut() {
            section.records[0].points[0].distance1 = None;
        }
        assert_eq!(
            extract(Some(&capture)),
            Err(CollisionReplaySkip::PointFieldMissing {
                manifold: 0,
                point: 0
            })
        );
    }

    #[test]
    fn witness_phase_classification_follows_the_recorder_order() {
        // The recorder samples before the game step. A captured witness still
        // describes the previous row pose.
        assert_eq!(classify_witness(1.0e-4, 0.3), WitnessPhase::PreviousStep);
        // A capture after the step dispatch would fit the current pose better.
        assert_eq!(classify_witness(0.3, 1.0e-4), WitnessPhase::CurrentStep);
        // A body that does not move gives no evidence either way.
        assert_eq!(classify_witness(1.0e-5, 2.0e-5), WitnessPhase::Ambiguous);
        assert_eq!(classify_witness(0.12, 0.7), WitnessPhase::PreviousStep);
        // A turning body can make the current pose fit a stale witness by a
        // hair. That is not a phase flip.
        assert_eq!(classify_witness(0.3531, 0.3492), WitnessPhase::Ambiguous);
        // A real flip differs by an order of magnitude.
        assert_eq!(classify_witness(0.45, 1.0e-3), WitnessPhase::CurrentStep);
    }

    #[test]
    fn only_the_previous_step_phase_is_pre_transition() {
        let previous = WitnessPhaseReport {
            rows_checked: 10,
            previous_step: 10,
            ..WitnessPhaseReport::default()
        };
        assert!(previous.is_pre_transition());

        let mixed = WitnessPhaseReport {
            rows_checked: 10,
            previous_step: 9,
            current_step: 1,
            ..WitnessPhaseReport::default()
        };
        assert!(!mixed.is_pre_transition());

        let ambiguous = WitnessPhaseReport {
            rows_checked: 10,
            ambiguous: 10,
            ..WitnessPhaseReport::default()
        };
        assert!(!ambiguous.is_pre_transition());

        assert!(previous.to_string().contains("phase previous 10"));
    }

    #[test]
    fn unverified_mask_on_a_skipped_manifold_does_not_refuse_the_row() {
        // A static manifold with an unproven mask cannot refuse the row,
        // because the restore never builds it.
        let mut capture = snapshot(
            roster(),
            vec![
                manifold(1, 3, vec![point(0.0)]),
                manifold(1, 0, vec![point(0.0)]),
            ],
            CollisionManifoldOrder::DispatcherStored,
        );
        if let Some(section) = capture.manifolds.as_mut() {
            section.records[0].point_field_mask = 1 << 9;
        }
        let extracted = extract(Some(&capture)).unwrap();
        assert_eq!(extracted.state.manifolds.len(), 1);
        assert_eq!(extracted.summary.skipped_unmappable_manifolds, 1);

        // The same mask on a covered manifold still refuses the row.
        let mut capture = snapshot(
            roster(),
            vec![manifold(1, 0, vec![point(0.0)])],
            CollisionManifoldOrder::DispatcherStored,
        );
        if let Some(section) = capture.manifolds.as_mut() {
            section.records[0].point_field_mask = 1 << 9;
        }
        assert_eq!(
            extract(Some(&capture)),
            Err(CollisionReplaySkip::PointFieldsUnverified { mask: 1 << 9 })
        );
    }

    #[test]
    fn state_row_must_exist() {
        assert_eq!(
            extract_for_state(&[], 0),
            Err(CollisionReplaySkip::MissingRow)
        );
    }

    #[test]
    fn skip_messages_name_the_reason() {
        assert!(
            CollisionReplaySkip::NotDispatcherOrder {
                order: CollisionManifoldOrder::SolveInput
            }
            .to_string()
            .contains("stored order")
        );
        assert_eq!(
            CollisionReplaySkip::LimitsClamped.to_string(),
            "a collision limit clamped a native count"
        );
        assert_eq!(
            CollisionReplaySkip::UnknownCatalogIndex { catalog_index: 7 }.to_string(),
            "catalog index 7 has no body"
        );
    }
}
