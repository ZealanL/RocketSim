//! Restore of native persistent collision manifolds for a replay.
//!
//! A replay stores a read-only view of the native Bullet collision state. This
//! module owns the replay policy. It selects the part of that view that maps
//! to RocketSim bodies without a guess:
//!
//! - the retained contact point geometry of a manifold whose two bodies are
//!   the ball and a car, or two cars;
//! - the stored contact point order in each manifold.
//!
//! The arena only upserts the manifolds that this module supplies. This module
//! decides which pairs are owned, which coefficients they use, and which
//! stored thresholds they keep.
//!
//! # Restore scope
//!
//! A restored manifold holds these replay values:
//!
//! - local contact point A and B;
//! - world contact point A and B;
//! - contact normal on body B;
//! - signed contact distance.
//!
//! The combined friction and restitution come from the RocketSim car contact
//! constants. They equal the values that the normal contact callback writes
//! for the same body pair. The replay does not set them.
//!
//! These values stay unchanged:
//!
//! - broadphase handles, edge arrays, and the pair array. The RocketSim
//!   broadphase has its own handle allocator and its own pair order.
//! - manifolds that touch a static world body. A replay has no static shape
//!   identity and no triangle part or index for these manifolds.
//! - the solver warm-start impulse of each point. A replay does not store it.
//!   A restored point starts at zero, the value that RocketSim also gives to
//!   a new point.
//! - the contact lifetime. RocketSim does not keep this value.
//!
//! A new body pair uses the smaller contact breaking threshold and the smaller
//! contact processing threshold of its two bodies. A pair that the arena
//! already holds keeps its stored thresholds. The captured thresholds are
//! diagnostics and are never written.
//!
//! # Units
//!
//! All values use native Bullet units. The restore copies them without
//! conversion. Local points use the body frame. World points and the normal
//! use the world frame. The normal is a unit vector. A positive distance is a
//! separating contact.
//!
//! # Order and ownership
//!
//! The restore owns contacts whose two bodies are the ball and a mapped car,
//! or two mapped cars. It replaces the points of an owned pair in place. An
//! owned pair absent from the replay keeps its slot with an empty point cache.
//! A new pair is appended. An empty replay clears all owned contact points.
//!
//! World contacts are not owned. Their values and indices do not change.
//! Existing indices must stay fixed because the solver uses them to order
//! equal-island-key contacts. The native dispatcher order is not restored.
//!
//! A fresh dispatch fixes the order of the two bodies. A ball and car
//! manifold is `(car, ball)`. A car and car manifold follows the broadphase
//! pair, which puts the smaller proxy unique id first. The restore applies the
//! same order. A reversed replay swaps the two local anchors and the two world
//! witnesses, negates the normal on body B, and keeps the signed distance and
//! the contact coefficients.
//!
//! # Validation
//!
//! The restore checks the slot table, the body pair, the thresholds, and every
//! point before the first write. It writes nothing when it returns an error.
//! See [`CollisionReplayError`].
//!
//! # Witness timing
//!
//! Bullet recomputes a contact world witness only during a collision dispatch.
//! A captured witness can describe an older body pose than the captured body
//! state. This is correct state, not an error: the next dispatch refreshes the
//! witness from the local anchor in both engines.
//!
//! The restore writes the local anchors and the normal. A refreshed point uses
//! those values. The captured world points and distance are copied as stored.
//! Use [`CollisionReplayReport::max_witness_residual_bt`] to read the
//! difference between a captured witness and the same witness rebuilt from the
//! local anchor and the current body pose.
//!
//! A restore is exact only for the listed fields. Do not treat it as a
//! complete native collision checkpoint.
//!
//! Use [`CollisionReplayReport`] to read the cleared and written counts.

use std::collections::{HashMap, HashSet};

use glam::Vec3A;
use rocketsim::{
    Arena, UserInfoType, consts,
    physics::{BodyId, BodyInfo, ContactManifold, ContactPoint, MAX_CONTACT_POINTS, StateError},
};

/// Largest accepted difference between two contact breaking thresholds, in
/// Bullet units.
const THRESHOLD_TOLERANCE_BT: f32 = 1.0e-6;

/// Extra margin for the local-anchor containment check, in Bullet units.
///
/// The margin covers the float error of a rebuild. The check adds the
/// manifold contact breaking threshold to this value.
const CONTAINMENT_EPSILON_BT: f32 = 1.0e-4;

/// One body of a restored manifold, as a recorded car slot or the ball.
///
/// A replay records at most eight car slots.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ReplayBodyRef {
    /// The shared ball.
    Ball,
    /// One recorded car slot.
    Car {
        /// Recorded car slot in `0..=7`.
        slot: u8,
    },
}

/// One retained native contact point.
///
/// All values use native Bullet units. The values are copied without
/// conversion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollisionReplayPoint {
    /// Contact point in the local frame of body A.
    pub local_point_a: Vec3A,
    /// Contact point in the local frame of body B.
    pub local_point_b: Vec3A,
    /// Contact point in the world frame on body A.
    pub position_world_on_a: Vec3A,
    /// Contact point in the world frame on body B.
    pub position_world_on_b: Vec3A,
    /// Unit contact normal on body B.
    pub normal_world_on_b: Vec3A,
    /// Signed contact distance. A positive value is a separating contact.
    pub distance: f32,
}

impl CollisionReplayPoint {
    /// True when every value is finite.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.local_point_a.is_finite()
            && self.local_point_b.is_finite()
            && self.position_world_on_a.is_finite()
            && self.position_world_on_b.is_finite()
            && self.normal_world_on_b.is_finite()
            && self.distance.is_finite()
    }
}

/// One native persistent manifold for one ordered body pair.
#[derive(Clone, Debug, PartialEq)]
pub struct CollisionReplayManifold {
    /// Body A. The contact normal points from body B to body A.
    pub body0: ReplayBodyRef,
    /// Body B.
    pub body1: ReplayBodyRef,
    /// Captured contact breaking threshold, in Bullet units.
    ///
    /// The restore uses this value as a diagnostic. A new pair uses the
    /// smaller body threshold. A pair that the arena already holds keeps its
    /// stored threshold.
    pub contact_breaking_threshold: f32,
    /// Captured contact processing threshold, in Bullet units.
    ///
    /// The restore does not write this value. A new pair uses the smaller
    /// body threshold. A pair that the arena already holds keeps its stored
    /// threshold.
    pub contact_processing_threshold: f32,
    /// Retained contact points in stored order. The limit is four.
    pub points: Vec<CollisionReplayPoint>,
}

/// The manifolds that a replay restores, in stored dispatcher order.
///
/// The state holds only manifolds whose two bodies are the ball and a car, or
/// two cars. Manifolds for other body pairs are absent, not empty.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionReplayState {
    /// Covered manifolds in stored dispatcher order.
    pub manifolds: Vec<CollisionReplayManifold>,
}

impl CollisionReplayState {
    /// True when the state restores no manifold.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.manifolds.is_empty()
    }

    /// Total number of contact points in the state.
    #[must_use]
    pub fn point_count(&self) -> usize {
        self.manifolds
            .iter()
            .map(|manifold| manifold.points.len())
            .sum()
    }
}

/// The body pairs that one restore owns.
struct SupportedUniverse {
    ball: BodyId,
    slot_bodies: Vec<Option<BodyId>>,
    cars: Vec<BodyId>,
}

impl SupportedUniverse {
    /// Resolve one recorded body to an arena body.
    fn body_id(&self, body: ReplayBodyRef) -> Option<BodyId> {
        match body {
            ReplayBodyRef::Ball => Some(self.ball),
            ReplayBodyRef::Car { slot } => {
                self.slot_bodies.get(usize::from(slot)).copied().flatten()
            }
        }
    }

    /// True when both bodies are the ball and a mapped car, or two mapped
    /// cars.
    fn contains_pair(&self, body0: BodyId, body1: BodyId) -> bool {
        if body0 == body1 {
            return false;
        }
        let ball0 = body0 == self.ball;
        let ball1 = body1 == self.ball;
        match (ball0, ball1) {
            (true, false) => self.cars.contains(&body1),
            (false, true) => self.cars.contains(&body0),
            (false, false) => self.cars.contains(&body0) && self.cars.contains(&body1),
            (true, true) => false,
        }
    }
}

/// Result of one collision replay restore.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CollisionReplayReport {
    /// Number of captured manifolds written.
    ///
    /// The count includes manifolds replaced in place and manifolds appended
    /// for a pair that the dispatcher did not hold.
    pub manifolds_installed: usize,
    /// Number of contact points written.
    pub points_installed: usize,
    /// Number of existing owned manifolds that the restore replaced or
    /// emptied.
    ///
    /// The count is [`Self::manifolds_replaced`] plus
    /// [`Self::manifolds_emptied`]. A replacement counts even when the
    /// supplied manifold has no points or the existing cache was already
    /// empty. No manifold is removed from the dispatcher, because removal
    /// moves other indices and changes the solver's equal-island-key order.
    pub manifolds_cleared: usize,
    /// Number of captured manifolds that replaced an existing manifold in
    /// place. The dispatcher index is unchanged.
    pub manifolds_replaced: usize,
    /// Number of captured manifolds appended for a pair that the dispatcher
    /// did not hold.
    pub manifolds_appended: usize,
    /// Number of owned manifolds that the capture does not list and whose
    /// retained contacts were cleared.
    ///
    /// The manifold stays in the dispatcher with an empty point cache. Its
    /// index stays valid, so non-restored manifolds keep their order.
    pub manifolds_emptied: usize,
    /// Number of captured manifolds whose body order was reversed to match a
    /// fresh RocketSim collision dispatch.
    pub manifolds_reoriented: usize,
    /// Number of covered manifolds whose captured contact breaking threshold
    /// differs from the fresh per-body minimum.
    pub threshold_mismatches: usize,
    /// Number of retained points whose captured signed distance is above the
    /// captured contact breaking threshold.
    ///
    /// The value is a diagnostic. A positive signed distance is a valid
    /// separating contact, and the restore does not drop such a point. The
    /// next RocketSim dispatch applies its own refresh rule.
    pub points_above_breaking_threshold: usize,
    /// Number of retained points whose local anchor maps outside its body
    /// AABB, expanded by the contact breaking threshold.
    ///
    /// The value is a diagnostic. The restore does not drop such a point.
    pub points_outside_body: usize,
    /// Largest local-anchor overshoot of its body AABB margin, in Bullet
    /// units.
    ///
    /// The value is a diagnostic. It catches a unit or scale error in the
    /// captured anchors. It is not a gate.
    pub max_anchor_overshoot_bt: f32,
    /// Largest difference between a captured world witness and the same
    /// witness rebuilt from the local anchor and the current body pose, in
    /// Bullet units.
    ///
    /// A witness is only recomputed during a collision dispatch. The expected
    /// value is therefore in the range of one step of body motion. The value
    /// is a diagnostic. It is not a gate.
    pub max_witness_residual_bt: f32,
}

/// A restore failure. The arena is unchanged when `apply` returns an error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionReplayError {
    /// A car slot has no arena body.
    UnmappedBody {
        /// Index of the manifold in the state.
        manifold: usize,
        /// The body that has no arena index.
        body: ReplayBodyRef,
    },
    /// Both sides of a manifold resolve to the same body.
    SameBodyPair {
        /// Index of the manifold in the state.
        manifold: usize,
    },
    /// The state holds the same body pair more than once.
    DuplicateBodyPair {
        /// Index of the second manifold in the state.
        manifold: usize,
    },
    /// The body pair is not a ball and car pair or a car and car pair.
    UnsupportedBodyPair {
        /// Index of the manifold in the state.
        manifold: usize,
    },
    /// A manifold holds more points than the RocketSim point cache.
    TooManyPoints {
        /// Index of the manifold in the state.
        manifold: usize,
        /// Number of points in the state.
        points: usize,
    },
    /// A point holds a NaN or an infinite value.
    NonFinitePoint {
        /// Index of the manifold in the state.
        manifold: usize,
        /// Index of the point in the manifold.
        point: usize,
    },
    /// A point normal is not a unit vector.
    NormalNotUnit {
        /// Index of the manifold in the state.
        manifold: usize,
        /// Index of the point in the manifold.
        point: usize,
    },
    /// The ball body is not the ball.
    BallSlotNotBall {
        /// The body index.
        body_index: usize,
    },
    /// A car slot names an arena car index that does not exist.
    CarSlotOutOfRange {
        /// Recorded car slot.
        slot: usize,
        /// The car index.
        body_index: usize,
    },
    /// A car slot body is not a car.
    CarSlotNotCar {
        /// Recorded car slot.
        slot: usize,
        /// The body index.
        body_index: usize,
    },
    /// One body is used by the ball and a car slot, or by two car slots.
    DuplicateSlotBody {
        /// The repeated body index.
        body_index: usize,
    },
    /// A captured contact breaking threshold is not a finite value at or
    /// above zero.
    InvalidBreakingThreshold {
        /// Index of the manifold in the state.
        manifold: usize,
    },
    /// A captured contact processing threshold is not a finite value at or
    /// above zero.
    InvalidProcessingThreshold {
        /// Index of the manifold in the state.
        manifold: usize,
    },
    /// The physics body API refused a query or a manifold update.
    Physics(StateError),
}

impl std::fmt::Display for CollisionReplayError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnmappedBody { manifold, body } => {
                write!(
                    formatter,
                    "manifold {manifold} body {body:?} has no arena body"
                )
            }
            Self::SameBodyPair { manifold } => {
                write!(formatter, "manifold {manifold} uses one body twice")
            }
            Self::DuplicateBodyPair { manifold } => {
                write!(formatter, "manifold {manifold} repeats a body pair")
            }
            Self::UnsupportedBodyPair { manifold } => write!(
                formatter,
                "manifold {manifold} body pair is not a ball and car pair or a car and car pair"
            ),
            Self::TooManyPoints { manifold, points } => write!(
                formatter,
                "manifold {manifold} holds {points} points, limit {MAX_CONTACT_POINTS}"
            ),
            Self::NonFinitePoint { manifold, point } => {
                write!(formatter, "manifold {manifold} point {point} is not finite")
            }
            Self::NormalNotUnit { manifold, point } => {
                write!(
                    formatter,
                    "manifold {manifold} point {point} normal is not a unit vector"
                )
            }
            Self::BallSlotNotBall { body_index } => {
                write!(formatter, "ball body index {body_index} is not the ball")
            }
            Self::CarSlotOutOfRange { slot, body_index } => write!(
                formatter,
                "car slot {slot} body index {body_index} is outside the car array"
            ),
            Self::CarSlotNotCar { slot, body_index } => {
                write!(
                    formatter,
                    "car slot {slot} body index {body_index} is not a car"
                )
            }
            Self::DuplicateSlotBody { body_index } => {
                write!(
                    formatter,
                    "body index {body_index} is used by more than one slot"
                )
            }
            Self::InvalidBreakingThreshold { manifold } => write!(
                formatter,
                "manifold {manifold} contact breaking threshold is not finite and non-negative"
            ),
            Self::InvalidProcessingThreshold { manifold } => write!(
                formatter,
                "manifold {manifold} contact processing threshold is not finite and non-negative"
            ),
            Self::Physics(error) => {
                write!(
                    formatter,
                    "physics body API refused the contact update: {error}"
                )
            }
        }
    }
}

impl std::error::Error for CollisionReplayError {}

/// Read one body description and wrap a physics error.
fn body_info(arena: &Arena, body: BodyId) -> Result<BodyInfo, CollisionReplayError> {
    arena.body_info(body).map_err(CollisionReplayError::Physics)
}

/// Resolve the recorded car slot table against the arena.
///
/// The check runs before any write, so an empty state cannot skip it.
fn resolve_slots(
    arena: &Arena,
    car_slots: &[Option<usize>],
) -> Result<SupportedUniverse, CollisionReplayError> {
    let ball = arena.ball_body_id();
    let ball_info = body_info(arena, ball)?;
    if ball_info.kind != UserInfoType::Ball {
        return Err(CollisionReplayError::BallSlotNotBall {
            body_index: ball.index(),
        });
    }

    let mut slot_bodies = Vec::with_capacity(car_slots.len());
    let mut cars: Vec<BodyId> = Vec::with_capacity(car_slots.len());
    for (slot, car_idx) in car_slots.iter().enumerate() {
        let Some(car_idx) = *car_idx else {
            slot_bodies.push(None);
            continue;
        };
        let body = arena
            .car_body_id(car_idx)
            .ok_or(CollisionReplayError::CarSlotOutOfRange {
                slot,
                body_index: car_idx,
            })?;
        let info = body_info(arena, body)?;
        if info.kind != UserInfoType::Car {
            return Err(CollisionReplayError::CarSlotNotCar {
                slot,
                body_index: body.index(),
            });
        }
        if body == ball || cars.contains(&body) {
            return Err(CollisionReplayError::DuplicateSlotBody {
                body_index: body.index(),
            });
        }
        cars.push(body);
        slot_bodies.push(Some(body));
    }

    Ok(SupportedUniverse {
        ball,
        slot_bodies,
        cars,
    })
}

/// Distance from a local anchor to its body volume, in Bullet units.
///
/// A value of zero means the anchor is inside the body AABB expanded by
/// `margin`. A positive value is the distance outside that expanded AABB.
/// The check uses the body AABB at the current pose and the local anchor. It
/// does not use a captured world witness.
fn local_point_overshoot(info: &BodyInfo, local: Vec3A, margin: f32) -> f32 {
    if !local.is_finite() {
        return f32::INFINITY;
    }
    let margin = Vec3A::splat(margin);
    let point = info.world_point(local);
    let closest = point.clamp(info.aabb.min - margin, info.aabb.max + margin);
    point.distance(closest)
}

/// True when the two bodies must swap to match a fresh dispatch order.
///
/// A fresh ball and car manifold is `(car, ball)`, because the car body is the
/// compound body. For a car and car pair, the broadphase pair keeps the
/// smaller proxy unique id first. The unique id is the body collision order.
///
/// A swapped manifold must swap the two local anchors and the two world
/// witnesses, and negate the normal on body B. The signed distance and the
/// contact coefficients do not change.
///
/// This function assumes that every arena car has a compound chassis body. It
/// does not inspect the shape. A car with a noncompound chassis would change
/// the fresh order.
fn needs_dispatch_swap(body0: &BodyInfo, body1: &BodyInfo) -> bool {
    let compound0 = body0.kind == UserInfoType::Car;
    let compound1 = body1.kind == UserInfoType::Car;
    match (compound0, compound1) {
        // Broadphase pair order: ascending unique id.
        (true, true) => body0.collision_order > body1.collision_order,
        // The compound body is body 0.
        (false, true) => true,
        (true, false) | (false, false) => false,
    }
}

/// Contact coefficients that the car contact callback writes for one pair.
fn contact_coefs(body0: ReplayBodyRef, body1: ReplayBodyRef) -> Option<(f32, f32)> {
    let coefs = match (body0, body1) {
        (ReplayBodyRef::Ball, ReplayBodyRef::Car { .. })
        | (ReplayBodyRef::Car { .. }, ReplayBodyRef::Ball) => consts::car::HIT_BALL_COEFS,
        (ReplayBodyRef::Car { .. }, ReplayBodyRef::Car { .. }) => consts::car::HIT_CAR_COEFS,
        _ => return None,
    };
    Some((coefs.friction, coefs.restitution))
}

/// The thresholds of a fresh manifold for one body pair.
fn fresh_thresholds(body0: &BodyInfo, body1: &BodyInfo) -> (f32, f32) {
    (
        body0
            .contact_breaking_threshold
            .min(body1.contact_breaking_threshold),
        body0
            .contact_processing_threshold
            .min(body1.contact_processing_threshold),
    )
}

/// Order a body pair for lookup and comparison.
fn ordered_pair(body0: BodyId, body1: BodyId) -> (BodyId, BodyId) {
    if body0 <= body1 {
        (body0, body1)
    } else {
        (body1, body0)
    }
}

/// Build every captured manifold in fresh dispatch order.
///
/// The call also collects the replay diagnostics. It writes nothing.
fn build_manifolds(
    arena: &Arena,
    universe: &SupportedUniverse,
    state: &CollisionReplayState,
) -> Result<(Vec<ContactManifold>, CollisionReplayReport), CollisionReplayError> {
    let mut report = CollisionReplayReport::default();
    let mut built = Vec::with_capacity(state.manifolds.len());
    let mut pairs: Vec<(BodyId, BodyId)> = Vec::with_capacity(state.manifolds.len());

    for (manifold_index, manifold_state) in state.manifolds.iter().enumerate() {
        if !manifold_state.contact_breaking_threshold.is_finite()
            || manifold_state.contact_breaking_threshold < 0.0
        {
            return Err(CollisionReplayError::InvalidBreakingThreshold {
                manifold: manifold_index,
            });
        }
        if !manifold_state.contact_processing_threshold.is_finite()
            || manifold_state.contact_processing_threshold < 0.0
        {
            return Err(CollisionReplayError::InvalidProcessingThreshold {
                manifold: manifold_index,
            });
        }

        let (friction, restitution) = contact_coefs(manifold_state.body0, manifold_state.body1)
            .ok_or(CollisionReplayError::UnsupportedBodyPair {
                manifold: manifold_index,
            })?;

        let body_id0 =
            universe
                .body_id(manifold_state.body0)
                .ok_or(CollisionReplayError::UnmappedBody {
                    manifold: manifold_index,
                    body: manifold_state.body0,
                })?;
        let body_id1 =
            universe
                .body_id(manifold_state.body1)
                .ok_or(CollisionReplayError::UnmappedBody {
                    manifold: manifold_index,
                    body: manifold_state.body1,
                })?;

        if body_id0 == body_id1 {
            return Err(CollisionReplayError::SameBodyPair {
                manifold: manifold_index,
            });
        }

        let pair = ordered_pair(body_id0, body_id1);
        if pairs.contains(&pair) {
            return Err(CollisionReplayError::DuplicateBodyPair {
                manifold: manifold_index,
            });
        }
        pairs.push(pair);

        if manifold_state.points.len() > MAX_CONTACT_POINTS {
            return Err(CollisionReplayError::TooManyPoints {
                manifold: manifold_index,
                points: manifold_state.points.len(),
            });
        }

        // A captured pair can be in the opposite order to a fresh dispatch.
        // Normalize once, then build the manifold and its points in dispatch
        // order.
        let info0 = body_info(arena, body_id0)?;
        let info1 = body_info(arena, body_id1)?;
        let swapped = needs_dispatch_swap(&info0, &info1);
        let (body0, body1, info0, info1) = if swapped {
            (body_id1, body_id0, info1, info0)
        } else {
            (body_id0, body_id1, info0, info1)
        };
        if swapped {
            report.manifolds_reoriented += 1;
        }

        let (breaking_threshold, processing_threshold) = fresh_thresholds(&info0, &info1);
        if (manifold_state.contact_breaking_threshold - breaking_threshold).abs()
            > THRESHOLD_TOLERANCE_BT
        {
            report.threshold_mismatches += 1;
        }

        let margin = breaking_threshold.max(manifold_state.contact_breaking_threshold)
            + CONTAINMENT_EPSILON_BT;
        let mut points = Vec::with_capacity(manifold_state.points.len());
        for (point_index, point_state) in manifold_state.points.iter().enumerate() {
            if !point_state.is_finite() {
                return Err(CollisionReplayError::NonFinitePoint {
                    manifold: manifold_index,
                    point: point_index,
                });
            }

            if !point_state.normal_world_on_b.is_normalized() {
                return Err(CollisionReplayError::NormalNotUnit {
                    manifold: manifold_index,
                    point: point_index,
                });
            }

            if point_state.distance
                > manifold_state.contact_breaking_threshold + THRESHOLD_TOLERANCE_BT
            {
                report.points_above_breaking_threshold += 1;
            }

            // Dispatch order puts body A and body B in a fixed order, so a
            // swapped manifold swaps the anchors and the witnesses and negates
            // the normal. The signed distance is unchanged: with
            // `point_a = point_b + normal * distance`, both sides flip.
            let (local_a, local_b, world_a, world_b, normal) = if swapped {
                (
                    point_state.local_point_b,
                    point_state.local_point_a,
                    point_state.position_world_on_b,
                    point_state.position_world_on_a,
                    -point_state.normal_world_on_b,
                )
            } else {
                (
                    point_state.local_point_a,
                    point_state.local_point_b,
                    point_state.position_world_on_a,
                    point_state.position_world_on_b,
                    point_state.normal_world_on_b,
                )
            };

            // A contact anchor is a point on or near its body surface. The
            // check uses the local anchor, so it does not depend on when the
            // captured witness was computed. It is a diagnostic: a native car
            // shape can have a feature that the RocketSim hitbox does not
            // have.
            let overshoot = local_point_overshoot(&info0, local_a, margin)
                .max(local_point_overshoot(&info1, local_b, margin));
            if overshoot > 0.0 {
                report.points_outside_body += 1;
                report.max_anchor_overshoot_bt = report.max_anchor_overshoot_bt.max(overshoot);
            }

            let witness_residual = info0
                .world_point(local_a)
                .distance(world_a)
                .max(info1.world_point(local_b).distance(world_b));
            report.max_witness_residual_bt = report.max_witness_residual_bt.max(witness_residual);

            points.push(ContactPoint {
                local_point_a: local_a,
                local_point_b: local_b,
                pos_world_on_a: world_a,
                pos_world_on_b: world_b,
                normal_world_on_b: normal,
                distance: point_state.distance,
                combined_friction: friction,
                combined_restitution: restitution,
                applied_impulse: 0.0,
                is_special: false,
            });
            report.points_installed += 1;
        }

        built.push(ContactManifold {
            body0,
            body1,
            contact_breaking_threshold: breaking_threshold,
            contact_processing_threshold: processing_threshold,
            points,
        });
    }

    report.manifolds_installed = built.len();
    Ok((built, report))
}

/// Restore the covered native manifolds into one arena.
///
/// The call writes each captured covered manifold and clears the retained
/// contacts of every other owned manifold. It checks every value before the
/// first write. It writes nothing when it returns an error. An empty valid
/// state clears owned contacts.
///
/// The call never removes a manifold and never moves one. A captured pair that
/// the arena already holds is replaced in place. A captured pair that the
/// arena does not hold is appended. An owned pair that the replay does not
/// list keeps its slot with an empty point cache. This keeps the index of
/// every non-restored manifold. The solver sorts active manifolds by
/// persistent index before an unstable island-key sort, so a moved index can
/// change the order of equal-island-key contacts.
///
/// The caller must apply this call after the car and ball state setters, so
/// that the body transforms match the captured state. Call it between
/// dispatches, when the solver has cleared the active manifold list.
///
/// A captured manifold is normalized to the order that a fresh dispatch
/// builds. See [`CollisionReplayReport::manifolds_reoriented`].
///
/// `car_slots` maps recorded car slots to current arena car indices. Use
/// `None` for an unavailable slot.
///
/// # Errors
///
/// Returns [`CollisionReplayError`] for an invalid slot map, body pair,
/// threshold, or contact value. The arena contact state is unchanged.
pub fn apply(
    arena: &mut Arena,
    car_slots: &[Option<usize>],
    state: &CollisionReplayState,
) -> Result<CollisionReplayReport, CollisionReplayError> {
    // Every check runs before the first write, so a failure leaves the arena
    // unchanged. This includes an empty captured state, which still clears the
    // owned manifolds.
    let universe = resolve_slots(arena, car_slots)?;
    let (built, mut report) = build_manifolds(arena, &universe, state)?;

    let existing: Vec<ContactManifold> = arena.contact_manifolds().collect();
    let mut existing_by_pair: HashMap<(BodyId, BodyId), &ContactManifold> =
        HashMap::with_capacity(existing.len());
    for manifold in &existing {
        existing_by_pair.insert(ordered_pair(manifold.body0, manifold.body1), manifold);
    }

    let mut captured_pairs = HashSet::with_capacity(built.len());
    let mut submitted = Vec::with_capacity(built.len() + existing.len());

    for mut manifold in built {
        let pair = ordered_pair(manifold.body0, manifold.body1);
        captured_pairs.insert(pair);

        if let Some(existing_manifold) = existing_by_pair.get(&pair) {
            // A fresh dispatch order is derived from body kind and collision
            // order. Keep the stored thresholds of the existing manifold. The
            // arena keeps its stored body order.
            manifold.contact_breaking_threshold = existing_manifold.contact_breaking_threshold;
            manifold.contact_processing_threshold = existing_manifold.contact_processing_threshold;
            report.manifolds_replaced += 1;
        } else {
            report.manifolds_appended += 1;
        }

        submitted.push(manifold);
    }

    // A pair the capture does not list had no retained manifold in native.
    // Clear its points but keep the slot and its index.
    let mut emptied = 0;
    for existing_manifold in &existing {
        if !universe.contains_pair(existing_manifold.body0, existing_manifold.body1) {
            continue;
        }
        if captured_pairs.contains(&ordered_pair(
            existing_manifold.body0,
            existing_manifold.body1,
        )) {
            continue;
        }
        if existing_manifold.points.is_empty() {
            continue;
        }
        emptied += 1;
        submitted.push(ContactManifold {
            body0: existing_manifold.body0,
            body1: existing_manifold.body1,
            contact_breaking_threshold: existing_manifold.contact_breaking_threshold,
            contact_processing_threshold: existing_manifold.contact_processing_threshold,
            points: Vec::new(),
        });
    }

    report.manifolds_emptied = emptied;
    report.manifolds_cleared = report.manifolds_replaced + emptied;

    arena
        .set_contact_manifolds(&submitted)
        .map_err(CollisionReplayError::Physics)?;

    Ok(report)
}

#[cfg(test)]
mod tests {
    use glam::Mat3A;
    use rocketsim::{BallState, CarBodyConfig, CarControls, CarCore, GameMode, PhysState, Team};

    use super::*;

    /// Ball body and car bodies for one test arena.
    fn arena_with_cars(car_count: usize) -> (Arena, Vec<BodyId>) {
        let mut arena = Arena::new(GameMode::TheVoid);
        let mut cars = Vec::with_capacity(car_count);
        for idx in 0..car_count {
            let team = if idx % 2 == 0 {
                Team::Blue
            } else {
                Team::Orange
            };
            let car = arena.add_car(team, CarBodyConfig::OCTANE);
            cars.push(arena.car_body_id(car).expect("car body"));
        }
        (arena, cars)
    }

    /// A recorded point with a witness rebuilt from its local anchor.
    fn replay_point(
        arena: &Arena,
        body0: BodyId,
        body1: BodyId,
        local_a: Vec3A,
    ) -> CollisionReplayPoint {
        let info0 = arena.body_info(body0).expect("body 0");
        let info1 = arena.body_info(body1).expect("body 1");
        let local_b = Vec3A::ZERO;
        CollisionReplayPoint {
            local_point_a: local_a,
            local_point_b: local_b,
            position_world_on_a: info0.world_point(local_a),
            position_world_on_b: info1.world_point(local_b),
            normal_world_on_b: Vec3A::Z,
            distance: -0.01,
        }
    }

    /// One stored contact point with a witness rebuilt from its local anchor.
    fn stored_point(arena: &Arena, body0: BodyId, body1: BodyId) -> ContactPoint {
        let info0 = arena.body_info(body0).expect("body 0");
        let info1 = arena.body_info(body1).expect("body 1");
        ContactPoint {
            local_point_a: Vec3A::ZERO,
            local_point_b: Vec3A::ZERO,
            pos_world_on_a: info0.world_point(Vec3A::ZERO),
            pos_world_on_b: info1.world_point(Vec3A::ZERO),
            normal_world_on_b: Vec3A::Z,
            distance: -0.01,
            combined_friction: 0.0,
            combined_restitution: 0.0,
            applied_impulse: 0.0,
            is_special: false,
        }
    }

    fn manifold(
        body0: ReplayBodyRef,
        body1: ReplayBodyRef,
        points: Vec<CollisionReplayPoint>,
    ) -> CollisionReplayManifold {
        CollisionReplayManifold {
            body0,
            body1,
            contact_breaking_threshold: 0.02,
            contact_processing_threshold: 1.0e30,
            points,
        }
    }

    fn stored_manifold(arena: &Arena, body0: BodyId, body1: BodyId) -> ContactManifold {
        ContactManifold {
            body0,
            body1,
            contact_breaking_threshold: 0.02,
            contact_processing_threshold: 1.0e30,
            points: vec![stored_point(arena, body0, body1)],
        }
    }

    /// Swap the two body frames of one recorded point.
    fn reversed_point(point: CollisionReplayPoint) -> CollisionReplayPoint {
        CollisionReplayPoint {
            local_point_a: point.local_point_b,
            local_point_b: point.local_point_a,
            position_world_on_a: point.position_world_on_b,
            position_world_on_b: point.position_world_on_a,
            normal_world_on_b: -point.normal_world_on_b,
            distance: point.distance,
        }
    }

    #[test]
    fn ball_car_state_uses_dispatch_order_and_ball_coefs() {
        for ball_first in [true, false] {
            let (mut arena, cars) = arena_with_cars(1);
            let ball = arena.ball_body_id();
            let car = cars[0];
            let (body0, body1, body0_id, body1_id) = if ball_first {
                (
                    ReplayBodyRef::Ball,
                    ReplayBodyRef::Car { slot: 0 },
                    ball,
                    car,
                )
            } else {
                (
                    ReplayBodyRef::Car { slot: 0 },
                    ReplayBodyRef::Ball,
                    car,
                    ball,
                )
            };

            let mut first = replay_point(&arena, body0_id, body1_id, Vec3A::new(0.0, 0.0, 0.01));
            // A stale witness is normal state: a witness is only recomputed
            // during a dispatch. The restore must accept it.
            first.position_world_on_a += Vec3A::new(0.2, 0.0, 0.0);
            let second = replay_point(&arena, body0_id, body1_id, Vec3A::new(0.0, 0.0, 0.02));
            let state = CollisionReplayState {
                manifolds: vec![manifold(body0, body1, vec![first, second])],
            };

            let report = apply(&mut arena, &[Some(0)], &state).expect("apply");
            // A fresh dispatch always puts the car first.
            let swapped = ball_first;
            assert_eq!(report.manifolds_installed, 1);
            assert_eq!(report.points_installed, 2);
            assert_eq!(report.manifolds_cleared, 0);
            assert_eq!(report.manifolds_reoriented, if swapped { 1 } else { 0 });

            let ball_info = arena.body_info(ball).expect("ball info");
            let car_info = arena.body_info(car).expect("car info");
            let fresh = fresh_thresholds(&ball_info, &car_info).0;
            let expected_mismatch = if (0.02 - fresh).abs() > THRESHOLD_TOLERANCE_BT {
                1
            } else {
                0
            };
            assert_eq!(report.threshold_mismatches, expected_mismatch);
            assert_eq!(report.points_above_breaking_threshold, 0);
            assert_eq!(report.points_outside_body, 0);
            assert!((report.max_witness_residual_bt - 0.2).abs() < 1.0e-5);

            let stored: Vec<ContactManifold> = arena.contact_manifolds().collect();
            assert_eq!(stored.len(), 1);
            let stored = &stored[0];
            assert_eq!(stored.body0, car);
            assert_eq!(stored.body1, ball);
            assert_eq!(stored.points.len(), 2);

            for (index, (restored, captured)) in
                stored.points.iter().zip([first, second]).enumerate()
            {
                let (local_a, local_b, world_a, world_b, normal) = if swapped {
                    (
                        captured.local_point_b,
                        captured.local_point_a,
                        captured.position_world_on_b,
                        captured.position_world_on_a,
                        -captured.normal_world_on_b,
                    )
                } else {
                    (
                        captured.local_point_a,
                        captured.local_point_b,
                        captured.position_world_on_a,
                        captured.position_world_on_b,
                        captured.normal_world_on_b,
                    )
                };
                assert_eq!(restored.local_point_a, local_a, "point {index}");
                assert_eq!(restored.local_point_b, local_b, "point {index}");
                assert_eq!(restored.pos_world_on_a, world_a, "point {index}");
                assert_eq!(restored.pos_world_on_b, world_b, "point {index}");
                assert_eq!(restored.normal_world_on_b, normal, "point {index}");
                assert_eq!(restored.distance, captured.distance, "point {index}");
                assert_eq!(
                    restored.combined_friction,
                    consts::car::HIT_BALL_COEFS.friction
                );
                assert_eq!(
                    restored.combined_restitution,
                    consts::car::HIT_BALL_COEFS.restitution
                );
                assert_eq!(restored.applied_impulse, 0.0);
                assert!(!restored.is_special);
            }
        }
    }

    #[test]
    fn car_car_state_uses_dispatch_order_and_car_coefs() {
        let (mut arena, cars) = arena_with_cars(2);
        let (car0, car1) = (cars[0], cars[1]);
        let info0 = arena.body_info(car0).expect("car 0");
        let info1 = arena.body_info(car1).expect("car 1");
        assert_ne!(info0.collision_order, info1.collision_order);

        // A fresh broadphase pair puts the smaller collision order first.
        let (high_slot, low_slot) = if info0.collision_order > info1.collision_order {
            (0u8, 1u8)
        } else {
            (1, 0)
        };
        let (high_body, low_body) = (cars[usize::from(high_slot)], cars[usize::from(low_slot)]);

        // The captured order is the reverse of the fresh order.
        let captured = replay_point(&arena, high_body, low_body, Vec3A::new(0.0, 0.0, 0.01));
        let state = CollisionReplayState {
            manifolds: vec![manifold(
                ReplayBodyRef::Car { slot: high_slot },
                ReplayBodyRef::Car { slot: low_slot },
                vec![captured],
            )],
        };

        let report = apply(&mut arena, &[Some(0), Some(1)], &state).expect("apply");
        assert_eq!(report.manifolds_reoriented, 1);

        let stored: Vec<ContactManifold> = arena.contact_manifolds().collect();
        assert_eq!(stored.len(), 1);
        let stored = &stored[0];
        assert_eq!(stored.body0, low_body);
        assert_eq!(stored.body1, high_body);
        let point = &stored.points[0];
        assert_eq!(point.local_point_a, captured.local_point_b);
        assert_eq!(point.local_point_b, captured.local_point_a);
        assert_eq!(point.pos_world_on_a, captured.position_world_on_b);
        assert_eq!(point.pos_world_on_b, captured.position_world_on_a);
        assert_eq!(point.normal_world_on_b, -captured.normal_world_on_b);
        assert_eq!(point.distance, captured.distance);
        assert_eq!(point.combined_friction, consts::car::HIT_CAR_COEFS.friction);
        assert_eq!(
            point.combined_restitution,
            consts::car::HIT_CAR_COEFS.restitution
        );
        assert_eq!(point.applied_impulse, 0.0);
        assert!(!point.is_special);
    }

    #[test]
    fn second_restore_replaces_in_place_without_duplicates() {
        let (mut arena, cars) = arena_with_cars(1);
        let ball = arena.ball_body_id();
        let car = cars[0];
        let state = CollisionReplayState {
            manifolds: vec![manifold(
                ReplayBodyRef::Ball,
                ReplayBodyRef::Car { slot: 0 },
                vec![replay_point(&arena, ball, car, Vec3A::ZERO)],
            )],
        };

        let first = apply(&mut arena, &[Some(0)], &state).expect("first");
        assert_eq!(first.manifolds_appended, 1);
        assert_eq!(first.manifolds_replaced, 0);

        let second = apply(&mut arena, &[Some(0)], &state).expect("second");
        assert_eq!(second.manifolds_replaced, 1);
        assert_eq!(second.manifolds_appended, 0);
        assert_eq!(second.manifolds_emptied, 0);
        assert_eq!(second.manifolds_cleared, 1);
        assert_eq!(second.manifolds_installed, 1);
        assert_eq!(second.points_installed, 1);

        let stored: Vec<ContactManifold> = arena.contact_manifolds().collect();
        assert_eq!(stored.len(), 1);
        assert_eq!(stored[0].body0, car);
        assert_eq!(stored[0].body1, ball);
        assert_eq!(stored[0].points.len(), 1);
    }

    #[test]
    fn empty_state_clears_owned_pairs_and_keeps_unowned_pairs() {
        let (mut arena, cars) = arena_with_cars(3);
        let ball = arena.ball_body_id();
        let (car0, car1, car2) = (cars[0], cars[1], cars[2]);

        // Slot 0 and slot 1 are owned. Car 2 has no recorded slot, so its
        // contact with car 1 is not owned.
        let seeded = vec![
            stored_manifold(&arena, ball, car0),
            stored_manifold(&arena, car1, car2),
            stored_manifold(&arena, car0, car1),
        ];
        arena.set_contact_manifolds(&seeded).expect("seed");
        assert_eq!(
            arena.contact_manifolds().collect::<Vec<_>>(),
            seeded,
            "the fixture must keep the supplied order"
        );

        let report = apply(
            &mut arena,
            &[Some(0), Some(1)],
            &CollisionReplayState::default(),
        )
        .expect("apply");
        assert_eq!(report.manifolds_emptied, 2);
        assert_eq!(report.manifolds_replaced, 0);
        assert_eq!(report.manifolds_cleared, 2);
        assert_eq!(report.manifolds_installed, 0);
        assert_eq!(report.points_installed, 0);

        // No manifold is removed. The owned pairs keep their slots and their
        // contacts are cleared. The unowned pair keeps its contact.
        let after: Vec<ContactManifold> = arena.contact_manifolds().collect();
        assert_eq!(after.len(), 3);
        assert_eq!(after[0].body0, ball);
        assert_eq!(after[0].body1, car0);
        assert!(after[0].points.is_empty());
        assert_eq!(after[1], seeded[1]);
        assert_eq!(after[2].body0, car0);
        assert_eq!(after[2].body1, car1);
        assert!(after[2].points.is_empty());
    }

    #[test]
    fn separated_and_outside_points_are_diagnostics_not_gates() {
        let (mut arena, cars) = arena_with_cars(1);
        let ball = arena.ball_body_id();
        let car = cars[0];

        // A positive distance above the captured threshold is a diagnostic,
        // not a refusal.
        let mut separated = replay_point(&arena, car, ball, Vec3A::ZERO);
        separated.distance = 0.5;
        // An anchor far outside its body is a diagnostic, not a refusal.
        let outside_local = Vec3A::new(3.0, 0.0, 0.0);
        let mut outside = replay_point(&arena, car, ball, outside_local);
        outside.position_world_on_a = arena
            .body_info(car)
            .expect("car info")
            .world_point(outside_local);

        let state = CollisionReplayState {
            manifolds: vec![manifold(
                ReplayBodyRef::Car { slot: 0 },
                ReplayBodyRef::Ball,
                vec![separated, outside],
            )],
        };
        let report = apply(&mut arena, &[Some(0)], &state).expect("apply");
        assert_eq!(report.manifolds_installed, 1);
        assert_eq!(report.points_installed, 2);
        assert_eq!(report.points_above_breaking_threshold, 1);
        assert_eq!(report.points_outside_body, 1);
        assert!(report.max_anchor_overshoot_bt > 0.0);

        // Both points stay in the stored manifold.
        let stored: Vec<ContactManifold> = arena.contact_manifolds().collect();
        assert_eq!(stored[0].points.len(), 2);
        assert_eq!(stored[0].points[0].distance, 0.5);
        assert_eq!(stored[0].points[1].local_point_a, outside_local);
    }

    #[test]
    fn invalid_captures_are_refused_atomically() {
        let (mut arena, cars) = arena_with_cars(1);
        let ball = arena.ball_body_id();
        let car = cars[0];
        let seeded = stored_manifold(&arena, ball, car);
        arena
            .set_contact_manifolds(std::slice::from_ref(&seeded))
            .expect("seed");
        let before: Vec<ContactManifold> = arena.contact_manifolds().collect();

        let ball_ref = ReplayBodyRef::Ball;
        let car_ref = ReplayBodyRef::Car { slot: 0 };
        let good = replay_point(&arena, ball, car, Vec3A::ZERO);

        let mut nan_breaking = manifold(ball_ref, car_ref, Vec::new());
        nan_breaking.contact_breaking_threshold = f32::NAN;
        let mut negative_breaking = manifold(ball_ref, car_ref, Vec::new());
        negative_breaking.contact_breaking_threshold = -0.02;
        let mut infinite_processing = manifold(ball_ref, car_ref, Vec::new());
        infinite_processing.contact_processing_threshold = f32::INFINITY;

        let mut non_finite = good;
        non_finite.local_point_a = Vec3A::NAN;
        let mut bad_normal = good;
        bad_normal.normal_world_on_b = Vec3A::new(0.0, 0.0, 2.0);

        let duplicate_pair = CollisionReplayState {
            manifolds: vec![
                manifold(ball_ref, car_ref, vec![good]),
                manifold(car_ref, ball_ref, Vec::new()),
            ],
        };

        let cases: Vec<(
            &str,
            Vec<Option<usize>>,
            CollisionReplayState,
            CollisionReplayError,
        )> = vec![
            (
                "duplicate slot body",
                vec![Some(0), Some(0)],
                CollisionReplayState::default(),
                CollisionReplayError::DuplicateSlotBody {
                    body_index: car.index(),
                },
            ),
            (
                "car slot out of range",
                vec![Some(99)],
                CollisionReplayState::default(),
                CollisionReplayError::CarSlotOutOfRange {
                    slot: 0,
                    body_index: 99,
                },
            ),
            (
                "duplicate body pair",
                vec![Some(0)],
                duplicate_pair,
                CollisionReplayError::DuplicateBodyPair { manifold: 1 },
            ),
            (
                "too many points",
                vec![Some(0)],
                CollisionReplayState {
                    manifolds: vec![manifold(
                        ball_ref,
                        car_ref,
                        vec![good; MAX_CONTACT_POINTS + 1],
                    )],
                },
                CollisionReplayError::TooManyPoints {
                    manifold: 0,
                    points: MAX_CONTACT_POINTS + 1,
                },
            ),
            (
                "unsupported pair",
                vec![Some(0)],
                CollisionReplayState {
                    manifolds: vec![manifold(ball_ref, ball_ref, vec![good])],
                },
                CollisionReplayError::UnsupportedBodyPair { manifold: 0 },
            ),
            (
                "same car pair",
                vec![Some(0)],
                CollisionReplayState {
                    manifolds: vec![manifold(car_ref, car_ref, vec![good])],
                },
                CollisionReplayError::SameBodyPair { manifold: 0 },
            ),
            (
                "unmapped car slot",
                vec![Some(0)],
                CollisionReplayState {
                    manifolds: vec![manifold(
                        ball_ref,
                        ReplayBodyRef::Car { slot: 2 },
                        vec![good],
                    )],
                },
                CollisionReplayError::UnmappedBody {
                    manifold: 0,
                    body: ReplayBodyRef::Car { slot: 2 },
                },
            ),
            (
                "non-finite point",
                vec![Some(0)],
                CollisionReplayState {
                    manifolds: vec![manifold(ball_ref, car_ref, vec![non_finite])],
                },
                CollisionReplayError::NonFinitePoint {
                    manifold: 0,
                    point: 0,
                },
            ),
            (
                "bad normal",
                vec![Some(0)],
                CollisionReplayState {
                    manifolds: vec![manifold(ball_ref, car_ref, vec![bad_normal])],
                },
                CollisionReplayError::NormalNotUnit {
                    manifold: 0,
                    point: 0,
                },
            ),
            (
                "nan breaking threshold",
                vec![Some(0)],
                CollisionReplayState {
                    manifolds: vec![nan_breaking],
                },
                CollisionReplayError::InvalidBreakingThreshold { manifold: 0 },
            ),
            (
                "negative breaking threshold",
                vec![Some(0)],
                CollisionReplayState {
                    manifolds: vec![negative_breaking],
                },
                CollisionReplayError::InvalidBreakingThreshold { manifold: 0 },
            ),
            (
                "infinite processing threshold",
                vec![Some(0)],
                CollisionReplayState {
                    manifolds: vec![infinite_processing],
                },
                CollisionReplayError::InvalidProcessingThreshold { manifold: 0 },
            ),
        ];

        for (name, car_slots, state, expected) in cases {
            assert_eq!(
                apply(&mut arena, &car_slots, &state),
                Err(expected),
                "{name}"
            );
            assert_eq!(
                arena.contact_manifolds().collect::<Vec<_>>(),
                before,
                "{name}"
            );
        }
    }

    #[test]
    fn threshold_and_orientation_policy_is_table_driven() {
        for capture_reversed in [false, true] {
            for seed_existing in [false, true] {
                let (mut arena, cars) = arena_with_cars(3);
                let ball = arena.ball_body_id();
                let (car0, car1, car2) = (cars[0], cars[1], cars[2]);
                let ball_ref = ReplayBodyRef::Ball;
                let car_ref = ReplayBodyRef::Car { slot: 0 };

                let ball_info = arena.body_info(ball).expect("ball info");
                let car_info = arena.body_info(car0).expect("car info");
                let (fresh_breaking, fresh_processing) = fresh_thresholds(&ball_info, &car_info);
                assert!(
                    needs_dispatch_swap(&ball_info, &car_info),
                    "the fixture expects a car-first fresh order"
                );

                // An unowned pair keeps index 0. It must stay unchanged.
                let unowned = stored_manifold(&arena, car1, car2);
                arena
                    .set_contact_manifolds(std::slice::from_ref(&unowned))
                    .expect("seed unowned");

                // A nondefault stored threshold and the reversed body order
                // make the existing cache disagree with both the capture and
                // the fresh dispatch order.
                let (stored_breaking, stored_processing) = if seed_existing {
                    let stored_breaking = fresh_breaking + 0.25;
                    let stored_processing = 5.0e29;
                    let mut existing = stored_manifold(&arena, ball, car0);
                    existing.contact_breaking_threshold = stored_breaking;
                    existing.contact_processing_threshold = stored_processing;
                    existing.points[0].distance = -0.5;
                    arena
                        .set_contact_manifolds(std::slice::from_ref(&existing))
                        .expect("seed owned");
                    (stored_breaking, stored_processing)
                } else {
                    (fresh_breaking, fresh_processing)
                };

                // The captured thresholds disagree with the fresh minima.
                let captured_breaking = fresh_breaking + 0.1;
                let captured_processing = 1.0e30;
                let (capture_body0, capture_body1, capture_body0_id, capture_body1_id) =
                    if capture_reversed {
                        (ball_ref, car_ref, ball, car0)
                    } else {
                        (car_ref, ball_ref, car0, ball)
                    };
                let mut captured = replay_point(
                    &arena,
                    capture_body0_id,
                    capture_body1_id,
                    Vec3A::new(0.0, 0.0, 0.02),
                );
                captured.distance = -0.01;
                let state = CollisionReplayState {
                    manifolds: vec![CollisionReplayManifold {
                        body0: capture_body0,
                        body1: capture_body1,
                        contact_breaking_threshold: captured_breaking,
                        contact_processing_threshold: captured_processing,
                        points: vec![captured],
                    }],
                };

                let report = apply(&mut arena, &[Some(0), Some(1)], &state).expect("apply");

                // The diagnostic compares the captured threshold with the
                // fresh minimum, not with the stored threshold.
                assert_eq!(report.threshold_mismatches, 1);
                assert_eq!(report.manifolds_installed, 1);
                assert_eq!(report.points_installed, 1);
                assert_eq!(report.manifolds_emptied, 0);
                assert_eq!(
                    report.manifolds_reoriented,
                    if capture_reversed { 1 } else { 0 }
                );
                if seed_existing {
                    assert_eq!(report.manifolds_replaced, 1);
                    assert_eq!(report.manifolds_appended, 0);
                    assert_eq!(report.manifolds_cleared, 1);
                } else {
                    assert_eq!(report.manifolds_replaced, 0);
                    assert_eq!(report.manifolds_appended, 1);
                    assert_eq!(report.manifolds_cleared, 0);
                }

                // The unowned pair keeps index 0. The owned pair stays at
                // index 1 and occurs once.
                let stored: Vec<ContactManifold> = arena.contact_manifolds().collect();
                assert_eq!(stored.len(), 2, "no duplicate pair");
                assert_eq!(stored[0], unowned, "unowned index 0 unchanged");
                let owned = &stored[1];
                assert_eq!(
                    ordered_pair(owned.body0, owned.body1),
                    ordered_pair(car0, ball)
                );

                // A new pair uses the fresh minima. An existing pair keeps
                // its stored thresholds and its stored body order.
                if seed_existing {
                    assert_eq!(owned.contact_breaking_threshold, stored_breaking);
                    assert_eq!(owned.contact_processing_threshold, stored_processing);
                    assert_eq!(owned.body0, ball);
                    assert_eq!(owned.body1, car0);
                } else {
                    assert_eq!(owned.contact_breaking_threshold, fresh_breaking);
                    assert_eq!(owned.contact_processing_threshold, fresh_processing);
                    assert_eq!(owned.body0, car0);
                    assert_eq!(owned.body1, ball);
                }

                // The point values come from the capture, expressed in the
                // stored body order. The captured distance differs from the
                // seeded existing distance, so this checks the point fields
                // too.
                let expected = if seed_existing == capture_reversed {
                    captured
                } else {
                    reversed_point(captured)
                };
                let point = &owned.points[0];
                assert_eq!(point.local_point_a, expected.local_point_a);
                assert_eq!(point.local_point_b, expected.local_point_b);
                assert_eq!(point.pos_world_on_a, expected.position_world_on_a);
                assert_eq!(point.pos_world_on_b, expected.position_world_on_b);
                assert_eq!(point.normal_world_on_b, expected.normal_world_on_b);
                assert_eq!(point.distance, captured.distance);
                assert_eq!(
                    point.combined_friction,
                    consts::car::HIT_BALL_COEFS.friction
                );
                assert_eq!(
                    point.combined_restitution,
                    consts::car::HIT_BALL_COEFS.restitution
                );
                assert_eq!(point.applied_impulse, 0.0);
                assert!(!point.is_special);
            }
        }
    }

    #[test]
    fn reversed_contact_state_matches_the_reference_after_step() {
        // Make a genuine car and ball contact. The ball overlaps the car
        // hitbox from above. The first step resolves part of the penetration
        // and leaves a retained manifold.
        let mut source = Arena::new(GameMode::TheVoid);
        let car_idx = source.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let car_pos = Vec3A::new(0.0, 0.0, 1000.0);
        source.set_car_state(
            car_idx,
            CarCore::aerial(PhysState {
                pos: car_pos,
                rot_mat: Mat3A::IDENTITY,
                vel: Vec3A::ZERO,
                ang_vel: Vec3A::ZERO,
            }),
        );
        let mut ball_state = BallState::DEFAULT;
        ball_state.phys.pos = car_pos + Vec3A::new(0.0, 0.0, 120.0);
        ball_state.phys.rot_mat = Mat3A::IDENTITY;
        ball_state.phys.vel = Vec3A::ZERO;
        ball_state.phys.ang_vel = Vec3A::ZERO;
        ball_state.wake();
        source.set_ball_state(ball_state);
        source.step_tick();

        let car = source.car_body_id(car_idx).expect("car body");
        let ball = source.ball_body_id();
        let actual: Vec<ContactManifold> = source.contact_manifolds().collect();
        assert_eq!(actual.len(), 1, "the fixture must keep one manifold");
        let actual = &actual[0];
        // A fresh dispatch puts the compound car body first.
        assert_eq!(actual.body0, car);
        assert_eq!(actual.body1, ball);
        assert!(!actual.points.is_empty());
        assert!(actual.points.iter().any(|point| point.distance < 0.0));

        // Both clones start from the same genuine contact cache.
        let mut reference = source.clone();
        let mut import = source.clone();

        // Reset the fields that the importer also resets. Use the same public
        // setter, so every stored field outside the public point type resets
        // in the same way.
        let mut normalized: Vec<ContactManifold> = reference.contact_manifolds().collect();
        for manifold in &mut normalized {
            for point in &mut manifold.points {
                point.applied_impulse = 0.0;
                point.is_special = false;
            }
        }
        reference
            .set_contact_manifolds(&normalized)
            .expect("normalize reference");

        // Record the actual geometry with the body order reversed.
        let reversed_points: Vec<CollisionReplayPoint> = actual
            .points
            .iter()
            .map(|point| CollisionReplayPoint {
                local_point_a: point.local_point_b,
                local_point_b: point.local_point_a,
                position_world_on_a: point.pos_world_on_b,
                position_world_on_b: point.pos_world_on_a,
                normal_world_on_b: -point.normal_world_on_b,
                distance: point.distance,
            })
            .collect();
        let state = CollisionReplayState {
            manifolds: vec![CollisionReplayManifold {
                body0: ReplayBodyRef::Ball,
                body1: ReplayBodyRef::Car { slot: 0 },
                contact_breaking_threshold: actual.contact_breaking_threshold,
                contact_processing_threshold: actual.contact_processing_threshold,
                points: reversed_points,
            }],
        };

        let report = apply(&mut import, &[Some(car_idx)], &state).expect("apply");
        assert_eq!(report.manifolds_installed, 1);
        assert_eq!(report.points_installed, actual.points.len());
        assert_eq!(report.manifolds_replaced, 1);
        assert_eq!(report.manifolds_appended, 0);
        assert_eq!(report.manifolds_reoriented, 1);
        assert_eq!(report.manifolds_emptied, 0);
        assert_eq!(report.manifolds_cleared, 1);

        // The imported cache matches the normalized reference cache. This
        // checks every point value, the point order, the body order, the
        // thresholds, and the persistent manifold order.
        assert_eq!(
            import.contact_manifolds().collect::<Vec<_>>(),
            reference.contact_manifolds().collect::<Vec<_>>()
        );

        // The next dispatch merges the retained points with fresh points and
        // builds the solver table. Both arenas must stay equal.
        let controls = CarControls::DEFAULT.with_throttle(1.0);
        reference.set_car_controls(car_idx, controls);
        import.set_car_controls(car_idx, controls);
        for tick in 0..2 {
            reference.step_tick();
            import.step_tick();
            assert_eq!(
                import.get_car_state(car_idx).phys,
                reference.get_car_state(car_idx).phys,
                "car state at tick {tick}"
            );
            assert_eq!(
                import.get_ball_state().phys,
                reference.get_ball_state().phys,
                "ball state at tick {tick}"
            );
            assert_eq!(
                import.contact_manifolds().collect::<Vec<_>>(),
                reference.contact_manifolds().collect::<Vec<_>>(),
                "contact cache at tick {tick}"
            );
        }

        // The restored pair must not be duplicated.
        let pair_count = import
            .contact_manifolds()
            .filter(|manifold| {
                ordered_pair(manifold.body0, manifold.body1) == ordered_pair(car, ball)
            })
            .count();
        assert_eq!(pair_count, 1);
    }
}
