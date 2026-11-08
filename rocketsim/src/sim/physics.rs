//! General physics body and contact access.
//!
//! Use this module to inspect the physics bodies of an [`Arena`]. Use it also
//! to read or replace the retained contact manifolds of the arena. Contact
//! inspection, contact planning, and diagnostics are the intended uses.
//! Normal control of a simulation uses [`crate::CarState`], [`crate::BallState`],
//! and [`Arena::step_tick`].
//!
//! # Units
//!
//! All values in this module use native Bullet units. One native unit is 50
//! Unreal units. See [`crate::consts::BT_TO_UU`]. Positions, bounds, contact
//! distances, and thresholds use native units. Angles use radians. Local
//! points use the body frame. World points and normals use the world frame.
//!
//! # Scope
//!
//! These types are not a complete arena snapshot. They change one part of the
//! body or contact state. They do not change body pose, velocity, activation,
//! game state, or time. Reading contact manifolds copies their point buffers.
//! Use [`Arena::clone`] when you need a complete simulation branch.

use std::fmt::{self, Display, Formatter};

use arrayvec::ArrayVec;
use glam::{Affine3A, Mat3A, Vec3A};

use crate::{
    bullet::{
        collision::narrowphase::{
            manifold_point::ManifoldPoint,
            persistent_manifold::{MANIFOLD_CACHE_SIZE, PersistentManifold, pair_key},
        },
        dynamics::rigid_body::CollisionFlags,
    },
    shared::Aabb,
    sim::{Arena, UserInfoType},
};

/// Maximum number of retained points in one contact manifold.
pub const MAX_CONTACT_POINTS: usize = MANIFOLD_CACHE_SIZE;

/// Handle for one physics body in one [`Arena`].
///
/// A handle is valid only for the arena and body layout that made it. The
/// arena does not keep a handle stable:
///
/// - [`Arena::remove_car`] removes a body and shifts the index of each later
///   body.
/// - [`Arena::remove_cars`] removes all car bodies.
///
/// Get the handles again after a structural change. Do not use a handle with
/// another arena. An old handle can name a different body in a changed arena.
/// An out-of-range handle returns [`StateError::UnknownBody`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyId(usize);

impl BodyId {
    /// Index of this body in the arena body array.
    ///
    /// The index is not stable across body removal. See the [`BodyId`] type
    /// documentation.
    #[inline]
    #[must_use]
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Read-only description of one physics body.
///
/// Read it with [`Arena::body_info`]. All values use native Bullet units.
/// See the [module documentation](self) for the unit convention.
#[derive(Clone, Copy, Debug)]
pub struct BodyInfo {
    /// What kind of body this is.
    pub kind: UserInfoType,
    /// World transform of the body. `matrix3` is the rotation basis.
    pub world_transform: Affine3A,
    /// World-space axis-aligned bounds of the body shape at the current pose.
    pub aabb: Aabb,
    /// Contact breaking threshold of the body shape.
    pub contact_breaking_threshold: f32,
    /// Contact processing threshold of the body.
    pub contact_processing_threshold: f32,
    /// Broadphase ordering key of this body.
    ///
    /// The key stays constant while the body exists. A lower key sorts first
    /// in a fresh broadphase pair. The arena can reuse this key after body
    /// removal. It is not a stable body identity.
    pub collision_order: usize,
    /// Collision flags that select the transform fast path.
    flags: u8,
}

impl BodyInfo {
    /// Convert a point from the body frame to the world frame.
    ///
    /// The method uses the same transform fast paths as the contact solver.
    /// An identity body returns the local point unchanged. A body without
    /// angular motion adds the body translation. Other bodies apply the full
    /// world transform. The result is exact for each path.
    #[inline]
    #[must_use]
    pub fn world_point(&self, local_point: Vec3A) -> Vec3A {
        if self.flags & CollisionFlags::IdentityTransform != 0 {
            local_point
        } else if self.flags & CollisionFlags::NoAngularMotion != 0 {
            local_point + self.world_transform.translation
        } else {
            self.world_transform.transform_point3a(local_point)
        }
    }
}

/// Low-level body values that a normal state setter computes.
///
/// Read it with [`Arena::body_state`]. Write it with [`Arena::set_body_state`]
/// only when a caller must keep values that differ from the computed values.
///
/// The normal state setters compute the inverse inertia tensor and the
/// interpolation basis from the supplied pose. They keep the sleeping
/// thresholds. A body state write changes only the values in this struct. It
/// does not change the high-level state, the linear or angular velocity, the
/// activation state, the deactivation timer, or the interpolation translation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyState {
    /// World-space inverse inertia tensor.
    ///
    /// [`Mat3A`] stores columns. Use [`Mat3A::from_rows`] for row-major input.
    pub inv_inertia_tensor_world: Mat3A,
    /// Rotation basis of the interpolation transform. This value has no unit.
    ///
    /// [`Mat3A`] stores columns. The setter keeps the interpolation
    /// translation.
    pub interp_world_basis: Mat3A,
    /// Linear sleeping speed threshold, in native units per second.
    pub linear_sleeping_threshold: f32,
    /// Angular sleeping speed threshold, in radians per second.
    pub angular_sleeping_threshold: f32,
}

/// One retained contact point.
///
/// All values use native Bullet units. A read copies the stored values without
/// conversion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactPoint {
    /// Contact point in the local frame of body 0.
    pub local_point_a: Vec3A,
    /// Contact point in the local frame of body 1.
    pub local_point_b: Vec3A,
    /// Contact point in the world frame on body 0.
    pub pos_world_on_a: Vec3A,
    /// Contact point in the world frame on body 1.
    pub pos_world_on_b: Vec3A,
    /// Unit contact normal on body 1.
    pub normal_world_on_b: Vec3A,
    /// Signed contact distance. A positive value is a separating contact.
    pub distance: f32,
    /// Combined friction coefficient of the contact.
    pub combined_friction: f32,
    /// Combined restitution coefficient of the contact.
    pub combined_restitution: f32,
    /// Solver warm-start impulse of the contact.
    ///
    /// The solver scales this value at the start of the next contact row
    /// setup. A new contact starts at zero.
    pub applied_impulse: f32,
    /// True when the arena marked this contact for special solver treatment.
    ///
    /// The arena sets this flag for a ball contact with static world geometry.
    pub is_special: bool,
}

/// One retained contact manifold for an ordered body pair.
///
/// Read manifolds with [`Arena::contact_manifolds`]. Write them with
/// [`Arena::set_contact_manifolds`].
#[derive(Clone, Debug, PartialEq)]
pub struct ContactManifold {
    /// Body 0. The contact normal points from body 1 to body 0.
    pub body0: BodyId,
    /// Body 1.
    pub body1: BodyId,
    /// Contact breaking threshold.
    pub contact_breaking_threshold: f32,
    /// Contact processing threshold.
    pub contact_processing_threshold: f32,
    /// Retained contact points in stored order. See [`MAX_CONTACT_POINTS`].
    pub points: Vec<ContactPoint>,
}

/// Error returned by a body or contact operation in this module.
///
/// A setter checks every supplied value before it writes any value. When a
/// setter returns an error, the arena is unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StateError {
    /// The body index is outside the current body array.
    UnknownBody {
        /// Index of the unknown body.
        body: usize,
    },
    /// A manifold body index is outside the current body array.
    UnknownManifoldBody {
        /// Index of the manifold in the supplied slice.
        manifold: usize,
        /// Index of the unknown body.
        body: usize,
    },
    /// The two bodies of a manifold are the same body.
    SameBodyPair {
        /// Index of the manifold in the supplied slice.
        manifold: usize,
    },
    /// The supplied slice lists one unordered body pair more than once.
    DuplicateBodyPair {
        /// Index of the repeated manifold in the supplied slice.
        manifold: usize,
    },
    /// A manifold has more points than the retained limit.
    TooManyPoints {
        /// Index of the manifold in the supplied slice.
        manifold: usize,
        /// Number of supplied points.
        points: usize,
    },
    /// A contact point has a NaN or an infinite value.
    NonFinitePoint {
        /// Index of the manifold in the supplied slice.
        manifold: usize,
        /// Index of the point in the manifold.
        point: usize,
    },
    /// A contact normal is not a unit vector.
    NormalNotUnit {
        /// Index of the manifold in the supplied slice.
        manifold: usize,
        /// Index of the point in the manifold.
        point: usize,
    },
    /// A solver warm-start impulse is less than zero.
    NegativeAppliedImpulse {
        /// Index of the manifold in the supplied slice.
        manifold: usize,
        /// Index of the point in the manifold.
        point: usize,
    },
    /// A contact breaking threshold is not finite and nonnegative.
    InvalidBreakingThreshold {
        /// Index of the manifold in the supplied slice.
        manifold: usize,
    },
    /// A contact processing threshold is not finite and nonnegative.
    InvalidProcessingThreshold {
        /// Index of the manifold in the supplied slice.
        manifold: usize,
    },
    /// A body state value has a NaN or an infinite value.
    NonFiniteBodyState {
        /// Name of the field with the invalid value.
        field: &'static str,
    },
    /// A sleeping speed threshold is less than zero.
    NegativeSleepingThreshold {
        /// Name of the field with the invalid value.
        field: &'static str,
    },
}

impl Display for StateError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownBody { body } => {
                write!(f, "body id {body} is not in this arena")
            }
            Self::UnknownManifoldBody { manifold, body } => write!(
                f,
                "manifold {manifold} refers to body id {body}, which is not in this arena"
            ),
            Self::SameBodyPair { manifold } => {
                write!(f, "manifold {manifold} uses the same body twice")
            }
            Self::DuplicateBodyPair { manifold } => {
                write!(f, "manifold {manifold} repeats an earlier body pair")
            }
            Self::TooManyPoints { manifold, points } => {
                write!(
                    f,
                    "manifold {manifold} has {points} points; the limit is {MANIFOLD_CACHE_SIZE}"
                )
            }
            Self::NonFinitePoint { manifold, point } => {
                write!(f, "manifold {manifold} point {point} is not finite")
            }
            Self::NormalNotUnit { manifold, point } => {
                write!(
                    f,
                    "manifold {manifold} point {point} normal is not a unit vector"
                )
            }
            Self::NegativeAppliedImpulse { manifold, point } => write!(
                f,
                "manifold {manifold} point {point} applied impulse is negative"
            ),
            Self::InvalidBreakingThreshold { manifold } => write!(
                f,
                "manifold {manifold} contact breaking threshold is not finite and nonnegative"
            ),
            Self::InvalidProcessingThreshold { manifold } => write!(
                f,
                "manifold {manifold} contact processing threshold is not finite and nonnegative"
            ),
            Self::NonFiniteBodyState { field } => {
                write!(f, "body state field {field} is not finite")
            }
            Self::NegativeSleepingThreshold { field } => {
                write!(f, "body state field {field} is negative")
            }
        }
    }
}

impl std::error::Error for StateError {}

impl Arena {
    /// All physics bodies in arena body order.
    ///
    /// The iterator includes static arena geometry, Dropshot tiles, the ball,
    /// and each car. The order is the stored body order.
    ///
    /// Body handles are valid only for the current body layout. Call this
    /// method again after you remove or add bodies.
    pub fn physics_body_ids(&self) -> impl ExactSizeIterator<Item = BodyId> + '_ {
        (0..self.bullet_world.bodies().len()).map(BodyId)
    }

    /// Body handle of the ball.
    ///
    /// See [`BodyId`] for the lifetime of a handle.
    #[must_use]
    pub fn ball_body_id(&self) -> BodyId {
        BodyId(self.ball.rigid_body_idx)
    }

    /// Body handle of one car.
    ///
    /// Returns `None` when `car_idx` is not a current car index. Get the car
    /// index from [`Arena::add_car`] or [`Arena::get_car_info`]. See [`BodyId`]
    /// for the lifetime of a handle.
    #[must_use]
    pub fn car_body_id(&self, car_idx: usize) -> Option<BodyId> {
        self.cars.get(car_idx).map(|car| BodyId(car.rigid_body_idx))
    }

    /// Read the stored description of one body.
    ///
    /// # Errors
    ///
    /// Returns [`StateError::UnknownBody`] when the index is outside the current
    /// body array. Remove and clear operations shift body indexes. Get the handle
    /// again after such an operation.
    pub fn body_info(&self, body: BodyId) -> Result<BodyInfo, StateError> {
        let index = body.index();
        let rigid_body = self
            .bullet_world
            .bodies()
            .get(index)
            .ok_or(StateError::UnknownBody { body: index })?;

        Ok(BodyInfo {
            kind: rigid_body.user_idx,
            world_transform: *rigid_body.get_world_trans(),
            aabb: rigid_body
                .get_collision_shape()
                .get_aabb(rigid_body.get_world_trans()),
            contact_breaking_threshold: rigid_body.get_contact_breaking_threshold(),
            contact_processing_threshold: rigid_body.contact_processing_threshold,
            collision_order: rigid_body.get_broadphase_handle(),
            flags: rigid_body.collision_flags,
        })
    }

    /// Read the low-level body values.
    ///
    /// Normal applications do not need this method. The normal state setters
    /// compute these values from the body pose. Use it when a caller must keep
    /// stored values that differ from the computed values.
    ///
    /// # Errors
    ///
    /// Returns [`StateError::UnknownBody`] when the index is outside the current
    /// body array.
    pub fn body_state(&self, body: BodyId) -> Result<BodyState, StateError> {
        let index = body.index();
        let rigid_body = self
            .bullet_world
            .bodies()
            .get(index)
            .ok_or(StateError::UnknownBody { body: index })?;

        Ok(BodyState {
            inv_inertia_tensor_world: rigid_body.inv_inertia_tensor_world,
            interp_world_basis: rigid_body.interp_world_trans.matrix3,
            linear_sleeping_threshold: rigid_body.linear_sleeping_threshold,
            angular_sleeping_threshold: rigid_body.angular_sleeping_threshold,
        })
    }

    /// Write the low-level body values.
    ///
    /// The method changes only the values in [`BodyState`]. It does not change
    /// the body pose, the linear or angular velocity, the activation state,
    /// the deactivation timer, or the interpolation translation.
    ///
    /// The method checks every value before it writes any value. A later
    /// normal state setter recomputes the inverse inertia tensor and the
    /// interpolation basis from the pose. The sleeping thresholds stay.
    ///
    /// The method works for a static body and for a dynamic body.
    ///
    /// # Errors
    ///
    /// Returns [`StateError::UnknownBody`] when the index is outside the current
    /// body array. Returns [`StateError::NonFiniteBodyState`] for a value that is
    /// not finite. Returns [`StateError::NegativeSleepingThreshold`] for a
    /// negative sleeping threshold. On error, the body is unchanged.
    pub fn set_body_state(&mut self, body: BodyId, state: BodyState) -> Result<(), StateError> {
        validate_body_state(&state)?;

        let index = body.index();
        let rigid_body = self
            .bullet_world
            .bodies_mut()
            .get_mut(index)
            .ok_or(StateError::UnknownBody { body: index })?;

        rigid_body.inv_inertia_tensor_world = state.inv_inertia_tensor_world;
        rigid_body.interp_world_trans.matrix3 = state.interp_world_basis;
        rigid_body.linear_sleeping_threshold = state.linear_sleeping_threshold;
        rigid_body.angular_sleeping_threshold = state.angular_sleeping_threshold;

        Ok(())
    }

    /// Read every retained contact manifold in stored order.
    ///
    /// The iterator includes manifolds with no points and manifolds that touch
    /// a static body. The order is the retained dispatcher order. A read
    /// copies each manifold at the time of the iteration. The method does not
    /// change the simulation.
    pub fn contact_manifolds(&self) -> impl Iterator<Item = ContactManifold> + '_ {
        self.bullet_world
            .persistent_manifolds()
            .iter()
            .map(|manifold| ContactManifold {
                body0: BodyId(manifold.body0_idx),
                body1: BodyId(manifold.body1_idx),
                contact_breaking_threshold: manifold.contact_breaking_threshold,
                contact_processing_threshold: manifold.contact_processing_threshold,
                points: manifold
                    .point_cache
                    .iter()
                    .map(|point| ContactPoint {
                        local_point_a: point.local_point_a,
                        local_point_b: point.local_point_b,
                        pos_world_on_a: point.pos_world_on_a,
                        pos_world_on_b: point.pos_world_on_b,
                        normal_world_on_b: point.normal_world_on_b,
                        distance: point.distance_1,
                        combined_friction: point.combined_friction,
                        combined_restitution: point.combined_restitution,
                        applied_impulse: point.applied_impulse,
                        is_special: point.is_special,
                    })
                    .collect(),
            })
    }

    /// Replace or add retained contact manifolds.
    ///
    /// Use this method to change the stored contacts of a simulation branch.
    /// Use [`Arena::contact_manifolds`] to inspect contacts. Normal simulation
    /// control uses [`Arena::step_tick`].
    ///
    /// # Match and order
    ///
    /// The method matches a supplied manifold to a stored manifold by the
    /// unordered body pair. Body order is not part of the identity.
    ///
    /// - A matched manifold is replaced in place. Its index and its stored
    ///   body order do not change. When the supplied body order is the reverse
    ///   of the stored order, the method swaps the two local points and the two
    ///   world points and negates the normal on body 1. Distance, coefficients,
    ///   impulse, and the special flag do not change.
    /// - A body pair with no stored manifold is appended in the supplied body
    ///   order.
    /// - A body pair that the input does not list keeps its points, its index,
    ///   and its body order.
    /// - A supplied manifold with no points keeps its body pair and clears the
    ///   points of that pair.
    ///
    /// The method uses the supplied thresholds and every supplied point value.
    /// `applied_impulse` is the solver warm-start value on the next step.
    ///
    /// This method changes only the retained contact cache. It does not add a
    /// broadphase pair or activate a body. Collision dispatch can refresh or
    /// discard these points. A new pair participates in the solver only when
    /// the broadphase tracks it during the next dispatch.
    ///
    /// # Validation
    ///
    /// The method checks every supplied manifold before it writes any value.
    /// On error, the arena contact state is unchanged. It checks:
    ///
    /// - each body index is in the current body array;
    /// - the two bodies are different;
    /// - each unordered body pair occurs once;
    /// - each manifold has at most four points;
    /// - each point value is finite;
    /// - each normal is a unit vector;
    /// - each threshold is finite and nonnegative;
    /// - each applied impulse is finite and nonnegative.
    ///
    /// Validation cannot detect a stale handle that now names another body.
    /// Reacquire [`BodyId`] values after a structural change. Do not pass a
    /// handle from another arena.
    ///
    /// # Timing
    ///
    /// Call the method between steps, with exclusive access to the arena. Do
    /// not call it while the solver holds active contacts.
    ///
    /// # Errors
    ///
    /// Returns a [`StateError`] variant that names the failed manifold or
    /// point. The arena is unchanged.
    pub fn set_contact_manifolds(
        &mut self,
        manifolds: &[ContactManifold],
    ) -> Result<(), StateError> {
        validate_contact_manifolds(manifolds, self.bullet_world.bodies().len())?;

        let dispatcher = self.bullet_world.collision_dispatcher_mut();
        debug_assert!(
            dispatcher.active_manifolds.is_empty(),
            "set_contact_manifolds must run between steps"
        );

        for manifold in manifolds {
            let body0 = manifold.body0.index();
            let body1 = manifold.body1.index();
            let pair = ordered_pair(body0, body1);

            let existing_index = dispatcher
                .persistent_manifolds
                .iter()
                .position(|stored| ordered_pair(stored.body0_idx, stored.body1_idx) == pair);

            if let Some(index) = existing_index {
                let stored = &mut dispatcher.persistent_manifolds[index];
                let reversed = stored.body0_idx != body0;

                stored.contact_breaking_threshold = manifold.contact_breaking_threshold;
                stored.contact_processing_threshold = manifold.contact_processing_threshold;
                stored.point_cache.clear();
                for point in &manifold.points {
                    stored
                        .point_cache
                        .push(contact_point_to_bullet(point, reversed));
                }
            } else {
                let key = pair_key(body0, body1);
                let mut point_cache: ArrayVec<ManifoldPoint, MANIFOLD_CACHE_SIZE> = ArrayVec::new();
                for point in &manifold.points {
                    point_cache.push(contact_point_to_bullet(point, false));
                }

                let stored = PersistentManifold {
                    point_cache,
                    body0_idx: body0,
                    body1_idx: body1,
                    pair_key: key,
                    contact_breaking_threshold: manifold.contact_breaking_threshold,
                    contact_processing_threshold: manifold.contact_processing_threshold,
                };
                dispatcher.insert_persistent_manifold(key, stored);
            }
        }

        Ok(())
    }
}

/// Order a body pair for matching. The order is independent of body order.
#[inline]
fn ordered_pair(body0: usize, body1: usize) -> (usize, usize) {
    (body0.min(body1), body0.max(body1))
}

/// Copy one public contact point into the stored point layout.
///
/// A reversed copy swaps the two local points and the two world points and
/// negates the normal on body 1. The other values stay unchanged.
fn contact_point_to_bullet(point: &ContactPoint, reversed: bool) -> ManifoldPoint {
    let (local_point_a, local_point_b, pos_world_on_a, pos_world_on_b, normal_world_on_b) =
        if reversed {
            (
                point.local_point_b,
                point.local_point_a,
                point.pos_world_on_b,
                point.pos_world_on_a,
                -point.normal_world_on_b,
            )
        } else {
            (
                point.local_point_a,
                point.local_point_b,
                point.pos_world_on_a,
                point.pos_world_on_b,
                point.normal_world_on_b,
            )
        };

    ManifoldPoint {
        local_point_a,
        local_point_b,
        pos_world_on_a,
        pos_world_on_b,
        normal_world_on_b,
        distance_1: point.distance,
        combined_friction: point.combined_friction,
        combined_restitution: point.combined_restitution,
        applied_impulse: point.applied_impulse,
        is_special: point.is_special,
    }
}

/// Check a body state before a write.
fn validate_body_state(state: &BodyState) -> Result<(), StateError> {
    if !state.inv_inertia_tensor_world.is_finite() {
        return Err(StateError::NonFiniteBodyState {
            field: "inv_inertia_tensor_world",
        });
    }
    if !state.interp_world_basis.is_finite() {
        return Err(StateError::NonFiniteBodyState {
            field: "interp_world_basis",
        });
    }
    if !state.linear_sleeping_threshold.is_finite() {
        return Err(StateError::NonFiniteBodyState {
            field: "linear_sleeping_threshold",
        });
    }
    if state.linear_sleeping_threshold < 0.0 {
        return Err(StateError::NegativeSleepingThreshold {
            field: "linear_sleeping_threshold",
        });
    }
    if !state.angular_sleeping_threshold.is_finite() {
        return Err(StateError::NonFiniteBodyState {
            field: "angular_sleeping_threshold",
        });
    }
    if state.angular_sleeping_threshold < 0.0 {
        return Err(StateError::NegativeSleepingThreshold {
            field: "angular_sleeping_threshold",
        });
    }
    Ok(())
}

/// Check all manifolds before the first write.
fn validate_contact_manifolds(
    manifolds: &[ContactManifold],
    body_count: usize,
) -> Result<(), StateError> {
    let mut pairs = Vec::with_capacity(manifolds.len());

    for (manifold_index, manifold) in manifolds.iter().enumerate() {
        let body0 = manifold.body0.index();
        let body1 = manifold.body1.index();

        if body0 >= body_count {
            return Err(StateError::UnknownManifoldBody {
                manifold: manifold_index,
                body: body0,
            });
        }
        if body1 >= body_count {
            return Err(StateError::UnknownManifoldBody {
                manifold: manifold_index,
                body: body1,
            });
        }
        if body0 == body1 {
            return Err(StateError::SameBodyPair {
                manifold: manifold_index,
            });
        }

        let pair = ordered_pair(body0, body1);
        if pairs.contains(&pair) {
            return Err(StateError::DuplicateBodyPair {
                manifold: manifold_index,
            });
        }
        pairs.push(pair);

        if !manifold.contact_breaking_threshold.is_finite()
            || manifold.contact_breaking_threshold < 0.0
        {
            return Err(StateError::InvalidBreakingThreshold {
                manifold: manifold_index,
            });
        }
        if !manifold.contact_processing_threshold.is_finite()
            || manifold.contact_processing_threshold < 0.0
        {
            return Err(StateError::InvalidProcessingThreshold {
                manifold: manifold_index,
            });
        }
        if manifold.points.len() > MANIFOLD_CACHE_SIZE {
            return Err(StateError::TooManyPoints {
                manifold: manifold_index,
                points: manifold.points.len(),
            });
        }

        for (point_index, point) in manifold.points.iter().enumerate() {
            validate_contact_point(point, manifold_index, point_index)?;
        }
    }

    Ok(())
}

/// Check one contact point before the first write.
fn validate_contact_point(
    point: &ContactPoint,
    manifold: usize,
    index: usize,
) -> Result<(), StateError> {
    let is_finite = point.local_point_a.is_finite()
        && point.local_point_b.is_finite()
        && point.pos_world_on_a.is_finite()
        && point.pos_world_on_b.is_finite()
        && point.normal_world_on_b.is_finite()
        && point.distance.is_finite()
        && point.combined_friction.is_finite()
        && point.combined_restitution.is_finite()
        && point.applied_impulse.is_finite();

    if !is_finite {
        return Err(StateError::NonFinitePoint {
            manifold,
            point: index,
        });
    }
    if !point.normal_world_on_b.is_normalized() {
        return Err(StateError::NormalNotUnit {
            manifold,
            point: index,
        });
    }
    if point.applied_impulse < 0.0 {
        return Err(StateError::NegativeAppliedImpulse {
            manifold,
            point: index,
        });
    }

    Ok(())
}
