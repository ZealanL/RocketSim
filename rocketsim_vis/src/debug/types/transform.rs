use super::{
    error::{DebugError, DebugResult},
    limits::finite3,
};

/// Reference frame for a group.
///
/// Offsets are in the anchor frame: `p_world = apos + arot * (p_group + offset)`.
/// `World` is identity (`apos = 0`, `arot = I`).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Anchor {
    /// Fixed world coordinates (Unreal units).
    #[default]
    World,
    /// Ball rigid transform plus a local offset.
    Ball {
        /// Offset in ball-local space (Unreal units).
        offset: [f32; 3],
    },
    /// Car transform by stable `CarInfo` identifier plus a local offset.
    /// If the car is gone the renderer falls back to world identity.
    Car {
        /// Stable `CarInfo` identifier.
        car_id: u32,
        /// Offset in car-local space (Unreal units).
        offset: [f32; 3],
    },
}

impl Anchor {
    /// Return the fixed world reference frame.
    #[must_use]
    pub const fn world() -> Self {
        Self::World
    }

    /// Return a ball anchor with zero local offset.
    #[must_use]
    pub const fn ball() -> Self {
        Self::Ball { offset: [0.0; 3] }
    }

    /// Return a car anchor with zero local offset for a stable car identifier.
    #[must_use]
    pub const fn car(car_id: u32) -> Self {
        Self::Car {
            car_id,
            offset: [0.0; 3],
        }
    }

    /// Check that the local offset is finite.
    ///
    /// This does not check whether a car identifier exists in a simulation.
    pub fn validate(&self) -> DebugResult<()> {
        match self {
            Self::World => Ok(()),
            Self::Ball { offset } => finite3(offset, "anchor offset"),
            Self::Car { offset, .. } => finite3(offset, "anchor offset"),
        }
    }
}

/// Group placement after the instance transform and before anchor placement.
///
/// Compute `p2 = position + rotation * (scale * p1)`. Store the matrix as
/// three rows: `rotation[row][column]`. Values must be finite. Validation
/// does not require an orthogonal matrix. The default is identity placement.
///
/// Convert a column-major `glam` matrix to rows before storing it:
///
/// ```
/// use glam::Mat3A;
/// use rocketsim_vis::debug::GroupTransform;
///
/// let rotation = Mat3A::from_rotation_z(std::f32::consts::FRAC_PI_2);
/// let transform = GroupTransform {
///     position: [100.0, 0.0, 0.0],
///     rotation: rotation.transpose().to_cols_array_2d(),
///     scale: [1.0; 3],
/// };
/// transform.validate()?;
/// assert!((transform.rotation[0][1] + 1.0).abs() < 1e-6);
/// # Ok::<(), rocketsim_vis::debug::DebugError>(())
/// ```
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GroupTransform {
    /// Translation in Unreal units.
    pub position: [f32; 3],
    /// Finite 3 by 3 row-major matrix. Each inner array is one row.
    pub rotation: [[f32; 3]; 3],
    /// Positive scale for X, Y, and Z; each value must be greater than 1e-6 and less than 1e6.
    pub scale: [f32; 3],
}

impl Default for GroupTransform {
    fn default() -> Self {
        Self::identity()
    }
}

impl GroupTransform {
    /// Return zero translation, identity rotation, and unit scale.
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            position: [0.0; 3],
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            scale: [1.0; 3],
        }
    }

    /// Return identity rotation and scale with a translation in Unreal units.
    #[must_use]
    pub const fn from_position(position: [f32; 3]) -> Self {
        Self {
            position,
            ..Self::identity()
        }
    }

    /// Return identity rotation with a translation and a uniform scale.
    #[must_use]
    pub const fn from_position_scale(position: [f32; 3], scale: f32) -> Self {
        Self {
            position,
            scale: [scale; 3],
            ..Self::identity()
        }
    }

    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        finite3(&self.position, "group position")?;
        finite3(&self.scale, "group scale")?;
        for row in &self.rotation {
            finite3(row, "group rotation")?;
        }
        if self.scale.iter().all(|s| *s > 1e-6 && *s < 1e6) {
            Ok(())
        } else {
            Err(DebugError::InvalidTransform)
        }
    }
}

/// Explicit line/point thickness.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Width {
    /// Screen-space pixels. Valid values are finite and in `(0, 64]`; the default is 2 pixels.
    Pixels(f32),
    /// Unreal units. Valid values are finite and in `(0, 10000]`.
    World(f32),
}

impl Default for Width {
    fn default() -> Self {
        Self::Pixels(2.0)
    }
}

impl Width {
    /// Create a pixel width without validation. Valid values are in `(0, 64]`.
    #[must_use]
    pub const fn pixels(px: f32) -> Self {
        Self::Pixels(px)
    }

    /// Create a width in Unreal units without validation.
    ///
    /// Valid values are finite and in `(0, 10000]`.
    #[must_use]
    pub const fn world(w: f32) -> Self {
        Self::World(w)
    }

    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        let v = match *self {
            Self::Pixels(v) => {
                if !(v.is_finite() && v > 0.0 && v <= 64.0) {
                    return Err(DebugError::InvalidWidth);
                }
                v
            }
            Self::World(v) => {
                if !(v.is_finite() && v > 0.0 && v <= 10_000.0) {
                    return Err(DebugError::InvalidWidth);
                }
                v
            }
        };
        let _ = v;
        Ok(())
    }
}

/// Depth handling per item.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum DepthMode {
    /// Depth-tested (hidden behind arena geometry).
    #[default]
    Test,
    /// Always drawn on top.
    Always,
    /// Draw hidden parts with reduced opacity and visible parts normally.
    Xray,
}

/// Expiry rule for a retained group.
///
/// Group replacement starts a new deadline. Transform, visibility, anchor,
/// and item updates preserve the deadline. Tick deadlines follow simulation
/// updates, not rendered frames or wall-clock time. Seconds deadlines use
/// absolute Unix time, so clock differences can affect remote viewers.
///
/// ```
/// use rocketsim_vis::debug::{DebugRenderer, GroupOptions, Lifetime};
///
/// let renderer = DebugRenderer::new_detached();
/// renderer.group("temporary").draw_with(
///     GroupOptions::new().lifetime(Lifetime::Frame),
///     |draw| {
///         draw.point([0.0; 3]);
///     },
/// )?;
/// renderer.advance_tick();
/// assert!(renderer.snapshot().groups.contains_key("temporary"));
/// renderer.advance_tick();
/// assert!(!renderer.snapshot().groups.contains_key("temporary"));
/// # Ok::<(), rocketsim_vis::debug::DebugError>(())
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Lifetime {
    /// Kept until explicitly replaced or cleared.
    #[default]
    Persistent,
    /// Show with the next simulation snapshot and expire at the following update.
    ///
    /// The viewer keeps it if the simulation pauses at that snapshot.
    /// This is not a wall-clock timeout.
    Frame,
    /// Remain through the next `n` simulation updates, then expire.
    ///
    /// Accept `1..=1_000_000`. At update `t`, the deadline is `t + n + 1`.
    Ticks(u32),
    /// Expires after finite wall-clock seconds in 0.001..=3600.
    Seconds(f32),
}

impl Lifetime {
    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        match *self {
            Self::Persistent | Self::Frame => Ok(()),
            Self::Ticks(n) => {
                if (1..=1_000_000).contains(&n) {
                    Ok(())
                } else {
                    Err(DebugError::InvalidLifetime)
                }
            }
            Self::Seconds(s) => {
                if s.is_finite() && (0.001..=3600.0).contains(&s) {
                    Ok(())
                } else {
                    Err(DebugError::InvalidLifetime)
                }
            }
        }
    }
}
