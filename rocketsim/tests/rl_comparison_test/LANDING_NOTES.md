# Landing impact trio — investigation notes

Status: `simple_jump_land`, `has_jumped_reset_landing`, `double_jump_window_edge`
fail at their touchdown ticks with large velocity errors. Everything else about
jump parity is exact (see passing scenarios).

## Verified RL ground truth

All numbers from `.rlpr` recordings; positions in UU, lengths in Bullet units
(BT = UU/50), `TICK_TIME = 1/120`.

- Wheel contact geometry anchor (drive_straight t1):
  `H_conn = len + radius − pos.z·0.02 = 0.414623 BT` (front wheels, Octane).
  Back wheels share the same connection z for Octane.
- **Recorded `len(t) = geo(pos(t))`** — same-tick geometry, no sampling skew.
  Earlier "stale-by-one-tick" claims were calibration artifacts.
- Recorded `len` is NOT pure instantaneous geometry: during fast
  extension/compression it lags with ratio ≈ 0.977 (front) / 0.967 (back) per
  tick of the remaining gap to geometric length. Equivalent formulation:
  `Δlen ≈ srv · TICK_TIME` where `srv` is the recorded
  suspension_relative_vel (validated on 2288 grounded tick pairs, median
  residual 4e-5 BT). During idle settle, `len` creeps very slowly (~−0.001
  BT/s) — a genuine spring micro-dynamic.
- Landing engagement (simple_jump_land): first contact tick t192,
  `len = 0.76386`; the previous tick t191 is airborne with
  `trace = pos·0.02 + H_conn = 1.0886 > R`, so plain ray geometry decides
  engagement correctly given `R = rest1 + travel + radius = 1.02510`.
- Takeoff disengage (single_jump): last contact t26 (`len 0.74246`),
  first miss t27. Also pure geometry.

## The actual defect

RS applies wheel forces one **transition** off at engagement/disengagement
events:

- At simple_jump_land i=190 (free-fall tick), PRED carried
  `WheelsSuspension +34.0 z` while REAL had none → vel error 34 uu/s.
- Mechanism: RS's post-integration raycast hits marginal geometry one tick
  before RL's does *or* applies the resulting impulse to the wrong stored
  velocity, depending on ordering. Each reordering tried fixed landing and
  broke takeoff/activation — see below.

## Impulse forensics at jump activation (single_jump i=20)

Ordering experiment with vehicle phase before game logic produced:

```
PRED: StickyForce, WheelsSuspension +8.166, WheelsFriction, Jump(+291.65),
      Jump hold(+12.15)
REAL: StickyForce, Jump(+291.65), Jump hold(+12.15), WheelsFriction(−3.56)
      ← no WheelsSuspension
```

Interpretation: RL computes suspension/friction **after** game impulses —
the jump-induced upward velocity makes `srv` strongly positive, and
`force − damping_vel_scale · srv` goes negative → early return → no
suspension impulse on the activation tick. Any correct fix must preserve
this interplay (vehicle phase AFTER game logic, BEFORE integration).

## Failed hypotheses (do not retry as-is)

1. **Engage-margin hysteresis** (`SUSPENSION_ENGAGE_MARGIN`, reject marginal
   hits from previously-airborne wheels): margin ≤ 0.0112 needed to accept
   the legit sjl engagement; margin large enough to reject RS's spurious hit
   was ~0.04 → rejected legit engagements too. Geometric thresholds cannot
   separate the cases because the discriminator is the lagged length, not
   trace.
2. **srv-integration of suspension length**
   (`len ← clamp(prev.len + srv·dt)` for continuing contact, snap on fresh
   contact): matches RL's recorded Δlen statistics but diverges during spawn
   settle — RS's settle-time `srv` differs from RL's (micro-oscillation
   ω×r term; e.g. RS −0.16 vs RL 0.000 BT/s), and integration accumulates
   that error into force within a few ticks (all tests fail by i≈3).
3. **Vehicle-phase reordering** (before/after game logic × before
   integration): before-game breaks activation (+8.17 = suspension fired
   when RL suppresses it); after-game fixes activation but leaves takeoff
   disengage one tick late (pure bool, norm_error 1.0 at i=25).

## Sharpest untested hypothesis

RS's `srv` (= `contact_normal · (lin_vel + ω×r_contact)`) differs from RL's
during low-speed transients. Check whether RL computes the contact-point
velocity WITHOUT the rotational term, or samples velocity pre-damping:
`predict_unconstraint_motion` calls `apply_damping` every step in RS; if RL
damps at a different point in its frame, settle-time srv changes sign/magnitude.
Fixing settle srv makes hypothesis 2 viable again, which should fix all three
landings plus possibly aerial_ascent/front_flip/side_dodge.

## Harness notes

- `shifted_on_ground` in mod.rs compares `is_on_ground` against recording
  index t+1; this compensates RS-vs-RL contact timing under the CURRENT
  (post-integration) vehicle phase. If wheel force timing is ever truly
  fixed, try removing the shift — landing bools then align unshifted.
- Test init races without the `Once` guard produce nondeterministic
  pass/fail flips across the suite; keep the guard.
