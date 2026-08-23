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

## 2024 session addendum — chaos confirmed, thresholds ruled out

Instrumented RS's raycast directly (`RAY hit/miss pz=... trace=...`) during
simple_jump_land's flight:

- At the marginal tick: `RAY miss pz=32.3138` while RL's recorded pos_26 is
  `31.6988` — a **0.62 uu trajectory divergence** by tick 26 of a ~146-tick
  flight, far below the harness position tolerance.
- That drift flips the marginal contact decision (RS trace ≈ 1.0615 vs
  R = 1.0551), which produces the 34 uu/s landing velocity error.
- Parameter sweep over ray-subtraction {0, 0.025} × engage-margin
  {0, 0.01, 0.02}: **identical results for every combo** — trio fails at
  exactly i=190/i=127/i=127, all takeoff tests pass. Contact-boundary
  parameters cannot fix a chaotic input difference.
- Removing the comparison shift makes ALL jump tests fail at their takeoff
  bool instead — confirming the shift compensates real sub-tolerance
  trajectory noise at the disengage boundary.

**Conclusion**: the landing failures are chaotic amplification of airborne
force differences that are individually below comparison tolerance. The fix
is not in wheel/suspension code at all — it is eliminating per-tick
airborne-phase force deltas (candidates: AirControl impulse rounding, angular
damping removal follow-ups from v3-rust, gravity/damping application order)
until flight trajectories match to near-ulp level. Only then do contact
events align and the trio can pass.

Plugin-source confirmations (rl_phys_record/src):
- TickRecord[t] snapshots at PreBullet of frame t ("Record FIRST ... PREVIOUS
  tick's bullet step" comment in hooks.cpp) — phys is end-of-frame t-1,
  impulses listed are from tick t-1's execution.
- RL has separate `bt_vehicle_update_susp_forces` and
  `bt_vehicle_update_friction` phases inside its vehicle update.
- Game wheel info includes `suspensionSubractionIdk` (validates our 0.05
  subtraction constant) and an RE-unidentified `always1_groundStickIdk_D0`
  flag worth investigating for contact persistence semantics.

## SOLVED-model addendum: contact hysteresis (Schmitt trigger)

Instrumented `update_suspension` firing site + raycast traces resolved all
contradictions. Constraint set (hardpoint heights above ground, Octane):

- RL does NOT engage at pos_191 of simple_jump_land: hardpoint height
  1.023732 BT, free-fall continues (vel_191 = -392, no impulse).
- RL DOES engage at pos_192: hardpoint height 0.963098 (vel_192 = -363,
  braking present; recorded len 0.76386).
- RL HOLDS contact through single_jump pos_26: hardpoint height 1.049076 —
  ABOVE the landing non-engage bound!
- Released by pos_27: height 1.094323.

A single ray threshold cannot satisfy "miss at 1.0237" + "hit/hold at
1.0491". RL therefore uses **contact hysteresis**, matching the SDK field
`always1_groundStickIdk_D0` ("ground stick"):

- ENGAGE (airborne -> contact): requires trace <= E, E in [0.9631, 1.0237).
- HOLD (already in contact): persists while trace <= D, D in [1.0491, 1.0943).

Implementation sketch (baseline ordering, post-integration vehicle phase):

```rust
// prepare_for_raycast:
let reach = if self.raycast_info.is_some() { HOLD_REACH } else { ENGAGE_REACH };
let target = hard_point - up * reach;
// HOLD_REACH ~= 1.06 BT, ENGAGE_REACH ~= 1.01 BT (mid-band, refine empirically)
```

Open detail: RL's first-contact stored length (0.76386) equals neither
geo(pos_192)=0.71262 nor geo(pos_191)=0.77326 — it sits between, suggesting
either mid-step sampling of the integrating length or the 0.977-relaxation
applied from the clamped carry-over. If hysteresis alone leaves small len
residuals, revisit the srv-integration rule ONLY for the contact-persistent
branch (settle-time srv divergence must be fixed first — see srv notes above).

Also verify `always1_groundStickIdk_D0` semantics against game memory: if it
is a literal stick flag rather than a scalar, HOLD may persist regardless of
trace until wheels_with_contact would drop for another reason (e.g. jump),
bounded only by the observed 1.0943 release.

## Post-rebase addendum: divergence isolated to extra_pushback

After merging da64013 (vehicle phase pre-integration) + b6f2632 (shift
drop), suite jumped to **10/21** and the landing trio's failures moved
PAST touchdown: they now fail ~10 ticks later during the settle bounce
(sjl i=199 e=1.50, hjrl/dwj i=139 e=1.07-1.08). Landing impact itself is
correct under the new ordering.

Per-tick waveform comparison through hjrl's post-landing bounce shows
len, srv and vel matching to 3-4 decimals until i=137, then:

```
        RS pushF/pushB      RL pushF/pushB     excess
i=137   23.145 / 28.963     21.559 / 27.247    +1.59/+1.72
i=138    12.218 /  17.326    9.862 / 14.433    +2.36/+2.89
i=139     6.462 /   9.911    3.944 /  6.654    +2.52/+3.26
```

The diverging component is **extra_pushback** (resolve_single_collision
output when chassis trace < rest1+radius-SUBTRACTION). len and srv match
exactly; the resolver produces different output for the same inputs.
Suspects: RL's resolver ERP (RS uses 0.2), its velocity basis, or an
additional clamp. Determining RL's constants requires game-memory
experiments (ztour hooks can set penetration and read output) or deeper
RE of the game's modified resolve function.

Note: an acceptance-filter variant of hysteresis was also tested
(ENGAGE=1.02/HOLD=1.06 applied in apply_ray_cast) - suite-neutral (10)
but shifted sjl's failure tick, so reverted. Re-run this experiment only
after the resolver question is answered.

## RESOLVED: wheel pushback resolver uses ERP = 0.1 (2026-08-22)

Offline fit of the game's own recorded `extra_pushback` outputs against
the closed-form Bullet resolver, using recorded chassis state as input:

    raw = max(0, ERP*(-delta)/dt + (-rel_vel)) / denom ;  pb = raw/4
    delta = (hard_point - hit).up - (rest1 + radius - SUBTRACTION)

Method: decode WheelRecord.extra_pushback (@44) from RLPR bytes, convert
recorded pos/lin_vel UU->BT, evaluate predictor per tick/wheel, solve for
the implied ERP. Results across hjrl settle-bounce events:

    tick138 F/B: implied ERP 0.0992/0.0997   (rel_vel -1.70)
    tick139 F/B: implied ERP 0.0995/0.0997   (rel_vel -0.64/-0.77)
    tick140 F/B: implied ERP 0.1002/0.1000   (rel_vel -0.12/-0.23)

i.e. the game uses **ERP = 0.1**, not Bullet's default 0.2 that RocketSim
had. With ERP' = 0.1 the max(0,...) clamp also reproduces the exact ticks
where RL's pushback is zero.

Gotchas that invalidated earlier fits (document so nobody re-trips):
- Recording rot triples are matrix ROWS; basis vectors are COLUMNS
  (fwd_w = (m00,m10,m20)). Using rows as basis shifts front trace by
  ~+0.021 BT and back by ~-0.008 on a pitched car.
- Recording pos/lin_vel are in UU; convert (*UU_TO_BT) before mixing
  with BT geometry.
- suspension_rest_length_1 = config rest - MAX_SUSPENSION_TRAVEL
  (front 0.5351, back 0.5011 BT for Octane).
- Recorded wheels[T] pair with raycast(state[T]) same-phase for
  len/srv; the pushback VALUES fit state[T-1] best (one substep lag,
  likely a hook-pipeline artifact) - magnitudes confirmed with ERP=0.1
  to 0.06-0.16% on strong events. RS keeps its self-consistent phase;
  only the constant changed.

Fix: contact_solver_info::WHEEL_PUSHBACK_ERP = 0.1 used by
resolve_single_collision (sole caller is the wheel pushback path).
Landing trio (simple_jump_land, has_jumped_reset_landing,
double_jump_window_edge) now PASSES. Suite: 13 passed / 8 failed.
The remaining 8 were failing before this change.

## Session addendum 2: ball sleep, sticky gate, remaining failures triaged

### Fixed: ball sleep-on-zero-velocity (idle, freefall_bounce now pass)
`Arena::step_tick` put the ball to Sleep whenever lin/ang vel were exactly
zero. A ball spawned at rest in MID-AIR therefore never gained gravity,
while recordings show the game integrates it (-650*dt on tick 1).
Removed the block; ball's own CanSleep path is inert anyway
(linear_sleeping_threshold = 0). Suite 13 -> 15.

### Fixed: sticky force gated on last tick's world contact
Game does not apply StickyForce on a spawn tick whose fresh raycast sees
ground (car spawned "airborne" flag-wise): supersonic_brake's first-tick
car vz matched once sticky used `sticky_gate_prev` (stored at end of
update_vehicle_second) instead of fresh raycast contact. Kept impulse in
its original pipeline position (order-sensitive f32 accumulation).

### Recording-file artifacts (user will regenerate records)
- boost_run: recorded boost starts at 100, snaps to exactly 1.0 at t12,
  then drains 0.002775/tick WHILE applying full boost acceleration every
  tick (+8.2646 uu/s = 991.67/120). Tank math impossible; RS empties a
  real 33.33 tank at t130 -> one missing boost tick diverges vx by
  exactly one boost-tick. Not an RS bug.
- front_flip / side_dodge: flip_rel_torque convention mismatch in
  records (user handling).

### OPEN: backboard_slope_descend (i=88, chassis-box world contact)
Only two recordings exercise car-vs-world chassis contacts:
falling_ball_aerial_touch (6 events, PASSES - full restitution 0.3
bounce matches) and backboard_slope_descend (1 event, FAILS).

The failing event: car level, falling vz=-1119.76 vy=-400, box hits
backboard slope (normal ~+z flat triangle), pen -0.10025 BT.
- RS single manifold point converges fully IN-TICK: kills to rest target
  (+6.72 BT/s separation), COM dvz=+19.50 BT/s -> vz=-150.86 after tick.
- RL spreads the kill over THREE ticks: dvz = 535/390/178 uu/s per tick,
  never bounces, settles into a chassis slide at z~70 with wheels NEVER
  contacting (rays miss on the slope). RL's per-tick response is weaker
  than even kill-to-zero, i.e. NOT a converged single-constraint solve.
- restitution=0 experiment: halves backboard error (152->75) but breaks
  fbat -> restitution IS right for gentle events. Discriminator unknown.

Leading hypothesis: game resolves multi-point manifolds (box face flush
on coplanar triangles -> up to 4 pts) with limited effective iterations
or sequential point distribution, so correction leaks across ticks;
RS generated only ONE cached point here and converges immediately.
Next steps: dump RS point_cache contents across ticks leading up to
impact (persistent manifold refresh logic), compare against game's
collision_records (ContactRecord stream in rlpr? not currently
recorded), consider emulating partial-per-tick response.

Debug tooling kept (env-gated, zero cost when unset):
- Arena::get_car_wheel_debug(idx) -> [(contact,len,srv);4]
- RS_TRACE=1 harness per-tick PRED/REAL pos+vel dump
