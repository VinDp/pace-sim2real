# DynaArm PACE: status and open issues

Last updated: 2026-11-17. This file tracks what has been set up, what was found and what is
still open. The details of the velocity reference issue are in
[dynaarm_velocity_reference.md](dynaarm_velocity_reference.md).

## Current state

PACE fits the DynaArm well on the existing recordings. With the recorded velocity reference in
the simulation and the torque limits of the latest Duatic description, the fitted simulation
tracks the real arm to **2–15 mrad RMS per joint**. The fitted parameters **generalize** to a
held-out recording (validation loss within 10 % of the training loss).

Several fitted parameters still look like they compensate for model errors rather than
describe the hardware: encoder biases at their bounds, and an elbow armature at its lower bound.
The main suspects are the gravity model and the shoulder/elbow drive coupling. See
[Open issues](#open-issues).

## Branches and commits

| Branch | Commit | Change |
|---|---|---|
| `dev` | `10b454f` | Docker setup on `nvcr.io/nvidia/isaac-lab:2.3.2` (Isaac Sim 5.1) |
| `dev` | `a587648` | README: Docker workflow |
| `dev` | `f6bb4df` | `list_envs.py` lists `Isaac-Pace` tasks |
| `dev` | `e70940f` | `scripts/dynaarm/build_usd.py`: USD from Duatic `718b7b6`, baracuda12 |
| `dev` | `b52e5ea` | Task `Isaac-Pace-DynaArm-v0` |
| `dev` | `828e5eb` | Report on the velocity reference mismatch |
| `velocity_reference` | `d17ea51` | `fit.py` uses `des_dof_vel` as velocity target when the data has it |
| `velocity_reference` | `dee80ec` | Torque limits 120 Nm (shoulder, elbow) / 40 Nm (wrist joints) |

Not committed on purpose:
- `old_data/` and `old_extension/` (local, excluded through `.git/info/exclude`)
- generated assets (`source/pace_sim2real/pace_sim2real/assets/dynaarm/`)
- data, logs and test scripts under `data/dynaarm/` and `logs/`

## Recording conditions (`old_data/calibration_*`)

| Item | Value |
|---|---|
| Robot | DynaArm baracuda12, mounted upright on a table |
| Controller | ros2_control `pid_controller`, 1 kHz, effort interface, P·e + D·(q̇_ref − q̇), no feed-forward or gravity compensation |
| Gains | P/D = 80/2 (shoulder, elbow), 60/1 (forearm, wrist) |
| Excitation | Chirp 0.1 → 2 Hz, amplitude 0.3 rad on all joints, around home (0, −0.5, 1.6, 0, 0, 0) |
| Data | `chirp_data.pt` at 400 Hz with `time`, `dof_pos`, `des_dof_pos`, `des_dof_vel` |
| Datasets | `20260616_113653`: unusable (shaking table, safety foam). `20260713_123006`: 60 s, **used for fitting**. `20260713_123212`: 20 s, **used for validation** |
| Driver limits | The URDF at recording time said 40 Nm and 4 rad/s, but the driver only started applying URDF limits on 2026-10-14 (Duatic `63ce91b`), so they were not enforced |

## Results

Both fits use 4,096 environments, about 50 s per CMA-ES iteration, on the 60 s dataset.

| RMS error [rad] | sh_rot | sh_flex | el_flex | fa_rot | wr_flex | wr_rot | Loss |
|---|---|---|---|---|---|---|---|
| Unfitted, no velocity reference | 0.086 | 0.148 | 0.137 | 0.053 | 0.048 | 0.057 | – |
| Fit, 40 Nm (189 it.) | 0.012 | 0.034 | 0.020 | 0.003 | 0.004 | 0.002 | 0.00169 |
| **Fit, 120/40 Nm (155 it.)** | **0.009** | **0.014** | **0.012** | **0.003** | **0.003** | **0.002** | **0.00042** |
| Validation (held out), 40 Nm | 0.011 | 0.031 | 0.018 | 0.003 | 0.004 | 0.002 | 0.00142 |
| **Validation (held out), 120/40 Nm** | **0.008** | **0.015** | **0.013** | **0.003** | **0.003** | **0.002** | **0.00046** |

Parameters of the 120/40 Nm fit (best sample of the last iteration):

| | sh_rot | sh_flex | el_flex | fa_rot | wr_flex | wr_rot |
|---|---|---|---|---|---|---|
| Armature [kg·m²] | 0.042 | 0.067 | 0.002 ⚠ | 0.005 | 0.013 | 0.013 |
| Viscous friction [Nm·s/rad] | 0.05 | 0.45 | 0.11 | 0.04 | 0.06 | 0.01 |
| Coulomb friction [Nm] | 1.28 | 1.20 | 0.18 | 0.59 | 0.46 | 0.28 |
| Encoder bias [rad] | n/i | −0.050 ⚠ | −0.014 | −0.031 | +0.100 ⚠ | n/i |
| Delay | 0.43 steps, cast to 0 | | | | | |

⚠ = at a bound or implausible. n/i = not identifiable (see issue 2); the `forearm_rotation` and
`wrist_flexion` biases are also not identifiable at this pose.

Logs: `logs/pace/dynaarm/26_11_16_23-20-38` (40 Nm) and `logs/pace/dynaarm/26_11_17_07-48-25`
(120/40 Nm), each with `fit_overview.png` and `fit_zoom.png`.

## Gravity check

While the arm is static at home (2.5–5 s before and after each chirp), the PID output equals
the torque holding the arm against gravity, give or take static friction. The lead-in comes
after a move to home and the lead-out after a step back to it, so friction pushes the two in
opposite directions, and their midpoint estimates the pure gravity torque. The model torques are
computed from the URDF at the measured joint angles.

Midpoint of lead-in and lead-out, datasets `20260713_123006` / `20260713_123212` (the
`20260616` dataset had safety foam mounted and reads 1.5 Nm higher on the elbow):

| Joint | Measured [Nm] | Duatic latest (forearm 1.237 kg) | Recorded URDF (forearm 1.337 kg) | Half spread (≈ friction band) |
|---|---|---|---|---|
| shoulder_flexion | +1.03 / +0.90 | +0.16 / +0.12 | +0.10 / +0.06 | ±0.62–0.75 |
| elbow_flexion | −8.25 / −8.25 | −8.41 / −8.41 | −8.66 / −8.67 | ±0.37–0.41 |
| wrist_flexion | −1.05 / −1.06 | −0.90 / −0.90 | −0.90 / −0.90 | ±0.12 |
| other joints | ≈ ±0.1 | ≈ 0 | ≈ 0 | – |

- **The gravity model is good.** The elbow is within 0.16 Nm with the latest Duatic masses and
  0.42 Nm with the recorded URDF, so the latest description (forearm 1.237 kg) fits better. The
  wrist is within 0.15 Nm.
- **`shoulder_flexion` has an unmodelled offset of about +0.8–0.9 Nm**, larger than the friction
  band (issue 1).
- **Static friction as a lower bound:** the half spread is a lower bound on the static friction.
  The fitted Coulomb friction satisfies it on `shoulder_flexion` (1.20 ≥ 0.6) and `wrist_flexion`
  (0.46 ≥ 0.12), but **not on `elbow_flexion` (0.18 < 0.4)**. That fits the drive coupling
  (issue 3): the elbow drive's friction acts on the sum of both joint velocities.

## Resolved issues

| Issue | Finding | Resolution |
|---|---|---|
| Velocity reference | The real PID used D·(q̇_ref − q̇), while PACE simulated D·(0 − q̇). The missing torque was up to 7.5 Nm, and 1–3× the real torque on the wrist joints | `fit.py` sets `des_dof_vel` as velocity target (`d17ea51`). The unfitted error is halved on 5/6 joints |
| Torque limit | 40 Nm in the sim clamped the shoulder flexion, but the real controller commanded up to 59 Nm and the driver did not enforce URDF limits in July | Duatic limits 120/40 Nm (`dee80ec`). Loss 4× lower, shoulder flexion RMS 0.034 → 0.014 rad |
| Old USD velocity cap | The old USD capped every joint at 4 rad/s in PhysX, but the real joints reached 9.35 rad/s | New USD from the latest Duatic URDF, and `velocity_limit_sim` set explicitly |
| Gains | First version of the task used Duatic's drive PD gains (600/20…) | Recorded PID gains 80/2, 60/1 |
| Numerical stability | At 400 Hz the explicit PD is unstable on the wrist rotation for armature ≤ 1e-3 kg·m² | Armature lower bound 2e-3 kg·m² |
| Isaac Sim shutdown hang | `simulation_app.close()` hangs after long headless runs | Test scripts exit with `os._exit(0)`. `fit.py` saves its results before closing, so they are safe |

## Open issues

1. **Unmodelled static torque on `shoulder_flexion`.** At home the real arm holds about
   +0.8–0.9 Nm more than the gravity model predicts (see [Gravity check](#gravity-check)). The
   fitted bias of −0.050 rad reproduces exactly this (+1.07 Nm through the gravity gradient). The
   cause is open: a real encoder offset of about 3°, cable or hose forces, or a torque offset in
   the drive. A single home pose can't separate them. *Next: record at several poses.*
2. **Biases that can't be identified at the home pose.** The bias shifts the controller and the
   loss equally, so only the gravity gradient makes it observable. At home that gradient is
   almost zero for `shoulder_rotation`, `forearm_rotation`, `wrist_flexion` and `wrist_rotation`
   (≤ 0.34 Nm/rad). Their fitted biases, including `wrist_flexion` at +0.1 rad, are flat
   directions of the loss and carry no information. That's also why the saved `mean_*.pt` differs
   from the best sample. *Option: fix these four to 0 with zero-width bounds; recordings at other
   poses would make them identifiable.*
3. **Shoulder/elbow drive coupling.** The elbow drive measures `q_shoulder_flexion +
   q_elbow_flexion` and delivers `τ_elbow_flexion`, while the shoulder drive delivers
   `τ_shoulder_flexion − τ_elbow_flexion` (Duatic `kinematic_translation.cpp`; hardware since at
   least May 2026). The armature and friction of the elbow drive therefore act on the *sum* of
   both joint velocities. That gives an off-diagonal inertia term and friction in drive space,
   neither of which PACE's per-joint model can represent. The elbow armature pinned at its lower
   bound and the large shoulder-flexion armature (≈ shoulder + elbow drive) may be symptoms.
   *Option: a DynaArm-specific actuator that computes the friction in drive space.*
4. **Armature lower bound.** The 2e-3 kg·m² bound is there for stability, not physics, and the
   elbow is pinned to it. A smaller `sim.dt` would allow a lower bound, but `fit.py` runs one sim
   step per data sample, so the data would have to be resampled.
5. **Delay resolution.** The delay is a whole number of 2.5 ms steps. The fitted 0.43 rounds down
   to 0, which is plausible for a 1 kHz controller, but sub-step delays can't be represented.
6. **Limited validation.** Both datasets share the same day, pose and amplitude. Generalization to
   other poses (where gravity loads differ) and to larger amplitudes is untested.
7. **`data_collection.py`** is hard-coded for ANYmal's 12 joints, so there are no sim-to-sim
   tests for the DynaArm yet.

## Next steps

1. Fix the four unidentifiable biases (issue 2) to 0 and refit, to get a cleaner parameter set
   for the same loss.
2. Decide how to handle the shoulder/elbow coupling (issue 3): accept the per-joint
   approximation, or model the elbow drive's friction and armature in drive space.
3. New recordings at several poses and amplitudes: they separate the encoder bias from the
   `shoulder_flexion` offset (issue 1) and validate the fit beyond the home pose (issue 6).

## Notes for future recordings

- Since 2026-10-14 the Duatic driver **applies the URDF limits**, so new recordings are clamped
  at 120/40 Nm and 10.64/13.647 rad/s. The installed driver version decides which limits apply.
- Keep the simulation and the real controller on the same control law. Either keep sending q̇_ref
  (supported now) or send zero velocity references. The choice should match the controller the
  policy will use later.
- Record several home poses and amplitudes, to make gravity and encoder bias separable (issue 1)
  and to validate issue 6.
- Keep the table stiff and the arm free of foam and loose cables (dataset `20260616` notes).

## Q&A

**Could the wrist joints need a 120 Nm limit too?** No. The wrist joints never commanded more
than 6.3 Nm (`forearm_rotation` 5.95, `wrist_flexion` 6.28, `wrist_rotation` 4.76), far below
their 40 Nm limit, and the latest Duatic description keeps them at 40 Nm. The torque–speed curve
of the DC motor model (120 Nm saturation) never clips any recorded torque on any joint either.

**What else changed in the Duatic description since the recordings?**

| Change (Duatic commit) | Relevant for PACE? |
|---|---|
| Driver applies URDF joint limits (`63ce91b`, Oct 14) | Yes: explains why the 40 Nm were not enforced in July; matters for new recordings |
| Torque/velocity limits 120/40 Nm, 10.64/13.647 rad/s | Yes: used now |
| Forearm mass 1.337 → 1.237 kg (the extra 100 g for freedrive was reverted) | Possibly: affects gravity; checked in the gravity check |
| `base_plate` link (5 mm) with a duplicated 1.94 kg inertial | No: fixed base |
| Wrist rotation limits −π/2…3π/2 for the new cover (`a5bc6f5`) | No: the chirp stays within ±0.3 rad |
| Collision shapes (capsules, `d73e4a8`), mesh offsets (`f91155f`), new meshes | No: no contacts, visuals only |
| Shoulder/elbow coupling in the driver (unchanged since May) | Yes: issue 3 |
