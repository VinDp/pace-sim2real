# DynaArm: velocity reference mismatch between the real controller and the PACE simulation

## Summary

During the chirp recordings, the real DynaArm controller used the **chirp's velocity reference**
in its D-term:

```
τ_real = P·(q_ref − q) + D·(q̇_ref − q̇)
```

PACE replays only the position reference (`des_dof_pos`). In the simulation the velocity target
is therefore zero, and the D-term acts on the measured velocity alone:

```
τ_sim  = P·(q_ref − q) + D·(0 − q̇)
```

The two control laws differ by exactly **`D·q̇_ref`**. That torque exists on the real robot and
is missing in the simulation, so the simulated arm brakes harder than the real one. During good
tracking (`q̇ ≈ q̇_ref`) the real D-term is close to zero, while the simulated one is about
`−D·q̇_ref`.

The missing torque is up to 7.5 Nm on the shoulder and elbow and up to 3.8 Nm on the wrist
joints. On the three wrist joints its RMS is **1–3× the total torque the real controller
produced**. PACE can't compensate for this with physical parameters, because it would need a
negative damping. That fits the earlier observation that "the parameters often went towards 0".
Replaying the recording in simulation with the velocity reference **halves the tracking error on
five of six joints** (section 5).

**`D = 0` is not the fix.** The fix is to give the simulation the same velocity reference
(option A below), or to record with a zero velocity reference (option B).

## Recording setup (from `old_data/calibration_*`)

| Item | Value | Source |
|---|---|---|
| Controller | ros2_control `pid_controller`, 1000 Hz, `command_interface: effort` | `pid_controller_params.yaml` |
| Reference interfaces | `position` **and** `velocity` | `pid_controller_params.yaml` |
| Feed-forward | off (`enable_feedforward: false`), no gravity compensation | `pid_controller_params.yaml` |
| Gains | P = 80, D = 2 (shoulder, elbow) · P = 60, D = 1 (forearm, wrist) · I = 0 | `pid_controller_params.yaml` |
| Chirp | `q_ref = home + A·sin(φ(t))`, `q̇_ref = A·2π·f(t)·cos(φ(t))`, A = 0.3 rad, 0.1 → 2 Hz | `metadata.yaml` |
| Reference rate | 400 Hz (the controller runs at 1 kHz and holds the last reference) | `metadata.yaml` |
| `chirp_data.pt` | 400 Hz, keys `time`, `dof_pos`, `des_dof_pos`, **`des_dof_vel`** | file contents |

The metadata states that `q̇_ref` was computed. The question was whether the controller actually
used it, which the rosbag answers.

## Evidence

### 1. The real control law, reconstructed from the rosbag

`/pid_controller/controller_state` (`control_msgs/MultiDOFStateStamped`) logs, per joint and
per 1 kHz tick: `reference`, `feedback` (q), `feedback_dot` (q̇), `error`, `error_dot` and
`output` (torque). Reconstructing the output with each candidate control law gives the
following result for `calibration_20260713_123006`:

| Control law | RMS residual vs. logged `output` |
|---|---|
| `P·e + D·(q̇_ref − q̇)` (with `q̇_ref = error_dot + feedback_dot`) | **0.000 Nm**: exact on all joints |
| `P·e − D·q̇` (what PACE simulates) | 2.95 Nm (shoulder, elbow), 1.48 Nm (wrist joints) |

The implied velocity reference `error_dot + feedback_dot` peaks at 3.769 rad/s. That equals
`A·2π·f_max = 0.3·2π·2`, the analytic chirp velocity.

### 2. What PACE simulates

- `scripts/pace/fit.py` loads only `time`, `des_dof_pos` and `dof_pos`, and sends `des_dof_pos`
  through the position action. It never reads `des_dof_vel`.
- The task only has a `JointPositionAction`, which calls only `set_joint_position_target`.
- Isaac Lab builds the actuator input from `joint_vel_target`, which stays at its default of
  0 (`articulation.py`, `_apply_actuator_model`).
- The PD law of the explicit actuator (`IdealPDActuator.compute`, which `PaceDCMotor` builds on)
  is `τ = K·(q_tgt − q) + D·(q̇_tgt − q̇) + τ_ff`, so with `q̇_tgt = 0` it reduces to `P·e − D·q̇`.

### 3. Size of the mismatch

Values are computed inside the 60 s window that `chirp_data.pt` covers
(`calibration_20260713_123006`). `u` is the torque the real controller commanded, and
`D·q̇_ref` is the torque missing from the simulation.

| Joint | P | D | RMS(u) [Nm] | max\|u\| [Nm] | RMS(D·q̇_ref) [Nm] | max\|D·q̇_ref\| [Nm] | RMS(D·q̇_ref) / RMS(u) |
|---|---|---|---|---|---|---|---|
| shoulder_rotation | 80 | 2 | 6.34 | 22.96 | 3.15 | 7.54 | 50 % |
| shoulder_flexion | 80 | 2 | 18.44 | 58.99 | 3.15 | 7.54 | 17 % |
| elbow_flexion | 80 | 2 | 14.69 | 40.34 | 3.15 | 7.54 | 21 % |
| forearm_rotation | 60 | 1 | 0.76 | 5.95 | 1.57 | 3.77 | **206 %** |
| wrist_flexion | 60 | 1 | 1.52 | 6.28 | 1.57 | 3.77 | **104 %** |
| wrist_rotation | 60 | 1 | 0.48 | 4.76 | 1.57 | 3.77 | **327 %** |

The wrist joints are hit hardest because they carry almost no gravity load. Their real torque
is small and mostly dynamic, so the artificial braking torque in the simulation dominates it.
On the shoulder and elbow, gravity torque makes up most of `u`, which dilutes the error, but
3 Nm RMS is still about as large as the friction PACE is supposed to identify.

### 4. Why this distorts the identification

PACE adjusts armature, viscous damping, Coulomb friction, encoder bias and delay until the
simulated `q(t)` matches the measured one. In the simulation, an extra torque of about
`−D·q̇_ref` brakes the motion. To compensate, the optimizer would need **negative** viscous
damping or friction, which the bounds (≥ 0) don't allow. So the damping and friction
parameters get pushed to their lower bound of 0, and the remaining error is spread over
armature, bias and delay. The parameters come out biased even when the loss looks acceptable.

### 5. Replay test: simulation with and without the velocity reference

The recorded chirp was replayed in the `Isaac-Pace-DynaArm-v0` simulation with the default
parameters: no friction, no damping, no bias, no delay, armature 2e-3 kg·m². The run used the
recording setup's gains and limits, and compared the simulated against the measured `q(t)`.
Only the velocity target differed between the two runs. PACE itself was unchanged; the test
sets the target in a standalone script.

| RMS error [rad] | sh_rot | sh_flex | el_flex | fa_rot | wr_flex | wr_rot |
|---|---|---|---|---|---|---|
| `q̇_tgt = 0` (what `fit.py` does now) | 0.086 | 0.148 | 0.137 | 0.053 | 0.048 | 0.057 |
| `q̇_tgt = des_dof_vel` | **0.040** | 0.132 | **0.078** | **0.028** | **0.027** | **0.032** |

Adding the velocity reference **roughly halves the error on five of six joints before any
parameter is fitted**. The results at armature 1e-2 kg·m² are almost identical.
`shoulder_flexion` barely improves (max error ≈ 1.1 rad in both runs), so a second, separate
issue affects that joint. Candidates, still to be checked:

- the 40 Nm torque limit: the real controller commanded up to 59 Nm on this joint, for about
  3 s inside the window. This turned out to be the cause: with the 120 Nm limit of the latest
  Duatic description, the fitted `shoulder_flexion` error drops from 0.034 to 0.014 rad RMS.
- the gravity load or masses

This is a likely explanation for the earlier fits that never converged and kept driving
parameters toward 0. The old setup had other problems too: its USD capped joint velocity at
4 rad/s, the first dataset came from a shaking table, and the robot still had its safety foam
on.

## Should we have used D = 0?

**No**, at least not as the fix. Setting `D = 0` on the real robot would make both control laws
equal to `P·e`, so the mismatch would vanish. But it would also:

- remove the controller's damping. With P = 60–80 and only the arm's own friction left, tracking
  the chirp becomes oscillatory, and resonance near the higher chirp frequencies is likely.
  That's a risk for the robot, and it degrades the data.
- leave the arm more sensitive to the table and cable vibrations that already spoiled the first
  dataset.

The problem isn't that D is non-zero. It's that the simulation doesn't receive the same velocity
reference as the real controller. Either option below fixes that and keeps the damping.

## Options

### A. Give the simulation the recorded velocity reference (recommended)

In `scripts/pace/fit.py`, also set the velocity target from `des_dof_vel` when the data
contains it:

```python
target_dof_vel = data["des_dof_vel"].to(env.unwrapped.device) if "des_dof_vel" in data else None
...
# in the loop, before env.step(actions)
if target_dof_vel is not None:
    articulation.set_joint_velocity_target(
        target_dof_vel[counter, :].unsqueeze(0).repeat(env.unwrapped.num_envs, 1), joint_ids=sim_joint_ids
    )
```

- The simulation then runs exactly `P·e + D·(q̇_ref − q̇)`, the real control law.
- With `delay_mode="command"` (the default), `PaceDCMotor` already delays the velocity target
  together with the position target, so the delay model stays consistent.
- The ANYmal data has no `des_dof_vel` key, so nothing changes for existing robots.
- The recorded datasets can be used as they are. `des_dof_vel` in `chirp_data.pt` matches the
  bag's velocity reference to within 0.049 rad/s RMS (resampling and timing from missed 400 Hz
  callbacks), against a peak of 3.77 rad/s.
- Downside: it changes a PACE script. The change is opt-in and only a few lines long. The
  replay in `data_collection.py` would need the same change if it's used for sim-to-sim tests.

### B. Re-record with a zero velocity reference

Send only position references (`q̇_ref = 0`) to the `pid_controller`. The real law becomes
`P·e − D·q̇`, which is what PACE already simulates.

- No change to PACE.
- Needs new recordings. The D-term then also brakes the real arm along the chirp, so tracking
  lags more, which is harmless for identification.
- It matches how an Isaac Lab policy usually runs later, with position targets and the D-term
  acting on `−q̇`. If the real deployment controller will run like that, B identifies the
  parameters under the same conditions they will be used in.

### C. Record with D = 0

As discussed above, not recommended.

## Recommendation

Use **A** to make the existing recordings usable right away. For new recordings, it's worth
deciding which control law the deployed policy will use and recording with that one (B, if it
will use position-only targets). Either way, the simulation and the real robot should share the
same control law. That rule also applies to the other recording conditions, which the DynaArm
task config already matches:

- P/D gains: 80/2 and 60/1
- torque limit: 120 Nm (shoulder, elbow) and 40 Nm (wrist joints), as in the latest Duatic
  description. A fit with 40 Nm on all joints underestimated the `shoulder_flexion` resonance
  (RMS error 0.034 vs. 0.014 rad), so the drive did not clamp at 40 Nm during recording.
- no gravity compensation, gravity on in the simulation
- 400 Hz `sim.dt`

## How the numbers were obtained

- Dataset: `old_data/calibration_20260713_123006` (60 s chirp, recorded after the table was
  secured).
- The `.pt` window was aligned to the rosbag by matching `des_dof_pos` to the logged
  `reference`. The chirp starts 3.295 s into the bag, and the `.pt` data starts 3.3445 s in.
  After alignment, the position reference agrees to within 0.0019 rad RMS and the measured
  position to within 0.002–0.003 rad RMS.
- The bag was decoded with the `mcap` and `mcap-ros2-support` Python packages. P and D were
  taken from `pid_controller_params.yaml`.
