<p align="center">
  <img src="https://github.com/user-attachments/assets/060f7774-a73f-4132-9413-36887ed09cfa" alt="Amrita Vishwa Vidyapeetham" width="430">
</p>

<h1 align="center">Full Quaternion-Based Attitude Control for a Quadrotor<br>+ Differential-Flatness Trajectory Tracking</h1>

<p align="center">
  <b>A Python Software-In-The-Loop Reproduction, Stability Analysis, 6-DOF Extension,<br>
  Four-Simulator Deployment, and Path-Planning Implementation of Two Papers</b><br>
  Group 12 · B.Tech AI Engineering · School of Artificial Intelligence, Amrita Vishwa Vidyapeetham
</p>

<p align="center">
  <img alt="Python" src="https://img.shields.io/badge/python-3.10%2B-3776AB?logo=python&logoColor=white">
  <img alt="Tests" src="https://img.shields.io/badge/tests-85%20total%20%2883%20pass%20%2B%202%20gated%29-brightgreen">
  <img alt="Stack" src="https://img.shields.io/badge/stack-numpy%20%7C%20scipy%20%7C%20matplotlib%20%7C%20pandas-9C27B0">
  <img alt="Base paper" src="https://img.shields.io/badge/paper%201-ECC%202013%20Fresk-00599C">
  <img alt="Path paper" src="https://img.shields.io/badge/paper%202-ICMSAO%202017%20Choutri-E65100">
</p>

---

<p align="center">
  <a href="https://quadrotor-quaternion-sim.vercel.app"><b> Live: four-simulator results & benchmark</b></a>
  &nbsp;·&nbsp;
  <a href="https://quadrotor-quaternion-sim.vercel.app/simulator.html"><b> Interactive simulator — fly the paper's controller in your browser</b></a>
  &nbsp;·&nbsp;
  <a href="RECORDINGS.md"><b> Recording gallery — every test & path-planning flight, animated</b></a>
</p>

## What this repository is

Two IEEE papers, one airframe, one repo — each implemented from its equations and
cross-validated on multiple physics engines:

1. **Paper 1 — attitude (the base paper).** Fresk & Nikolakopoulos, *Full Quaternion
   Based Attitude Control for a Quadrotor* (ECC 2013): the derivative-free nonlinear
   quaternion control law τ = −P_q·q_vec − P_ω·ω (P_q=20, P_ω=4, ±4 N·m). Reproduced
   bit-exactly ([`quat_sitl/`](quat_sitl/)), extended to 6-DOF ([`pysitl/`](pysitl/)),
   and deployed **unmodified** to four simulators ([`sim/`](sim/)): MuJoCo,
   gym-pybullet-drones, Gazebo, and real ArduPilot SITL firmware.
2. **Paper 2 — path planning.** Choutri, Lagha, Dala & Lipatov, *Quadrotors Trajectory
   Tracking using a Differential Flatness–Quaternion based Approach* (IEEE 2017):
   flat outputs σ=(x,y,z,ψ) → total thrust T_d + desired quaternion q_d → double-loop
   LQR tracking a circular path. Implemented equation-by-equation
   ([`flatness/`](flatness/)) on the same airframe, on the paper's ideal model **and**
   MuJoCo contact physics, to **2.7–2.8 cm radial RMS** on a 1 m-radius circle.

Everything below is measured, not claimed: each result regenerates from one command,
and every recording in [`RECORDINGS.md`](RECORDINGS.md) *is* the measured run.

## Results at a glance

Every number below is reproduced by this repository (commands in
[Running everything](#running-everything); full tables, plots and per-axis RMS in the
linked sections):

| Paper | Experiment | Measured result | Details |
|---|---|---|---|
| 1 — attitude | Sine 0.5 rad @ 1 rad/s, paper noise ±0.1 | RMS: ArduPilot (real firmware) **14.7° ± 0.2**, Gazebo **17.3°**, gym-pybullet-drones 17.52°, MuJoCo 17.61° — cross-engine step agreement 0.03° | [Part II](#part-ii--four-simulator-deployment-same-frozen-controller) · [`results/comparison.md`](results/comparison.md) |
| 1 — attitude | 360° flip (0→2π ramp) | completes on ArduPilot (φ settles **2.89 s**) and MuJoCo (**2.93 s**), 3/3 seeds each | [Part II](#part-ii--four-simulator-deployment-same-frozen-controller) · [`RECORDINGS.md`](RECORDINGS.md) |
| 2 — study case | Circle, radius 1 m @ 1 m altitude | radial RMS **2.7 cm** (ideal Eq. 8) · **2.8 cm** (MuJoCo contact) · **3.7 cm** (+noise ±0.1) — ≈97% accuracy, zero rotor saturation | [Part III results](#results--the-papers-study-case-circle-radius-1-m-at-1-m-altitude) |
| 2 — extensions | Figure-8 (A=1.5/B=1) · vertical orbit | 3D RMS **4.7 cm** (identical on MuJoCo) · **3.9 cm** | [Extensions](#extensions-delivered--phase-a-trajectory-library--aggressive-paths) |
| 2 — extensions | 5-waypoint tour: min-snap vs min-jerk | **6.1 cm vs 8.1 cm** 3D RMS — min-snap 25% better, exactly as flatness predicts | [Extensions](#extensions-delivered--phase-a-trajectory-library--aggressive-paths) |

## Team - Group 12

| Member | Roll No. | Email |
|---|---|---|
| B Sainath | CB.SC.U4AIE24309 | cb.sc.u4aie24309@cb.students.amrita.edu |
| Manohar Paturi | CB.SC.U4AIE24339 | cb.sc.u4aie24339@cb.students.amrita.edu |
| K Pushpak | CB.SC.U4AIE24328 | cb.sc.u4aie24328@cb.students.amrita.edu |
| B Sai Krishna | CB.SC.U4AIE24308 | cb.sc.u4aie24308@cb.students.amrita.edu |
| Vishal | CB.SC.U4AIE24363 | cb.sc.u4aie24363@cb.students.amrita.edu |

B.Tech (AI Engineering), School of Artificial Intelligence,
Amrita School of Engineering, Coimbatore, Amrita Vishwa Vidyapeetham.

---

# Part I — The base paper (Fresk & Nikolakopoulos 2013)

## How it works

The closed loop never leaves quaternion space — Euler angles appear only in plotting and
logging (eq. 16):

```text
        q_ref
          │
          ▼
   q_err = q_ref ⊗ q_m*            -- eq. (19)
          │  take vector part
          ▼
   Axis_err                         -- eq. (20)
          ▼
   τ = −P_q·Axis_err − P_ω·ω_m      -- eq. (21), saturated at ±4 N·m
          ▼
   Quadrotor attitude plant         -- eqs. (17)–(18)
          │
          ▼
   q_m , ω_m  (uniform ±0.1 noise) ──► fed back to q_err
```

The law is derivative-free and needs no integral term: torque → ω̇ → attitude is
a double integrator,so steady-state error decays to zero on its own. Where the paper's
sign on eq. (18) differs from the textbook convention, it is **preserved literally**, not
silently corrected (see Fidelity notes).

## Equation → function map (paper 1)

| Paper eq. | Description | Location |
|---|---|---|
| (1)–(2) | Quaternion product, `Q`/`Q̄` matrices | `quaternion.mul`, `quaternion.Q`, `quaternion.Q_bar` |
| (3)–(5) | Norm, conjugate, inverse | `quaternion.norm`, `.conj`, `.inv` |
| (6)–(7) | q̇, fixed-frame ω / body-frame ω′ | `quaternion.qdot_fixed`, `.qdot_body` |
| (8) | Rotation q ⊗ v ⊗ q* | `quaternion.rotate` |
| (9)–(13) | DCM columns and transpose | `quaternion.to_dcm` (+ `.T`) |
| (14)–(16) | Axis-angle / Euler ↔ quaternion | `quaternion.from_axis_angle`, `.from_euler`, `.to_euler` |
| (17)–(18) | Rigid-body plant | `dynamics.state_derivative`, `dynamics.InertiaParams` |
| (19)–(21) | Nonlinear P² controller | `controller.NonlinearP2Controller.compute_torque` |

## Results vs. paper

> **Runnable report (MATLAB Live Script):** [`matlab/project_report.mlx`](matlab/project_report.mlx)
> — reproduces both papers end-to-end (every figure and table below regenerates live
> from its equations). PDF export: [`matlab/published/project_report_final.pdf`](matlab/published/project_report_final.pdf).

Every numeric parameter the paper states is reproduced exactly:

| Quantity | Paper | Implemented | Match |
|---|---|---|---|
| $I_{xx}=I_{yy}$, $I_{zz}$ (kg·m²) | $6.5\times10^{-4}$, $1.2\times10^{-3}$ | same | Exact |
| $P_q$ / $P_\omega$ | 20 / 4 | 20 / 4 | Exact |
| Torque saturation | ±4 N·m | ±4 N·m | Exact |
| Sine amplitude / frequency | 0.5 rad, 1 rad/s | same | Exact |
| Flip range | 0 → 2π rad | same | Exact |
| Noise amplitude | 0.1 | 0.1 | Exact |

**Step** — 1 rad steps on $\phi,\theta,\psi$, staggered at $t=1,5,9$ s over 15 s.
Essentially no overshoot, ≈ 2 s settling per axis (paper Fig. 3–4).

<p align="center">
  <img src="docs/figures/step_attitude.png" width="600" alt="Step response: reference (dashed) vs. output (solid)">
</p>
<p align="center">
  <img src="docs/figures/step_torque.png" width="600" alt="Step response torque, briefly saturating at each step onset">
</p>

**Sine** — 0.5 rad, 1 rad/s on all axes, phase-shifted so torques stay out of phase.
Measured lag ≈ 0.4–0.5 s vs. the paper's "about 0.5 second"; torque stays linear
throughout steady state (paper Fig. 5–6). *(One logged sample at $t=0.001$ s saturates —
a startup transient, since the phase-shifted reference starts at 0.5 rad, not a tracking
defect).*

<p align="center">
  <img src="docs/figures/sine_attitude.png" width="600" alt="Sine tracking: reference vs. output">
</p>

**360° flip** — shortest-path correction deliberately disabled; reference ramps 0 → 2π rad
over 2 s. Raw $q_0, q_1$ stay smooth through the full rotation — **no gimbal-lock
artifact**, the paper's central claim (paper Fig. 7–8).

<p align="center">
  <img src="docs/figures/flip_attitude.png" width="600" alt="360° flip: wrapped Euler roll alongside smooth raw q0, q1">
</p>

## Key finding 1: the control-rate stability bound

The paper specifies no digital control rate. A plausible 200 Hz control / 1 kHz plant
design **never converges** — for any step size. Linearizing the closed loop about identity
attitude:

$$I\,\ddot e + P_\omega\,\dot e + \tfrac{P_q}{2}\,e = 0
\;\Rightarrow\;
\lambda_{1,2} = \frac{-P_\omega \pm \sqrt{P_\omega^2 - 2IP_q}}{2I}
\approx -2.5,\ -6150\ \mathrm{rad/s}$$

The continuous loop is heavily overdamped ($\zeta\approx25$) and fine — the instability is
purely a **sampling artifact of the fast pole**. Exact zero-order-hold spectral radius
$\rho(A_{cl})$:

| Control rate | 200 Hz | 1 kHz | 10 kHz | 100 kHz |
|---|---|---|---|---|
| $\rho(A_{cl})$ | 29.9 | 5.16 | 0.9997 | 0.99997 |
| Stable? | No | No | Marginal | Yes |

Minimum safe rate, derived (not tuned): $f_{control} \ge P_\omega/(m \cdot I_{\min}) \approx$
**12.3 kHz** at margin $m=0.5$. `dynamics.stable_control_rate_hz` computes it from the
configured gains/inertia; the simulator adopts it by default; the 6-DOF scheduler refuses
to run below it. Full derivation: [`REPORT.md` §2.5](REPORT.md).

## Fidelity notes (paper 1)

- **Eq. (18)'s minus sign is preserved deliberately** — opposite to eq. (7). Verified
  against the published PDF, kept literal in `dynamics.py`, and guarded by a test
  ([`REPORT.md` §4.4](REPORT.md)).
- **Every paper ambiguity is resolved with a documented default**, never a silent guess:
  uniform noise $\mathcal{U}(-0.1,0.1)$; step stagger x@1 s / y@5 s / z@9 s; sine phases
  $\phi_y{=}0$, $\phi_z{=}\pi/2$; flip ramp 2 s; DCM in eq. (12) column form, pinned by
  test ([`REPORT.md` §3.3](REPORT.md)).
- **Control-signal → torque is the identity**, the paper's own stated simplification.

## The 6-DOF extension (`pysitl`)

The paper never needs mass or geometry, so both are *derived* from its inertia:
$I_{zz}/I_{xx}=1.846 \approx 2$ (planar-X point-mass quad) ⇒ $L=0.15$ m, ≈ 12.2 g rotors,
≈ 200 g total, ≈ 1.95 N hover thrust. The **unmodified** controller then flies this vehicle
through a PX4-shaped stack — uORB-style pub/sub bus, deterministic lockstep scheduler
(16 kHz physics/attitude/mixer, 1 kHz sensors, 250 Hz altitude, 50 Hz commander, 200 Hz
logger), arming + failsafes, thrust-preserving mixer desaturation, and a browser ground
station. Under full paper noise, a 0.3 rad roll command converges to 0.309 rad with
altitude held to centimeters.

Design decisions and derivations: [`pysitl/README.md`](pysitl/README.md) ·
dashboard manual: [`pysitl/UI_GUIDE.md`](pysitl/UI_GUIDE.md).

---

# Part II — Four-simulator deployment (same frozen controller)

> Fresk & Nikolakopoulos's quaternion attitude law, gains **untouched**
> ($P_q = 20$, $P_\omega = 4$, $\pm 4\,\mathrm{N\cdot m}$), flying the same derived
> paper vehicle ($0.2$ kg, $I_{xx} = I_{yy} = 6.5\times10^{-4}$,
> $I_{zz} = 1.2\times10^{-3}\ \mathrm{kg\cdot m^2}$) on **gym-pybullet-drones**, **MuJoCo**, **Gazebo**, and
> **ArduPilot SITL** through one shared bridge ([`sim/common/`](sim/common/)).
> Machine-generated benchmark: `python -m sim.compare` → `results/comparison.md`.

| Stack | Status | One-line result |
|---|---|---|
| **MuJoCo** (20 kHz, contact physics) | ✅ validated natively | all 3 paper scenarios; flip completes **fully airborne**, 3 seeds, φ settles 2.87–2.97 s |
| **gym-pybullet-drones** (24 kHz, custom paper-vehicle URDF) | ✅ validated natively | step & sine within **0.2° RMS of MuJoCo** — the loop belongs to the controller, not the engine |
| **ArduPilot SITL** (real firmware, compiled from source) | ✅ **flown under WSL2** | all 3 scenarios × 3 seeds; **sine 14.7° ± 0.2 — the best of all four stacks**; flip settles in 2.89 s |
| **Gazebo** (gz-harmonic 8.15 + ardupilot_gazebo) | ✅ **flown under WSL2** | all 3 scenarios × 3 seeds; **sine 17.3° ± 0.0 — within 0.3° of the native stacks**; step/flip divergence documented (tilt-limit regime) |

**Headline:** with identical gains, references, and the paper's ±0.1 noise, sine tracking
agrees across all four stacks — ArduPilot (real firmware) **14.7° ± 0.2**, Gazebo
**17.3° ± 0.0**, gym-pybullet-drones 17.52°, MuJoCo 17.61°; step agrees
17.71°/17.68° between the native stacks (**0.03° cross-engine agreement**). The 360° flip
completes on real ArduPilot firmware (φ settles 2.89 s) and MuJoCo (2.93 s) — while
PyBullet/Gazebo sit in the engine-dependent knife-edge regime (see finding 2 below).

The full 30-row, 3-seed machine-generated table lives in
[`results/comparison.md`](results/comparison.md) (regenerated by `python -m sim.compare`).

<p align="center">
  <img src="results/recordings/mujoco_flip_RENDER.gif" width="420" alt="In-engine MuJoCo render: the 360° flip benchmark — color-coded drone chasing the RGB reference triad with live quaternion error">
</p>

## Key finding 2: the flip's knife-edge (Eq. 33 pursuit equilibrium)

With shortest-path disabled, the P² pursuit equilibrium $\omega = 5\sin(e/2)$ means a
2 s $0 \to 2\pi$ ramp (rate $\pi\ \mathrm{rad/s}$) is trackable only at lag
$e^{*} = 2\arcsin(\pi/5) \approx 1.36\ \mathrm{rad}$ — the
vehicle crosses 2π *just after* the reference. Whether it completes is structurally
marginal: MuJoCo and real ArduPilot firmware complete deterministically (3/3 seeds);
PyBullet enters a limit cycle near φ ≈ 2.1 rad and unwinds; Gazebo behaves like PyBullet.
Measured and documented (finding 4 in [`sim/README.md`](sim/README.md)) — an engine-
dependent property of the sampled fast pole, not a tuning matter.

---

# Part III — Path planning: differential flatness + quaternion LQR

> **Paper 2:** K. Choutri, M. Lagha, L. Dala, M. Lipatov, *"Quadrotors Trajectory
> Tracking using a Differential Flatness–Quaternion based Approach,"* IEEE 2017 —
> implemented equation-by-equation in [`flatness/`](flatness/) on the same 0.2 kg
> airframe, branch `sainath/flatness-path-planning`.

## What the paper does

The base paper controls **attitude only** — point the body, and the vehicle goes wherever
the tilt pushes it (the step/sine benchmarks drift kilometers by design). Paper 2 adds the
missing layers: choose where the vehicle should be (flat outputs), use **differential
flatness** to convert the desired path into the thrust and attitude that produce it, and
close the loop with a **double-loop LQR**:

```text
   path  σ(t) = (x, y, z, ψ)  +  derivatives            -- Eq. (13)
      │
      ▼  OUTER LOOP — position LQR (Eq. 18):  a_cmd = a_ref − K·[p−p_ref, v−v_ref]
      │                                     (actuator-aware clamp, paper §V limits)
      ▼  FLATNESS MAP (Eqs. 15, 19–21):
      │      thrust vector f = m·(a_cmd + g·ẑ)          -- Eq. (15)
      │      T_d = ‖f‖            (altitude output)
      │      q_pd = tilt taking ẑ onto f/‖f‖ (zero yaw content, Eq. 19)
      │      q_zd = yaw quaternion from ψ_d             (Eq. 20)
      │      q_d  = q_pd ⊗ q_zd                         (Eq. 21)
      ▼  INNER LOOP — attitude LQR (Eq. 17):  τ = −K·[q_vec_err, ω]  (±4 N·m)
      │
      ▼  MIXER (Eq. 9): (T_d, τ) → 4 rotor speeds, priority desaturation
      ▼
   the plant — the paper's ideal Eq. (8) model  AND  MuJoCo contact physics
```

Controller runs at the paper's stated **200 Hz** (zero-order hold); the ideal model
integrates at 1 kHz (RK4), MuJoCo at 20 kHz.

## Equation → function map (paper 2)

> **Complete derivations (MATLAB Live Script):**
> [`matlab/quadrotor_math_derivations.mlx`](matlab/quadrotor_math_derivations.mlx)
> — every equation of this paper derived step by step (quaternion algebra through the
> Riccati equation, with the study-case and extension results). Verified block-by-block
> against the source document: [`matlab/MLX_VERIFICATION.md`](matlab/MLX_VERIFICATION.md).
> The runnable end-to-end report for both papers is
> [`matlab/project_report.mlx`](matlab/project_report.mlx).

| Paper eq. | Description | Location |
|---|---|---|
| (1)–(7) | Quaternion algebra (product, conjugate, norm, inverse, rotation matrix, axis-angle, q̇) | reused from `quat_sitl.quaternion` (paper 1's verified implementation) |
| (8) | Newton–Euler quaternion model: ṗ=v, v̇ = R(q)[0,0,T/m] − g, q̇ = ½q⊗ω, Jω̇ = −ω×Jω + τ | `flatness/model.py::QuadrotorModel.deriv` (RK4, z-up) |
| (9) | Rotor mixing: T = Σ b·ωᵢ², torques from arm + motor reactions | `flatness/model.py::_mixer_matrix`, `.mix` (priority desaturation), `.wrench` |
| (10) | Hover linearization | implicit in `flatness/lqr.py::AttitudeLQR` (A = [[0, I/2],[0,0]]) |
| (11)–(12) | Flatness definition (system flat in output z(t), derivatives up to β) | the design premise of `flatness/flatness.py` |
| (13) | Flat outputs σ = (x, y, z, ψ) | `flatness/trajectories.py::FlatRef` (p, v, a, ψ, ψ̇) |
| (14) | Trivial position/velocity/acceleration map | `FlatRef` fields |
| (15) | States & input parameterized by σ and derivatives (thrust-vector row) | `flatness/flatness.py::flatness_reference` |
| (16) | State-space form ẋ = Ax + Bu | `flatness/lqr.py` (A, B of both loops) |
| (17) | Attitude model: [q_vec, ω], A = [[0, I/2],[0,0]], B = [0; J⁻¹] | `flatness/lqr.py::AttitudeLQR.__post_init__` |
| (18) | Position model: double integrator + input mapping | `flatness/lqr.py::PositionLQR.__post_init__` (per-axis) |
| (19) | Position quaternion q_pd ("4th element zero" — scalar-first: zero z component) | `flatness_reference`: shortest-arc tilt ẑ→thrust-dir; z component measured −5.6e-17 |
| (20) | Yaw quaternion q_zd from ψ_d | `[cos(ψ/2), 0, 0, sin(ψ/2)]` in `flatness_reference` |
| (21) | q_d = q_pd ⊗ q_zd | `quat.mul(q_pd, q_zd)` — literal composition, body-z along thrust exact to 1e-16 |
| (22) | LQR quadratic cost J = ∫ (xᵀQx + uᵀRu) dt | `lqr_gain` (Q/R as documented assumptions — paper gives no numbers) |
| (23) | Riccati equation, optimal U = −Lx | `lqr_gain` via `scipy.linalg.solve_continuous_are` |
| §V | Study case: circle at 1 m altitude from (−1,0,0), 200 Hz, actuator limits | `flatness/run_paper.py`, `flatness/run_mujoco.py` |

## Complete mathematical derivations — MATLAB Live Script (Paper 2)

This section is a full Markdown transcription of [`matlab/quadrotor_math_derivations.mlx`](matlab/quadrotor_math_derivations.mlx), integrated into the project README. It retains **all 23 numbered paper equations**, their intermediary derivation steps, symbol definitions, controller design, two worked LQR examples, measured project results, paper-typo notes, revision questions, and the Live Script's own references. Equations are rendered using GitHub-compatible LaTeX; matrix notation has been typeset as matrices where the Live Script used semicolon-separated rows.

**Source distinction:** References to “the paper” in this section concern Choutri et al. (2017), unless explicitly indicated otherwise. Text labeled **extra** reflects supplementary explanation or project-specific analysis, not a quotation or claim by the original paper. Printed-paper corrections are retained as the Live Script labels them; they should be checked against the typeset papers before citing them as confirmed errata. The study-case numerical results are project measurements, not values published in the paper.

### Quadrotor Trajectory Tracking — Full Math Derivations

Choutri, Lagha, Dala, Lipatov (2017), ICMSAO. Every equation, where it comes from, and what each symbol means.

### How to Use This File
- Every equation keeps the paper's equation number, e.g. Eq. (6), so you can say "this is Eq. (6) of the paper" in front of the professor.
- Each equation has the same pattern: what it says → where it comes from (derivation, step by step) → meaning of every symbol → where it is used next.
- Paper typo: marks a place where the printed paper has a typo or inconsistency. The corrected form is derived here. See Section 7 for the full list.
- Things marked (extra, not in the paper) are my added reasoning or examples, to help you understand. Do not say they are from the paper.

### 0. Big Picture — How the Equations Connect

```text
Quaternion math (Eq. 1-7)    : q, q*, q (x) r, R_q, q_dot = 1/2 q (x) w
Dynamics (Eq. 8-10)          : full model (8), motors to T and torques (9), linear model at hover (10)
Flatness (Eq. 11-15)         : outputs x, y, z, psi give the desired angles (15)
State-space (Eq. 16-18)      : A and B matrices from the linear model
Desired attitude (Eq. 19-21) : q_pd and q_zd combine into q_d
LQR (Eq. 22-23)              : cost and Riccati equation give u = -Lx
```

| Eq. | What it is | Where it comes from |
| --- | --- | --- |
| (1) | Quaternion definition | Definition |
| (2) | Quaternion product | Rules $i^2=j^2=k^2=ijk=-1$ |
| (3) | Conjugate, norm, inverse | Eq. (2) applied to $q\otimes q^*$ |
| (4) | Rotation matrix $R_q$ | Eq. (2) and (3) applied to $q\otimes v\otimes q^*$ |
| (5) | Axis–angle form of $q$ | Eq. (4) compared with Rodrigues' rotation formula |
| (6) | $\dot q=\frac12 q\otimes\omega$ | Small rotation in time $dt$ |
| (7) | Euler angles to/from quaternion | Multiply three axis quaternions; compare with Eq. (4) |
| (8) | Full nonlinear dynamics | Newton's law + Euler's rotation equation |
| (9) | Motor speeds → thrust & torques | Rotor physics (force × arm) |
| (10) | Linear model at hover | Eq. (8) with small-angle approximation |
| (11),(12) | Definition of differential flatness | Definition |
| (13),(14) | The four flat outputs | Definition + trivial derivatives |
| (15) | Desired pitch/roll from acceleration | Translation part of Eq. (8) written with Euler angles |
| (16) | General linear state-space | Definition |
| (17) | Attitude state-space | Eq. (10) rotation part |
| (18) | Position state-space | Eq. (10) translation part |
| (19)–(21) | Desired quaternion $q_d$ | Eq. (5) and (2) |
| (22) | LQR cost function | Definition (design choice) |
| (23) | Riccati equation | Minimising Eq. (22) subject to Eq. (16) |

#### 0.1 Symbols Used in the Whole Paper

| Symbol | Meaning |
| --- | --- |
| $p\in\mathbb{R}^3$ | Position of the drone in the inertial (ground) frame: $[x,y,z]^T$ |
| $\dot p, \ddot p$ | Velocity, acceleration |
| $q\in\mathbb{H}$ | Unit quaternion = orientation of the drone body w.r.t. the ground frame |
| $q_0$ | Scalar (real) part of $q$ |
| $\bar{q}=[q_1,q_2,q_3]^T$ | Vector (imaginary) part of $q$ |
| $\omega\in\mathbb{R}^3$ | Angular velocity in the body frame |
| $m$ | Mass of the drone |
| $J$ | Inertia matrix (3×3) in the body frame |
| $g$ | Gravity, $9.81 \mathrm{m/s}^2$; $\bar{g}=[0,0,-g]^T$ |
| $T$ | Total thrust (force along the body $z$-axis) |
| $\tau=[\tau_x,\tau_y,\tau_z]^T$ | Torques about the body axes (roll, pitch, yaw) |
| $\phi,\theta,\psi$ | Roll, pitch, yaw (Euler angles) |
| $\omega_i$ | Speed of rotor $i$ ($i=1\ldots4$) |
| $k_i$ | Thrust coefficient of rotor $i$ |
| $l$ | Distance from centre of mass to rotor axis (arm length) |
| ${X}=[X_1,X_2,X_3,X_4]^T$ | Flat output $=[x,y,z,\psi]^T$ |
| $A,B,C$ | State-space matrices |
| $Q,R,P,L$ | LQR weights, Riccati solution, feedback gain |

NOTE: The paper sometimes numbers quaternion elements 1–4 (e.g. $q_{4zd}$) and sometimes 0–3. In this file I always use $q_0,q_1,q_2,q_3$, with $q_3$ = the $z$-component.

### 1. Quaternion Mathematics (Eq. 1 – 7)

Why quaternions at all? Euler angles $(\phi,\theta,\psi)$ are easy to imagine but break at $\theta=\pm90^\circ$ (gimbal lock: two axes line up and one degree of freedom is lost). A quaternion uses 4 numbers with 1 constraint ($\| q\|=1$), which gives 3 degrees of freedom, with no singularity.

#### Eq. (1) — Quaternion definition

$$
q = q_0 + \bar{q} = q_0 + \begin{bmatrix}q_1 \\ q_2 \\ q_3\end{bmatrix},\;\;\;\; q\in\mathbb{H},\;\bar{q}\in\mathbb{R}^3,\;q_0\in\mathbb{R} \;\;\;\;\mathrm{(1)}
$$

**Meaning of each part**
- $q_0$: scalar part. It is linked to how much we rotate.
- $\bar{q}$: vector part. It is linked to which axis we rotate about.
- Written fully: $q=q_0+q_1 i+q_2 j+q_3 k$, where $i,j,k$ are three different "imaginary units".
- Identity quaternion $[1,0,0,0]^T$ means no rotation (this is the hover value in the paper).

#### Eq. (2) — Quaternion product

$$
q\otimes r = (q_0r_0-\bar{q}\cdot\bar{r})+(r_0\bar{q}+q_0\bar{r}+\bar{q}\times\bar{r}) \;\;\;\;\mathrm{(2)}
$$

**Where it comes from (derivation)**

Hamilton's rules for the imaginary units:

$$
i^2=j^2=k^2=ijk=-1\;\Longrightarrow\;ij=k,\;jk=i,\;ki=j,\;\; ji=-k,\;kj=-i,\;ik=-j
$$

Step 1. Multiply term by term: $(q_0+q_1i+q_2j+q_3k)(r_0+r_1i+r_2j+r_3k)$.

Step 2. Scalar part. Only $q_0r_0$ and the products $i\cdot i, j\cdot j, k\cdot k$ (each $=-1$) give scalars:

$$
q_0r_0-q_1r_1-q_2r_2-q_3r_3=q_0r_0-\bar{q}\cdot\bar{r}
$$

Step 3. $i$-part. Collect all terms ending in $i$: $q_0r_1+q_1r_0+q_2r_3 (jk=i)-q_3r_2 (kj=-i)$

$$
= q_0r_1+r_0q_1+(q_2r_3-q_3r_2)
$$

The bracket is exactly the $x$-component of the cross product $\bar{q}\times\bar{r}$.

Step 4. The $j$ and $k$ parts work the same way, so the full vector part is $r_0\bar{q}+q_0\bar{r}+\bar{q}\times\bar{r}$.

**Meaning of each part**
- $-\bar{q}\cdot\bar{r}$ (dot product): how much the two axes agree.
- $r_0\bar{q}+q_0\bar{r}$: each scalar scales the other's axis.
- $\bar{q}\times\bar{r}$ (cross product): the "twisting" term. It changes sign if you swap the order, so $q\otimes r\neq r\otimes q$.
- Example: $i\otimes j=k$ but $j\otimes i=-k$. Rotation order matters.

#### Eq. (3) — Conjugate, norm, inverse

$$
q^*=q_0-\bar{q},\;\;\;\; \| q\|=\sqrt{q\otimes q^*}=\sqrt{q_0^2+q_1^2+q_2^2+q_3^2},\;\;\;\; q^{-1}=\frac{q^*}{\| q\|^2} \;\;\;\;\mathrm{(3)}
$$

**Derivation**

Step 1. Put $r=q^*=(q_0,-\bar{q})$ into Eq. (2).

Step 2. Scalar part: $q_0q_0-\bar{q}\cdot(-\bar{q})=q_0^2+\|\bar{q}\|^2=q_0^2+q_1^2+q_2^2+q_3^2$.

Step 3. Vector part: $q_0(-\bar{q})+q_0\bar{q}+\bar{q}\times(-\bar{q})=\mathbf{0}$ (because $\bar{q}\times\bar{q}=0$).

Step 4. So $q\otimes q^*=\| q\|^2$, a pure real number. Dividing: $q\otimes\frac{q^*}{\| q\|^2}=1$, so $q^{-1}=\frac{q^*}{\| q\|^2}$.

Step 5. Unit quaternion ($\| q\|=1$): $q^{-1}=q^*$. This is why the paper only uses unit quaternions: the inverse is free (just flip the sign of the vector part).

Paper typo: The paper prints the inverse as $q^*/\| q\|$ and the norm with indices $q_2^2+q_3^2+q_4^2$. The correct forms are above.

Meaning: conjugate = "rotate backwards"; norm = "length" of the quaternion; inverse = the quaternion that undoes $q$.

#### Eq. (4) — Rotation matrix R_q

$$
R_q=(q_0I+q_{1_{3\times}})^2+\bar{q}\bar{q}^T =(q_0^2-\bar{q}^T\bar{q})I+2q_0q_{1_{3\times}}+2\bar{q}\bar{q}^T \;\;\;\;\mathrm{(4a)}
$$

$$
R_q=\begin{bmatrix}q_0^2+q_1^2-q_2^2-q_3^2 & 2(q_1q_2-q_3q_0) & 2(q_1q_3+q_2q_0) \\ 2(q_1q_2+q_3q_0) & q_0^2-q_1^2+q_2^2-q_3^2 & 2(q_2q_3-q_1q_0) \\ 2(q_1q_3-q_2q_0) & 2(q_2q_3+q_1q_0) & q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix} \;\;\;\;\mathrm{(4b)}
$$

Where it comes from. A vector $v$ is rotated by $v'=q\otimes v\otimes q^*$, with $v$ treated as the pure quaternion $(0,v)$.

Step 1. First product, using Eq. (2) with $r=(0,v)$:

$$
q\otimes v=(-\bar{q}\cdot v,\;\;q_0v+\bar{q}\times v)
$$

Step 2. Multiply by $q^*=(q_0,-\bar{q})$ on the right. The scalar part cancels to $0$ (so $v'$ is again a pure vector). The vector part is:

$$
q_0(q_0v+\bar{q}\times v)+(\bar{q}\cdot v)\bar{q}-(q_0v+\bar{q}\times v)\times\bar{q}
$$

Step 3. Expand the last term with $v\times\bar{q}=-\bar{q}\times v$ and the identity $(\bar{q}\times v)\times\bar{q}=v (\bar{q}\cdot\bar{q})-\bar{q} (v\cdot\bar{q})$:

$$
v'=(q_0^2-\bar{q}\cdot\bar{q})\;v+2(\bar{q}\cdot v)\;\bar{q}+2q_0\;(\bar{q}\times v) \;\;\;\;\mathrm{(4c)}
$$

Step 4. Write each piece as a matrix times $v$:
- $\bar{q}\times v=q_{1_{3\times}}v$, where $q_{1_{3\times}}$ is the skew-symmetric matrix of $\bar{q}$ (written out below).
- $(\bar{q}\cdot v)\bar{q}=\bar{q}\bar{q}^Tv$.

$$
q_{1_{3\times}}=\begin{bmatrix}0 & -q_3 & q_2 \\ q_3 & 0 & -q_1 \\ -q_2 & q_1 & 0\end{bmatrix}
$$

So $v'=R_qv$ with $R_q=(q_0^2-\bar{q}^T\bar{q})I+2q_0q_{1_{3\times}}+2\bar{q}\bar{q}^T$. This is the second form in Eq. (4a).

Step 5. Check that the first form equals the second. $(q_0I+S)^2=q_0^2I+2q_0S+S^2$ with $S=q_{1_{3\times}}$, and $S^2=\bar{q}\bar{q}^T-(\bar{q}^T\bar{q})I$. Therefore $(q_0I+S)^2+\bar{q}\bar{q}^T=(q_0^2-\bar{q}^T\bar{q})I+2q_0S+2\bar{q}\bar{q}^T$.

Step 6. Matrix entries (Eq. 4b). Example: entry (1,1) $=q_0^2-(q_1^2+q_2^2+q_3^2)+2q_1^2=q_0^2+q_1^2-q_2^2-q_3^2$. Entry (1,2) $=2q_1q_2+2q_0S_{12}=2q_1q_2-2q_0q_3$. All others follow the same way.

**Meaning of each part**
- $I$: 3×3 identity. $S$: skew matrix, which performs the cross product.
- $\bar{q}\bar{q}^T$: 3×3 matrix = (column)×(row).
- $R_q$ is orthogonal: $R_q^{-1}=R_q^T$ and $\mathrm{det} R_q=1$ (it is a pure rotation, with no stretching). This is the statement under Eq. (4) in the paper.
- Third column of $R_q$ = where the body $z$-axis (thrust direction) points in the ground frame:

$$
R_q\;e_3=\begin{bmatrix}2(q_1q_3+q_2q_0) \\ 2(q_2q_3-q_1q_0) \\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}
$$

This column is used in Eq. (10).

#### Eq. (5) — Axis–angle form

$$
q=\cos\frac{\alpha}{2}+u\sin\frac{\alpha}{2} \;\;\;\;\mathrm{(5)}
$$

Meaning: $u$ is the unit rotation axis ($\| u\|=1$) and $\alpha$ is the rotation angle.

Derivation (it must rotate vectors the right way)

Step 1. Put $q_0=\cos\frac\alpha2$ and $\bar{q}=u\sin\frac\alpha2$ into Eq. (4c).

Step 2. Use $c=\cos\frac\alpha2, s=\sin\frac\alpha2$:

$$
v'=(c^2-s^2)v+2s^2(u\cdot v)u+2cs\;(u\times v)
$$

Step 3. Double-angle identities: $c^2-s^2=\cos\alpha$, $2cs=\sin\alpha$, $2s^2=1-\cos\alpha$:

$$
v'=\cos\alpha\;v+\sin\alpha\;(u\times v)+(1-\cos\alpha)(u\cdot v)\;u
$$

Step 4. This is exactly Rodrigues' rotation formula: rotate $v$ by angle $\alpha$ about axis $u$.

Why half-angles? Because $q$ appears twice in $q\otimes v\otimes q^*$, so each copy carries half of the rotation.

Where used: Eq. (20), the yaw quaternion (axis $u=[0,0,1]^T$, angle $\alpha=\psi_d$).

#### Eq. (6) — Quaternion kinematics

$$
\dot q=\frac12\;q\otimes\omega \;\;\;\;\mathrm{(6)}
$$

Meaning: how the orientation changes when the body spins with angular velocity $\omega$ (body frame). $\omega$ is used as the pure quaternion $(0,\omega)$.

Derivation (small rotation)

Step 1. In a tiny time $dt$, the body rotates by the small angle $\alpha=\|\omega\| dt$ about the axis $u=\omega/\|\omega\|$.

Step 2. By Eq. (5), this small rotation is

$$
\delta q=\begin{bmatrix}\cos\frac{\|\omega\| dt}{2} & u\sin\frac{\|\omega\| dt}{2}\end{bmatrix}\approx\begin{bmatrix}1 & \frac12\omega\;dt\end{bmatrix}
$$

(because $\cos(\mathrm{small})\approx1$ and $\sin(\mathrm{small})\approx$ small).

Step 3. The new orientation: $q(t+dt)=q(t)\otimes\delta q$ (body-frame rotation multiplies on the right).

Step 4. Difference quotient:

$$
\dot q=\lim_{dt\rightarrow0}\frac{q\otimes\delta q-q}{dt}=q\otimes\frac{(0,\;\frac12\omega\;dt)}{dt}=\frac12\;q\otimes\omega
$$

Where used: third row of Eq. (8). At hover, $q\approx[1,0,0,0]$, so $\dot{\bar{q}}\approx\frac12\omega$, which is the "0.5" in Eq. (17).

#### Eq. (7) — Euler angles to/from quaternion

Euler → quaternion (yaw–pitch–roll, order Z–Y–X, with $c_x=\cos\frac x2, s_x=\sin\frac x2$):

$$
q=\begin{bmatrix}c_\phi c_\theta c_\psi+s_\phi s_\theta s_\psi \\ s_\phi c_\theta c_\psi-c_\phi s_\theta s_\psi \\ c_\phi s_\theta c_\psi+s_\phi c_\theta s_\psi \\ c_\phi c_\theta s_\psi-s_\phi s_\theta c_\psi\end{bmatrix} \;\;\;\;\mathrm{(7a)}
$$

**Derivation**

Step 1. Each single-axis rotation is a quaternion by Eq. (5):
- Roll: $q_\phi=[c_\phi, s_\phi, 0, 0]$ (axis $x$)
- Pitch: $q_\theta=[c_\theta, 0, s_\theta, 0]$ (axis $y$)
- Yaw: $q_\psi=[c_\psi, 0, 0, s_\psi]$ (axis $z$)

Step 2. Total rotation: $q=q_\psi\otimes q_\theta\otimes q_\phi$.

Step 3. First $q_\psi\otimes q_\theta$ with Eq. (2): scalar $=c_\psi c_\theta$; vector $=c_\theta(0,0,s_\psi)+c_\psi(0,s_\theta,0)+(0,0,s_\psi)\times(0,s_\theta,0)=(-s_\psi s_\theta, c_\psi s_\theta, s_\psi c_\theta)$.

Step 4. Then multiply by $q_\phi=(c_\phi,(s_\phi,0,0))$. Scalar: $c_\psi c_\theta c_\phi-(-s_\psi s_\theta)s_\phi=c_\phi c_\theta c_\psi+s_\phi s_\theta s_\psi$ (the first row of Eq. 7a). The three vector components give the other three rows in the same way.

Quaternion → Euler (from comparing Eq. 4b with the Euler rotation matrix):

$$
\phi =\operatorname{atan2}(2(q_0q_1+q_2q_3),\;q_0^2-q_1^2-q_2^2+q_3^2)
$$

$$
\theta =\arcsin(2(q_0q_2-q_3q_1))
$$

$$
\psi =\operatorname{atan2}(2(q_0q_3+q_1q_2),\;q_0^2+q_1^2-q_2^2-q_3^2) \;\;\;\;\mathrm{(7b)}
$$

Derivation. For ZYX Euler angles: $R_{31}=-\sin\theta$, $R_{32}=\cos\theta\sin\phi$, $R_{33}=\cos\theta\cos\phi$, $R_{21}=\cos\theta\sin\psi$, $R_{11}=\cos\theta\cos\psi$. Match these to entries of Eq. (4b):
- $R_{31}=2(q_1q_3-q_2q_0)=-\sin\theta$ so $\theta=\arcsin(2(q_0q_2-q_1q_3))$
- $R_{32}/R_{33}=\mathrm{tan}\phi$ so $\phi=\operatorname{atan2}(2(q_2q_3+q_1q_0), q_0^2-q_1^2-q_2^2+q_3^2)$
- $R_{21}/R_{11}=\mathrm{tan}\psi$ so $\psi=\operatorname{atan2}(2(q_1q_2+q_3q_0), q_0^2+q_1^2-q_2^2-q_3^2)$

Paper typo: The paper prints $2(q_0q_1 \mathbf{-} q_2q_3)$ inside the $\phi$ formula. Derivation gives plus. This is most likely a typo.

Why useful: the controller works in quaternions, but humans (and plots) want angles in degrees.

NOTE: Understanding check: If $q=[1,0,0,0]$, what are $\phi,\theta,\psi$? (Answer: all 0. Check: $\theta=\arcsin 0=0$, $\phi=\operatorname{atan2}(0,1)=0$.)

### 2. Dynamic Model (Eq. 8 – 10)

#### Eq. (8) — Full nonlinear model (Newton–Euler with quaternions)

$$
\dot x=\frac{d}{dt}\begin{bmatrix}p \\ \dot p \\ q \\ \omega\end{bmatrix}= \begin{bmatrix}\dot p \\ q\otimes\frac{T}{m}\otimes q^*+\bar{g} \\ \frac12\;q\otimes\omega \\ J^{-1}(\tau-\omega\times J\omega)\end{bmatrix} \;\;\;\;\mathrm{(8)}
$$

State: $p$ (3) + $\dot p$ (3) + $q$ (4) + $\omega$ (3) = 13 states. Inputs: $T,\tau_x,\tau_y,\tau_z$ = 4 inputs. That is why the system is underactuated (6 degrees of freedom, 4 inputs).

**Row-by-row derivation**

Row 1: $\dot p$. The derivative of position is velocity. (Definition.)

Row 2: translation (Newton's second law $F=ma$ in the ground frame).

Step 1. Forces on the drone: thrust and gravity. $m\ddot p=F_{thrust}+m\bar{g}$.

Step 2. The thrust is produced by the rotors along the body $z$-axis: in the body frame it is $[0,0,T]^T$.

Step 3. To express it in the ground frame, rotate it with $q$ (Eq. 4): $q\otimes[0,0,T]\otimes q^*=R_q[0,0,T]^T$.

Step 4. Divide by $m$: $\ddot p=q\otimes\frac Tm\otimes q^*+\bar{g}$, with $\bar{g}=[0,0,-g]^T$.

Row 3: attitude kinematics is Eq. (6).

Row 4: rotation (Euler's rotation equation).

Step 1. Angular momentum in the body frame: $H=J\omega$.

Step 2. Newton's law for rotation: $\frac{dH}{dt}|_{ground}=\tau$.

Step 3. Converting the derivative from the rotating body frame to the ground frame adds a term: $\frac{dH}{dt}|_{ground}=\frac{dH}{dt}|_{body}+\omega\times H$.

Step 4. $J$ is constant in the body frame, so $J\dot\omega+\omega\times J\omega=\tau$, and therefore $\dot\omega=J^{-1}(\tau-\omega\times J\omega)$.

**Meaning of each part**
- $T$: total thrust. $m$: mass. $\bar{g}$: gravity vector. $J^{-1}$: inverse inertia (heavier/larger body so smaller angular acceleration).
- $\omega\times J\omega$: gyroscopic term. It appears because the body is rotating. It is zero when $\omega=0$, which matters for Eq. (10).
- $\tau=\tau_u+\tau_{ext}$: torque from the motors ($\tau_u$) plus external disturbances such as wind ($\tau_{ext}$).
- Assumptions: rigid body; centre of gravity at the origin of the body frame.

#### Eq. (9) — Motors → thrust and torques

$$
\begin{bmatrix}T \\ \tau_{u_x} \\ \tau_{u_y} \\ \tau_{u_z}\end{bmatrix}= \begin{bmatrix}\sum_{i=1}^{4}k_i\omega_i^2 \\ l\;(k_1\omega_1^2-k_2\omega_2^2-k_3\omega_3^2+k_4\omega_4^2) \\ l\;(k_1\omega_1^2+k_2\omega_2^2-k_3\omega_3^2-k_4\omega_4^2) \\ \sum_{i=1}^{4}\tau_i(-1)^{i}\end{bmatrix} \;\;\;\;\mathrm{(9)}
$$

Paper typo: The paper prints $(-1)^2$ in the last row, which would always be $+1$. It should be $(-1)^i$, so that the reaction torques alternate in sign.

**Derivation (physical reasoning)**

Step 1. A propeller spinning at $\omega_i$ gives thrust $F_i=k_i\omega_i^2$ (thrust grows with the square of the speed). Total thrust: $T=\sum F_i$.

Step 2. Roll/pitch torque = force × lever arm. A rotor at distance $l$ from the centre pushes up with $F_i$ and creates a torque $l F_i$ about the axis. Rotors on opposite sides of an axis give opposite signs. That gives the +/- patterns in rows 2 and 3.

Step 3. Yaw torque. A spinning propeller drags air, and the air pushes back on the drone with a reaction torque $\tau_i$. Neighbouring rotors spin in opposite directions, so their reaction torques alternate in sign: $\sum\tau_i(-1)^i$. If all four are equal, yaw torque is zero and the drone does not spin.

**Meaning of each part**
- $k_i$: thrust coefficient (depends on propeller shape). $\omega_i$: rotor speed. $l$: arm length. $\tau_i$: reaction (drag) torque of rotor $i$.
- The map is linear in $\omega_i^2$, so with four unknowns $\omega_i^2$ and four equations it can be inverted. We can therefore treat $(T,\tau_x,\tau_y,\tau_z)$ as the control inputs, and a "mixer" converts them into rotor speeds (the PWM signals in the paper's Fig. 8).

#### Eq. (10) — Linearisation at hover

Operating point (hover): $\dot p=[0,0,0]^T$, $\omega=[0,0,0]^T$, $q=[1,0,0,0]^T$.

$$
J\dot\omega=\tau,\;\;\;\; \dot q=\frac12\begin{bmatrix}0 \\ \omega\end{bmatrix},\;\;\;\; m\ddot p=\begin{bmatrix}mg\;(2q_2) \\ -mg\;(2q_1) \\ T\end{bmatrix}-\begin{bmatrix}0 \\ 0 \\ mg\end{bmatrix} =\begin{bmatrix}mg\;u_x \\ -mg\;u_y \\ T\end{bmatrix}-\begin{bmatrix}0 \\ 0 \\ mg\end{bmatrix} \;\;\;\;\mathrm{(10)}
$$

with $u_x=2q_2$ and $u_y=2q_1$ (small tilt values).

Paper typo: The paper prints $2q_0q_2$ in both the first and second row of the first vector. From the derivation below, the second row is $-2q_0q_1\approx-2q_1$.

**Derivation**

Step 1 — rotation part. Near hover, $\omega\approx0$, so the gyroscopic term $\omega\times J\omega$ is a product of two small numbers so ignore it. Eq. (8) row 4 becomes $J\dot\omega=\tau$.

Step 2 — kinematics. With $q\approx[1,0,0,0]$ in Eq. (6): $q\otimes\omega\approx\omega$, so $\dot q\approx\frac12[0;\omega]$.

Step 3 — translation. Take the third column of $R_q$ from Eq. (4b), which is where the thrust points, and multiply by $T$:

$$
m\ddot p=T\begin{bmatrix}2(q_1q_3+q_2q_0) \\ 2(q_2q_3-q_1q_0) \\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}-\begin{bmatrix}0 \\ 0 \\ mg\end{bmatrix}
$$

Step 4 — small angles. Near hover $q_0\approx1$ and $q_1,q_2,q_3$ are small, so products like $q_1q_3$ and squares like $q_1^2$ are negligible:
- $x$-row: $2(q_1q_3+q_2q_0)\approx2q_2$
- $y$-row: $2(q_2q_3-q_1q_0)\approx-2q_1$
- $z$-row: $q_0^2-\ldots\approx1$

Step 5 — thrust near hover. $T\approx mg$ (the thrust that balances gravity), so in the horizontal rows $T\cdot2q_2\approx mg\cdot2q_2$. The $z$ row stays $T-mg$.

Result: Eq. (10).

Meaning: a small tilt ($q_2$ about $y$, $q_1$ about $x$) makes the thrust point slightly sideways, giving horizontal acceleration $\ddot x\approx 2g q_2$ and $\ddot y\approx-2g q_1$ (about $g\times$ tilt angle, since $2q_2\approx\theta$). Thrust above $mg$ makes the drone climb.

NOTE: Understanding check: At hover, what is $T$? (Answer: $T=mg$, so $\ddot z=0$.)

### 3. Differential Flatness (Eq. 11 – 15)

#### Eq. (11), (12) — Definition

$$
\dot x(t)=f(x(t),u(t)),\;\;\;\; y=h(x(t)) \;\;\;\;\mathrm{(11)}
$$

The system is flat if there is an output $z(t)$ (the flat output) such that all states and inputs can be written using only $z$ and a finite number of its derivatives:

$$
x(t)=\Xi_1(z,\dot z,\ddot z,\ldots,z^{(\alpha)}),\;\;\;\; u(t)=\Xi_2(z,\dot z,\ddot z,\ldots,z^{(\beta)}) \;\;\;\;\mathrm{(12)}
$$

**Meaning of each part**
- $x$: states, $u$: inputs, $f$: system dynamics, $h$: output map.
- $z$: flat output. $z^{(i)}$: its $i$-th time derivative. $\alpha,\beta$: finite integers (how many derivatives are needed).
- $\Xi_1,\Xi_2$: smooth maps (just algebraic formulas, with no integration needed).
- The components of $z$ must be differentially independent (none is a function of the others and their derivatives).

Simple example first (extra, not in the paper). The double integrator $\ddot x=u$ with state $[x,\dot x]$.
- Choose $z=x$. Then $x=z$, $\dot x=\dot z$, and $u=\ddot z$. Both state and input follow from $z$ and its derivatives, so the system is flat.
- Planning means: draw any smooth path $z(t)$, and the needed input is just $\ddot z(t)$, with no differential equation to solve.

Why it helps the quadrotor: plan a smooth path for $x,y,z,\psi$, and the required angles, thrust and torques come out as formulas of the path's derivatives.

#### Eq. (13), (14) — Flat outputs of the quadrotor

$$
{X}=\begin{bmatrix}x \\ y \\ z \\ \psi\end{bmatrix} \;\;\;\;\mathrm{(13)}
$$

$$
\begin{bmatrix}x & y & z\end{bmatrix}^T =\begin{bmatrix}{X}_1 & {X}_2 & {X}_3\end{bmatrix}^T
$$

$$
\begin{bmatrix}\dot x & \dot y & \dot z\end{bmatrix}^T =\begin{bmatrix}\dot{X}_1 & \dot{X}_2 & \dot{X}_3\end{bmatrix}^T \;\;\;\;\mathrm{(14)}
$$
- Four flat outputs for four inputs. Position gives velocity and acceleration by differentiating, which is "trivial".
- All 13 states and 4 inputs can be recovered from $x,y,z,\psi$ and their derivatives (up to the 4th derivative for the torques).

#### Eq. (15) — Desired pitch and roll from the desired acceleration

$$
\theta_d=\arctan(\frac{\ddot{X}_1\cos{X}_4+\ddot{X}_2\sin{X}_4}{\ddot{X}_3+g}) \;\;\;\;\mathrm{(15a)}
$$

Derivation (this is the key flatness step)

Step 1. Take the translation equation of Eq. (8) and write the rotation $R$ with Euler angles ($R=R_z(\psi)R_y(\theta)R_x(\phi)$). The thrust direction (third column of $R$) is:

$$
R\;e_3=\begin{bmatrix}\cos\phi\sin\theta\cos\psi+\sin\phi\sin\psi \\ \cos\phi\sin\theta\sin\psi-\sin\phi\cos\psi \\ \cos\phi\cos\theta\end{bmatrix}
$$

Step 2. Newton's law $\ddot p=\frac Tm Re_3+[0,0,-g]^T$ gives three equations:

$$
\ddot x=\frac Tm(\cos\phi\sin\theta\cos\psi+\sin\phi\sin\psi)
$$

$$
\ddot y=\frac Tm(\cos\phi\sin\theta\sin\psi-\sin\phi\cos\psi)
$$

$$
\ddot z+g=\frac Tm\cos\phi\cos\theta
$$

Step 3 — pitch. Form $\ddot x\cos\psi+\ddot y\sin\psi$. The $\sin\phi$ terms cancel and $\cos^2\psi+\sin^2\psi=1$:

$$
\ddot x\cos\psi+\ddot y\sin\psi=\frac Tm\cos\phi\sin\theta
$$

Step 4. Divide by the third equation: $\frac{\ddot x\cos\psi+\ddot y\sin\psi}{\ddot z+g}=\frac{\sin\theta}{\cos\theta}=\mathrm{tan}\theta$. Hence Eq. (15a), with ${X}_1=x, {X}_2=y, {X}_3=z, {X}_4=\psi$.

Step 5 — roll. Form $\ddot x\sin\psi-\ddot y\cos\psi=\frac Tm\sin\phi$. Divide by $\frac Tm\cos\phi=(\ddot z+g)/\cos\theta$:

$$
\phi_d=\arctan(\frac{\ddot{X}_1\sin{X}_4-\ddot{X}_2\cos{X}_4}{\ddot{X}_3+g}\cdot\cos\theta_d) \;\;\;\;\mathrm{(15b)}
$$

Paper typo: The paper prints $\ddot{X}_1\cos{X}_4-\ddot{X}_2\sin{X}_4$ for roll. For $\psi=0$ the correct roll should depend on the $y$-acceleration (side motion needs roll), which the paper's form does not show. If asked, say: "Eq. (15a) matches my derivation exactly. For (15b) my Z-Y-X derivation gives sin/cos swapped; the paper may use a different sign or frame convention."

Step 6 — thrust (needed for the altitude controller). From the third equation:

$$
T=\frac{m(\ddot z+g)}{\cos\phi\cos\theta}
$$

Step 7 — rates. $\dot\theta_d$ and $\dot\phi_d$ in Eq. (15) are the time derivatives of (15a) and (15b) (the superscript $^{(1)}$ in the paper). This needs the third derivative of position (jerk), which is why the planned path must be smooth.

**Meaning of each part**
- Numerator of (15a): horizontal acceleration component along the heading direction. Denominator: vertical acceleration plus gravity (the thrust needed vertically).
- $\psi$ (yaw) is a free choice, as one of the four flat outputs.
- $\arctan(\mathrm{sideways}/\mathrm{vertical})$: simply the tilt angle needed so the thrust vector has the wanted sideways component.

Worked example (extra, not in the paper). Circle of radius $r$, angular rate $\Omega$, constant altitude, $\psi=0$:
- $x_d=r\cos\Omega t, y_d=r\sin\Omega t, z_d=\mathrm{const}$, so $\ddot x_d=-\Omega^2x_d, \ddot y_d=-\Omega^2y_d, \ddot z_d=0$.
- Then $\theta_d=\arctan (-\Omega^2x_d/g)$ and $\phi_d=\arctan (+\Omega^2y_d/g\cdot\cos\theta_d)$.
- With $r=1$ m and an assumed period of 8 s ($\Omega=2\pi/8\approx0.785$ rad/s): $\Omega^2\approx0.617$, so max acceleration $\approx0.617$ m/s² and max tilt $\approx\arctan(0.617/9.81)\approx3.6^\circ$. Small tilts, so the linear model of Eq. (10) is a reasonable approximation for this slow circle.

NOTE: Understanding check: Why does the drone need to tilt to move sideways? (Answer: thrust only acts along the body $z$-axis; tilting puts part of the thrust in the horizontal direction.)

### 4. Controller Design (Eq. 16 – 21)

#### Eq. (16) — General linear state-space

$$
\dot x=Ax+Bu,\;\;\;\; y=Cx \;\;\;\;\mathrm{(16)}
$$
- $x$: state vector, $u$: input, $y$: output.
- $A$: how states evolve by themselves. $B$: how inputs push the states. $C$: which combination of states we measure.

#### Eq. (17) — Attitude subsystem

State $x_A=[\bar{q}; \omega]\in\mathbb{R}^6$, input $=\tau_u=[\tau_{u_x},\tau_{u_y},\tau_{u_z}]^T$:

$$
\dot x_A=\begin{bmatrix}0_{3\times3} & 0.5\;I_{3\times3} \\ 0_{3\times3} & 0_{3\times3}\end{bmatrix}x_A+\begin{bmatrix}0_{3\times3} \\ I_q^{-1}\end{bmatrix}\begin{bmatrix}\tau_{u_x} \\ \tau_{u_y} \\ \tau_{u_z}\end{bmatrix} \;\;\;\;\mathrm{(17)}
$$

Derivation from Eq. (10)

Step 1. From $\dot q\approx\frac12[0;\omega]$, the vector part gives $\dot{\bar{q}}=0.5 \omega$. This is the top row: $[ 0  0.5I ] x_A=0.5 \omega$.

Step 2. From $J\dot\omega=\tau$, $\dot\omega=J^{-1}\tau$. This is the bottom row: the state part is $0$ (angular acceleration does not depend on the current state) and the input part is $J^{-1}$.

Step 3. In the paper, the inertia matrix is written $I_q$, so the input matrix is $I_q^{-1}$.

Meaning: the attitude loop is a chain of two integrators: torque → angular velocity → quaternion vector part. We drive $\bar{q}\rightarrow0$ (identity orientation) or, with an error quaternion, $\bar{q}_e\rightarrow0$ (see below).

#### Eq. (18) — Position subsystem

State $x_P=[p; \dot p]\in\mathbb{R}^6$:

$$
\dot x_P=\begin{bmatrix}0_{3\times3} & I_{3\times3} \\ 0_{3\times3} & 0_{3\times3}\end{bmatrix}x_P+\begin{bmatrix}0_{3\times3} \\ \mathrm{diag}(g,\;-g,\;\frac1m)\end{bmatrix}\begin{bmatrix}u_{xd} \\ u_{yd} \\ T_d\end{bmatrix} \;\;\;\;\mathrm{(18)}
$$

Derivation from Eq. (10)

Step 1. Top row: $\dot p=\dot p$, so $[ 0  I ] x_P=\dot p$.

Step 2. Bottom row from the translation part of Eq. (10):
- $\ddot x=g u_x$
- $\ddot y=-g u_y$
- $\ddot z=\frac{T-mg}{m}=\frac{T_d}{m}$, where $T_d$ is the thrust deviation from hover thrust.

Step 3. Put these into the matrix form: the diagonal $\mathrm{diag}(g,-g,1/m)$ multiplies the inputs $(u_{xd},u_{yd},T_d)$.

**Meaning of each part**
- $u_{xd},u_{yd}$: desired tilt commands (they equal $2q_2$ and $2q_1$ in Eq. 10).
- $T_d$: desired extra thrust from the altitude controller.
- Each axis is a double integrator: acceleration is proportional to the input. LQR is easy to apply to this.

#### Controller structure (paper's Fig. 2 and Fig. 3)

1. Altitude controller (LQR on the $z$ channel): outputs the desired total thrust $T_d$.

2. Position controller (LQR on $x,y$): outputs the desired tilt, i.e. the desired position quaternion $q_{pd}$.

3. Combine $q_{pd}$ with the yaw quaternion into the desired attitude $q_d$ (Eq. 19–21).

4. Attitude controller (inner loop, LQR using Eq. 17): outputs torques $\tau_{u_x},\tau_{u_y},\tau_{u_z}$.

5. Total thrust $T$ and torques are turned into rotor PWM signals via Eq. (9).

Why two loops? Position cannot be controlled directly: first the drone must tilt (inner loop), and only then does it accelerate sideways (outer loop). The inner loop must be faster than the outer one, so that the tilt is reached before the position controller expects it.

#### Eq. (19) — Desired position quaternion

$$
q_{pd}=\begin{bmatrix}q_{0pd} & q_{1pd} & q_{2pd} & 0\end{bmatrix}^T \;\;\;\;\mathrm{(19)}
$$

Meaning and origin
- It describes only the tilt (roll/pitch), with no yaw, so the $z$-component is 0. (The paper calls it "4th element" because it counts 1–4.)
- (Extra reasoning consistent with Eq. 10) From Eq. (10), $u_x=2q_2$ and $u_y=2q_1$, so $q_{2pd}=u_{xd}/2$, $q_{1pd}=u_{yd}/2$, and since the quaternion must have unit length, $q_{0pd}=\sqrt{1-q_{1pd}^2-q_{2pd}^2}$.

#### Eq. (20) — Desired yaw quaternion

$$
q_{zd}=\begin{bmatrix}q_{0zd} & 0 & 0 & q_{3zd}\end{bmatrix}^T=\begin{bmatrix}\cos\frac{\psi_d}{2} & 0 & 0 & \sin\frac{\psi_d}{2}\end{bmatrix}^T \;\;\;\;\mathrm{(20)}
$$

Derivation: Eq. (5) with rotation axis $u=[0,0,1]^T$ (the $z$-axis) and angle $\alpha=\psi_d$. The second and third elements are zero because the axis has no $x$ or $y$ component. ($\psi_d$ is the 4th flat output.)

#### Eq. (21) — Combined desired attitude

$$
q_d=q_{pd}\otimes q_{zd} \;\;\;\;\mathrm{(21)}
$$

Meaning: "do the yaw rotation and the tilt rotation together" by multiplying the two quaternions with Eq. (2). The order matters because $\otimes$ is not commutative (the cross-product term in Eq. 2 changes sign when the factors are swapped). The result is again a unit quaternion.

How $q_d$ is used (extra, standard practice from the references the paper cites, e.g. Fresk & Nikolakopoulos; not printed in this paper). The attitude loop works on the error quaternion $q_e=q_d^{*}\otimes q$. When the vector part $\bar{q}_e\rightarrow0$, the actual orientation $q$ equals the desired $q_d$. That is why $x_A$ in Eq. (17) contains the vector part $\bar{q}$: driving it to zero means "no attitude error".

### 5. LQR (Eq. 22 – 23)

#### Eq. (22) — Cost function

$$
J(x,u)=\frac12\int_0^{\infty}(x^T(t)\;Q\;x(t)+u^T(t)\;R\;u(t))\;dt \;\;\;\;\mathrm{(22)}
$$

**Meaning of each part**
- $x^TQx$: penalty for being away from the target (state error). $Q\geq0$ (positive semi-definite), usually diagonal. A big entry means "keep this state small".
- $u^TRu$: penalty for using a lot of control effort. $R>0$ (positive definite). Big $R$ means "be gentle with the motors/battery".
- $\int_0^\infty$: judge the whole future from now to infinity (an infinite-horizon problem).
- $\frac12$: only for convenience; it cancels the 2 that appears when differentiating.
- Because both terms are $\geq0$ and we need the integral to be finite, minimising $J$ forces $x\rightarrow0$, which gives stability.

Trade-off analogy: driving a car. Big $Q$ = "arrive exactly on the line, even with harsh braking". Big $R$ = "drive smoothly, even if you drift a bit".

NOTE: The paper says " $R$ and $Q$ are weight matrices used respectively to increase or diminish the effect of the states and the inputs". The order is swapped in that sentence: $Q$ weights states and $R$ weights inputs.

#### Eq. (23) and the control law — Derivation

Goal: find $u$ that minimises Eq. (22) subject to $\dot x=Ax+Bu$ (Eq. 16).

Step 1 — value function. Define $V(x)=\frac12x^TPx$, the minimum remaining cost from state $x$ ($P$ symmetric, $P>0$).

Step 2 — optimality condition (Hamilton–Jacobi–Bellman). At the optimum, the running cost plus the rate of change of $V$ is zero:

$$
\min_u[\frac12(x^TQx+u^TRu)+x^TP\;(Ax+Bu)]=0
$$

(because $\dot V=\frac{\partial V}{\partial x}\dot x=x^TP(Ax+Bu)$).

Step 3 — minimise over $u$. Differentiate the bracket w.r.t. $u$ and set it to zero: $Ru+B^TPx=0$, so

$$
u=-R^{-1}B^TPx=-Lx,\;\;\;\; L=R^{-1}B^TP
$$

This is the paper's " $u=-Lx$, $L=R^{-1}B^TP$ ".

Step 4 — plug back. With $u=-R^{-1}B^TPx$: $u^TRu=x^TPBR^{-1}B^TPx$ and $x^TPBu=-x^TPBR^{-1}B^TPx$. The bracket becomes

$$
\frac12x^TQx+\frac12x^TPBR^{-1}B^TPx+x^TPAx-x^TPBR^{-1}B^TPx=0
$$

Step 5 — symmetrise. $x^TPAx=\frac12x^T(PA+A^TP)x$. Collect terms:

$$
\frac12x^T[A^TP+PA-PBR^{-1}B^TP+Q]x=0\;\;\mathrm{for all }x
$$

Step 6. The matrix in brackets must be zero:

$$
A^TP+PA-PBR^{-1}B^TP+Q=0 \;\;\;\;\mathrm{(23)}
$$

This is the algebraic Riccati equation (ARE).

Paper typo: The paper calls it the "Riccati differential equation". The finite-horizon problem has $-\dot P=\ldots$; for the infinite horizon used here, $\dot P=0$ and we get the algebraic form (23).

Why it is stable (idea). With $V=\frac12x^TPx$ as a Lyapunov function, $\dot V=-\frac12(x^TQx+u^TRu)<0$, so $x\rightarrow0$. LQR needs $(A,B)$ controllable (true for Eq. 17 and 18 because they are chains of integrators).

**Meaning of each part**
- $P$: the matrix that measures "how costly is each state error from now on".
- $L$: gain matrix. $u=-Lx$ is state feedback (each input is a weighted sum of all state errors).
- $R^{-1}B^TP$: bigger $R$ so smaller gain so gentler control.

**Design recipe (matches the paper's steps)**

1. Get $A,B$ (Eq. 17, 18).

2. Choose $Q,R$.

3. Solve ARE (23) for $P$.

4. Compute $L=R^{-1}B^TP$.

5. Apply $u=-Lx$ with $x$ = state error, $e=x_d-x$.

#### Worked example 1 — scalar system (extra, for exam practice)

$\dot x=u$ so $A=0, B=1$. ARE: $0+0-P\cdot\frac1R\cdot P+Q=0=> P=\sqrt{QR}$, then $L=P/R=\sqrt{Q/R}$.

| $Q$ | $R$ | $P$ | $L$ | Control law |
| --- | --- | --- | --- | --- |
| 1 | 1 | 1 | 1 | $u=-x$ |
| 1 | 4 | 2 | 0.5 | $u=-0.5x$ (gentler) |
| 4 | 1 | 2 | 2 | $u=-2x$ (more aggressive) |

#### Worked example 2 — double integrator (like each axis of Eq. 18) (extra)

$\ddot x=u$ gives the matrices below. Take $Q=I$, $R=1$, and let $P$ be the unknown symmetric matrix:

$$
A=\begin{bmatrix}0 & 1 \\ 0 & 0\end{bmatrix},\;\; B=\begin{bmatrix}0 \\ 1\end{bmatrix},\;\; P=\begin{bmatrix}p_1 & p_2 \\ p_2 & p_3\end{bmatrix}
$$

Step 1. Work out the three matrix products that appear in Eq. (23):

$$
A^TP=\begin{bmatrix}0 & 0 \\ p_1 & p_2\end{bmatrix},\;\; PA=\begin{bmatrix}0 & p_1 \\ 0 & p_2\end{bmatrix},\;\; PBB^TP=\begin{bmatrix}p_2^2 & p_2p_3 \\ p_2p_3 & p_3^2\end{bmatrix}
$$

Step 2. Eq. (23) entry by entry:
- (1,1): $-p_2^2+1=0=> p_2=1$
- (1,2): $p_1-p_2p_3=0=> p_1=p_3$
- (2,2): $2p_2-p_3^2+1=0=> p_3^2=3=> p_3=\sqrt3$

Step 3. Putting the values together:

$$
P=\begin{bmatrix}\sqrt3 & 1 \\ 1 & \sqrt3\end{bmatrix},\;\;\;\; L=R^{-1}B^TP=[\;1\;\;\sqrt3\;]\approx\begin{bmatrix}1 & 1.73\end{bmatrix}
$$

Step 4. Control law: $u=-1\cdot x-1.73 \dot x$, closed loop $\ddot x+1.73\dot x+x=0$, so $\omega_n=1$, $\zeta=0.866$: stable and well damped.

### 6. Simulation and Results (Section "Simulation Results")

| Item | Paper value | Meaning |
| --- | --- | --- |
| Path | Circle, fixed altitude 1 m | Test of $x,y$ tracking while $z$ is held |
| Start point | $p_0=(-1,0,0)$ | On the ground, on the circle's edge |
| Rate | 200 Hz | Controller updates every 5 ms |
| Effects included | Actuator limits, battery model | Closer to a real drone |
| Result | "High accuracy"; ≈1 s delay in X and Y | Delay because the drone must first climb to 1 m (altitude settling) |
- Figs. 4–6: X, Y, Z responses (reference vs actual). Fig. 7: the 3D path. Fig. 8: PWM signals of four rotors (ESC range 1000–2000 µs). Fig. 9: quaternion components, with $q_0\approx1$ and $q_1,q_2,q_3\approx0$ meaning the attitude stays near hover.
- The gain matrix $L$ was "tuned to be adaptive with the full non-linear model" (Eq. 8). In other words, the LQR gains came from the linear model (Eq. 17–18), and were then tuned in simulation on the nonlinear model.

#### 6.1 Our reproduction — measured study-case results (this project)

Same controller and rates as the paper (200 Hz zero-order hold), 0.2 kg vehicle with the paper's inertia. RMS after the settling transient, deterministic seed 0:

| Plant | RMS x | RMS y | RMS z | RMS radial | Peak rotor |
| --- | --- | --- | --- | --- | --- |
| Paper's Eq. (8) ideal model (RK4 @ 1 kHz) | 1.6 cm | 3.6 cm | 0.6 cm | 2.7 cm | 0.57 N |
| MuJoCo contact physics (20 kHz) | 1.4 cm | 3.7 cm | 0.7 cm | 2.8 cm | 0.99 N |
| MuJoCo + paper sensor noise ±0.1 | 2.3 cm | 4.6 cm | 0.9 cm | 3.7 cm | 1.69 N |

≈97% radial accuracy on the 1 m-radius circle with zero rotor saturation (per-rotor limit 1.95 N) — and near-identical numbers on two independent plants, confirming the flatness map (not the integrator) does the work.

#### 6.2 Extensions — new flat-output paths, same frozen controller (E1/E2)

Generated by `python -m flatness.run_scenarios` (min-snap waypoints from `flatness/polytraj.py`, figure-8/orbit from `flatness/trajectories.py`). 3D RMS after settling:

| Scenario | RMS 3D | Peak rotor |
| --- | --- | --- |
| figure-8 (A=1.5/B=1, yaw option) — ideal model | 4.7 cm | 0.57 N |
| figure-8 — MuJoCo contact physics | 4.7 cm | 1.01 N |
| vertical orbit (cone sweep) — ideal model | 3.9 cm | 0.57 N |
| waypoints, min-snap (5 waypoints @ 0.35 m/s) | 6.1 cm | 0.83 N |
| waypoints, min-jerk (A/B control) | 8.1 cm | 0.84 N |

Min-snap tracks 25% better than min-jerk on identical waypoints and times — the flatness argument measured: snap (the 4th derivative) drives the torques, so a path that minimizes snap is cheaper to track. The extension guarantees (waypoints hit to 1e-10, velocity/acceleration continuity at knots, rest-to-rest ends) are verified by `tests/test_flatness_extensions.py` in the project repository.

### 7. Typos / Inconsistencies in the Printed Paper

Mention these only if asked; they show you checked the maths.

| Issue | Where | Printed | Correct / note |
| --- | --- | --- | --- |
| 1 | Eq. (3) |  $q_2^2+q_3^2+q_4^2$ |  $q_1^2+q_2^2+q_3^2$ (index slip) |
| 2 | After Eq. (3) |  $q^{-1}=q^*/\| q\|$ |  $q^*/\| q\|^2$ |
| 3 | Eq. (7), $\phi$ |  $2(q_0q_1-q_2q_3)$ |  $2(q_0q_1+q_2q_3)$ |
| 4 | Eq. (9), yaw |  $\sum\tau_i(-1)^2$ |  $\sum\tau_i(-1)^i$ |
| 5 | Eq. (10), $y$-row |  $-2q_0q_2$ |  $-2q_0q_1\approx-2q_1$ |
| 6 | Eq. (15), $\phi_d$ |  $\ddot X_1\cos X_4-\ddot X_2\sin X_4$ | Z-Y-X derivation: $\ddot X_1\sin X_4-\ddot X_2\cos X_4$ |
| 7 | Eq. (23) text | "Riccati differential equation" | Eq. (23) is the algebraic Riccati equation |
| 8 | Eq. (22) text | "R and Q ... states and inputs" order |  $Q$ → states, $R$ → inputs |
| 9 | Simulation | "1 m diameter", start $(-1,0,0)$ | Plots swing between −1 and +1, so the radius looks like 1 m |
| 10 | Fig. 8 text | PWM "1–2 sec" | Axis shows 1000–2000, i.e. 1–2 ms (µs units) |

### 8. Exam Points

#### Key formulas to remember

| Topic | Formula |
| --- | --- |
| Quaternion | $q=q_0+\bar{q}$, $\| q\|=1$ |
| Product | $q\otimes r=(q_0r_0-\bar{q}\cdot\bar{r})+(r_0\bar{q}+q_0\bar{r}+\bar{q}\times\bar{r})$ |
| Conjugate / inverse | $q^*=q_0-\bar{q}$, $q^{-1}=q^*/\| q\|^2$ (unit: $q^{-1}=q^*$) |
| Rotate a vector | $v'=q\otimes v\otimes q^*=R_qv$, $R_q^{-1}=R_q^T$ |
| Axis–angle | $q=\cos\frac\alpha2+u\sin\frac\alpha2$ |
| Kinematics | $\dot q=\frac12q\otimes\omega$ |
| Translation | $\ddot p=q\otimes\frac Tm\otimes q^*+\bar{g}$ |
| Rotation | $J\dot\omega=\tau-\omega\times J\omega$ |
| Hover linear model | $\ddot x=g u_x, \ddot y=-g u_y, \ddot z=T_d/m, J\dot\omega=\tau$ |
| Flat outputs | $[x,y,z,\psi]^T$ |
| Desired pitch | $\theta_d=\arctan\frac{\ddot x\cos\psi+\ddot y\sin\psi}{\ddot z+g}$ |
| Desired attitude | $q_d=q_{pd}\otimes q_{zd}$ |
| LQR cost | $J=\frac12\int_0^\infty(x^TQx+u^TRu)dt$ |
| LQR law | $u=-Lx, L=R^{-1}B^TP$ |
| ARE | $A^TP+PA-PBR^{-1}B^TP+Q=0$ |

#### Definitions
- Underactuated: fewer independent control inputs (4) than degrees of freedom (6).
- Gimbal lock: loss of one rotational degree of freedom when two Euler axes align (at $\theta=\pm90^\circ$).
- Differentially flat: all states and inputs are algebraic functions of some outputs and a finite number of their derivatives.
- LQR: optimal state-feedback controller that minimises a quadratic cost of state error and control effort.

#### Common mistakes

1. Using non-unit quaternions with $q^{-1}=q^*$. This only holds when $\| q\|=1$.

2. Forgetting that $q\otimes r\neq r\otimes q$.

3. Mixing up $Q$ (states) and $R$ (inputs).

4. Forgetting the half-angles in $q=\cos\frac\alpha2+u\sin\frac\alpha2$.

5. Writing the ARE as a differential equation. For infinite horizon it is algebraic.

6. Saying "LQR is nonlinear". It is designed on a linear model (here, linearised at hover).

#### Likely questions with short model answers

1. Why quaternions? No gimbal lock, 4 numbers with one constraint, cheap to compose and to invert (conjugate).

2. How does $q\otimes v\otimes q^*$ rotate a vector? Expanding gives Rodrigues' formula (Eq. 4c to Eq. 5), the same as multiplying by $R_q$.

3. Where does $\dot q=\frac12q\otimes\omega$ come from? A small rotation $\delta q\approx[1,\frac12\omega dt]$ is composed with $q$ (Eq. 6 derivation).

4. How is the linear model obtained? Hover point $q=[1,0,0,0], \omega=0, T=mg$; drop second-order terms in Eq. (8) to get Eq. (10), then Eq. (17), (18).

5. What makes the quadrotor flat and why does it help? The four outputs $x,y,z,\psi$ determine every state and input (Eq. 15), so planning happens in output space with simple formulas.

6. Derive $\theta_d$. Combine $\ddot x\cos\psi+\ddot y\sin\psi=\frac Tm\cos\phi\sin\theta$ with $\ddot z+g=\frac Tm\cos\phi\cos\theta$ and divide.

7. Derive the LQR gain. HJB with $V=\frac12x^TPx$ gives $u=-R^{-1}B^TPx$, and substituting back gives the ARE.

8. What do $Q$ and $R$ do? Larger $Q$ = tighter tracking, larger $R$ = gentler control.

9. Why a double loop? Position is controlled through tilt; the faster inner attitude loop must achieve the tilt that the outer position loop requests.

10. Limitations? Simulation only, one circular trajectory, linear controller designed at hover. The authors suggest nonlinear control such as backstepping.

### 9. References

1. Choutri, K., Lagha, M., Dala, L., & Lipatov, I. (2017). Quadrotors trajectory tracking using a differential flatness-quaternion based approach. Proceedings of the 2017 7th International Conference on Modeling, Simulation, and Applied Optimization (ICMSAO), IEEE, pp. 1–6. DOI: https://doi.org/10.1109/ICMSAO.2017.7934901

2. Fresk, E., & Nikolakopoulos, G. (2013). Full quaternion based attitude control for a quadrotor. European Control Conference (ECC), pp. 3864–3869. DOI: https://doi.org/10.23919/ECC.2013.6669617 — source of the quaternion algebra toolbox (Eqs. 1–7 of this file) and of the error-quaternion practice noted under Eq. (19).

3. Chovancová, A., Fico, T., Hubinský, P., & Duchoň, F. (2016). Comparison of various quaternion-based control methods applied to quadrotor with disturbance observer and position estimator. Robotics and Autonomous Systems, 79, 104–116. DOI: https://doi.org/10.1016/j.robot.2016.01.011 — the quaternion-cascade controller that paper 2's attitude loop builds on.

---

## Results — the paper's study case (circle, radius 1 m at 1 m altitude)

RMS after the settling transient, 2 full circles, deterministic seed 0
(after the validation fixes below):

| Plant | RMS x | RMS y | RMS z | **RMS radial** | Peak rotor | Simulator |
|---|---|---|---|---|---|---|
| Paper's Eq. (8) ideal model | 1.6 cm | 3.6 cm | 0.6 cm | **2.7 cm** | 0.57 N | `python -m flatness.run_paper` |
| **MuJoCo contact physics** (20 kHz) | 1.4 cm | 3.7 cm | 0.7 cm | **2.8 cm** | 0.99 N | `python -m flatness.run_mujoco` |
| MuJoCo + paper sensor noise ±0.1 | 2.3 cm | 4.6 cm | 0.9 cm | **3.7 cm** | 1.69 N | `run_mujoco(noise=0.1)` |

**≈97% radial accuracy**, zero rotor saturation (per-rotor limit 1.95 N), and
near-identical numbers across two independent plants — the flatness map, not the
integrator, does the work. The paper's own figures (Figs. 4–9: X/Y/Z responses, 3D
trajectory, motor PWM signals, quaternion stability) are reproduced as
`results/flatness/*.png` for both plants.

<p align="center">
  <img src="results/flatness/axis_responses.png" width="420" alt="Flatness circle: X/Y/Z tracking, ideal model">
  <img src="results/flatness/mujoco_trajectory_3d.png" width="380" alt="Flatness circle: 3D trajectory, MuJoCo">
</p>
<p align="center">
  <img src="results/recordings/mujoco_flatness_circle_RENDER.gif" width="420" alt="In-engine MuJoCo render of the study case: drone tracking the reference ring, with desired-attitude triad">
</p>

## Key findings 3–5 (from building paper 2)

3. **Naive rotor clipping runs the altitude loop away.** Clipping per-rotor thrust at
   [0, max] corrupts the *total* thrust one-sidedly (clipped rotors lose thrust,
   unclipped keep it) → the z-loop sees false thrust → runs away to thousands of meters.
   Fix: **priority desaturation** — thrust is always delivered exactly; infeasible
   torque is uniformly scaled (`flatness/model.py::mix`), the same idea as real flight
   controllers and `pysitl`'s mixer.
4. **The actuator lag is load-bearing.** The paper says ESC/battery limits are "taken
   into consideration" but gives no model. With our documented 20 ms first-order ESC
   lag the loop is stable; with the lag removed, the raw 200 Hz ZOH torque is marginally
   stable and the lateral mode diverges — proven by running the *ideal* model without
   the filter (it diverges identically, 1.7 m). Real motors don't change thrust
   instantly; the lag belongs in the loop.
5. **The pinned interactive plant keeps shortest-path ON.** The free-falling benchmark
   flip needs `shortest_path=False` (the error sign at exactly 180° is ambiguous), but
   on the *position-pinned* interactive plant the flip never stalls past 180° of lag —
   shortest-path coasts over the top and converges (peak 180°, final error < 10°).
   Different plants, different sign choice — both documented.

## Assumptions where the paper gives no numbers (all validated as deliberate)

| Quantity | Paper | Our documented choice |
|---|---|---|
| LQR weights Q, R | "tuned to be adaptive" — no values | outer (49, 42; r=1) per axis; inner (100, 20; r=200·I₃) — tuned on the repo vehicle, `flatness/lqr.py` |
| Circle size | "1 m of diameter" **and** start (−1,0,0) — self-contradictory (a 1 m circle about the origin can't pass through a point 1 m away; no center given) | radius 1 m — resolves in favor of the explicit start point, documented in `trajectories.py` |
| Circle speed / direction / revolutions | not stated | 0.5 m/s tangential, clockwise, 2 revolutions |
| Climb onto the circle | not stated (Figs. 4–5 imply ~1 s settling from rest) | 2 s min-jerk altitude + angular-rate ramp, starts at rest |
| ESC / battery model | "taken into consideration" — no model | first-order 20 ms lag + per-rotor thrust caps (finding 4) |
| Gravity | symbolic ḡ | 9.81 m/s² |
| Plant integrator | only the 200 Hz controller rate is stated | RK4 at 1 kHz, zero-order hold |
| External torque τ_ext | in Eq. (8), never exercised | 0 (no disturbance in the study case) |
| Inner-loop rate reference | Eq. (15) parameterizes ω from σ derivatives; no values | ω_ref = 0 (hover-linearized inner model; second-order effect on this path) |
| Acceleration clamps | qualitative only | 3 m/s² lateral, 5 m/s² vertical (actuator-aware) |
| Zero-thrust point | not discussed | (T_d, q_d) = (0, identity) |

## Extensions delivered - Phase A (trajectory library + aggressive paths)

Same frozen double-loop LQR stack (gains untouched), new flat-output
generators only - `python -m flatness.run_scenarios`:

| Scenario (ideal Eq. (8) model) | RMS x | RMS y | RMS z | **RMS 3D** | Peak rotor |
|---|---|---|---|---|---|
| circle (the paper's study case) | 1.6 cm | 3.6 cm | 0.6 cm | **3.9 cm** | 0.57 N |
| **figure-8** (E2, A=1.5/B=1, yaw option) | 2.5 cm | 3.9 cm | 0.9 cm | **4.7 cm** | 0.57 N |
| **figure-8 on MuJoCo** contact physics | 2.4 cm | 4.0 cm | 1.0 cm | **4.7 cm** | 1.01 N |
| **vertical orbit** (E2, cone sweep) | 1.6 cm | 3.5 cm | 0.6 cm | **3.9 cm** | 0.57 N |
| **waypoints, min-snap** (E1, 5 wp tour @ 0.35 m/s) | 4.6 cm | 3.9 cm | 1.3 cm | **6.1 cm** | 0.83 N |
| waypoints, min-jerk (A/B control) | 5.8 cm | 5.2 cm | 2.3 cm | **8.1 cm** | 0.84 N |

**A/B result: min-snap tracks 25% better than min-jerk** on identical
waypoints and times - the flatness argument (snap drives torques) measured.

<p align="center">
  <img src="matlab/figures/p2_fig8.png" width="460" alt="Figure-8 scenario (E2): 3D trajectory and per-axis tracking">
  <img src="matlab/figures/p2_wp_minsnap.png" width="460" alt="Waypoint mission, min-snap (E1): 3D tour through 5 waypoints vs min-jerk">
</p>

`flatness/polytraj.py` builds waypoint trajectories with per-segment
boundary conditions (waypoints hit to 1e-10, v/a continuous at knots to
machine precision, rest-to-rest ends, analytic derivatives up to snap for
the E3 feedforward); `flatness/trajectories.py` adds the figure-8 and
vertical-orbit generators with at-rest min-jerk entries.

Key finding 6 (E1): **two ways waypoint optimization fails, and the fixes.**
(a) Minimizing only snap with position-only waypoints lets long segments
loiter-then-teleport - position is four integrals from the cost, so slow
huge excursions are nearly free (measured: a -5 m bulge, then a 0.06 s
teleport into the next segment). Fix: pin the corridor tangent velocity
(and zero acceleration) at every intermediate waypoint. (b) A monolithic
KKT solve in raw monomial time spans 10+ orders of magnitude and returns
the same pathology even with correct constraints; the robust form is a
normalized-time per-segment solve (degree 7, min-snap interior via a 2x2
null-space minimization). Also measured: at 0.5 m/s the frozen-gain stack
corner-cuts 90-degree waypoint turns (curvature-limited: radius ~
v^2/a_max = 0.16 m) - an honest boundary that motivates the E3 feedforward
and E7 MPC extensions, not a tuning flaw.

## Independent validation (equation-by-equation, per-finding confirmed)

The implementation was validated equation-by-equation against the paper by
independent review — three separate reviewers (dynamics/mixer, flatness map,
double-loop LQR), and every finding was independently re-verified with its
own numerical experiments before we acted on it.
**Verdict: minor divergences — 17 findings, all confirmed, 0 high-severity;
2 medium unintended bugs, found and FIXED:**

1. **Quaternion kinematics (medium, fixed):** the plant used the world-rate
   form `q̇ = ½[0,ω]⊗q` with body-frame rates; Eq. (6)/(8)'s literal
   `q̇ = ½q⊗ω` is the consistent body-rate form. Confirmer's 2 s spin test:
   literal errs 3e-8 rad, old form 1.5 rad. Now literal — spin error 3e-4 rad
   in the crude closed-loop harness (5000× better), and the study-case numbers
   barely move (tilt is only ~2.8° on the circle).
2. **Flatness map equivalence (medium, fixed):** the Gram–Schmidt q_d
   differed from the paper's `q_pd ⊗ q_zd` by a twist about the thrust axis,
   growing quadratically with tilt (0.028° on the study circle, 2° at 45°
   tilt). Replaced with the **literal construction**: shortest-arc tilt
   quaternion (z-component exactly 0 to machine precision — Eq. 19's "4th
   element" — measured −5.6e-17) composed with the yaw quaternion (Eq. 20/21).
   This also fixed the degeneracy guard, which fired on the wrong case (thrust
   ∥ heading produced NaN or silently dropped yaw at 90° bank; the true
   degeneracy is only thrust = −ẑ).
3. Cosmetic: the trajectory's "counter-clockwise" docstring label corrected to
   clockwise (geometric cross-product check).

The remaining 14 confirmed findings are the documented assumptions in the
table above (paper gives no numbers — we name every choice). What the
validation could **not** check (PDF extraction lost the math): Eq. (9)'s
per-motor signs/rotor config, exact Eq. (15) rows beyond the thrust vector,
Eq. (21)'s multiplication order in the original typeset, and the Eq. (17)
error-state convention — each judged structurally from prose and consistent
with the paper's cited sources. Full report: the validation run's artifacts
on the branch.

---

# Part IV — Interactive simulators & recordings

## Interactive sims (desktop launchers)

| Launcher | What it is | Keys |
|---|---|---|
| `PyBullet SIM - INTERACTIVE (click me).bat` | gym-pybullet-drones, 24 kHz, the paper's 3 tests **pinned at one point** (MuJoCo-parity attitude plant), color-coded drone, orbit camera, live HUD + reference triad | **X/Z/B** = STEP/SINE/FLIP · I/K/J/L = pitch/roll · U/O = yaw · SPACE hover · M mode · T reset · ESC quit |
| `DRONE SIM - INTERACTIVE (click me).bat` | MuJoCo app: city environment, collisions, missions, wind/noise | X/Z/B tests · WASD flight · see [`sim/interactive/README.md`](sim/interactive/README.md) |
| `FLY IT YOURSELF (keyboard controls).bat` | `pysitl` browser ground station (real flight-stack) | Arm + modes in the browser UI |

*(Flight keys moved off WASD in the PyBullet app: WASD/arrows are PyBullet's own camera
controls — they collided with flying.)*

## Recordings — every result, animated

All 20 recordings, organized by paper and experiment. Every one regenerates from a
command (`python make_recordings.py`, `python -m sim.mujoco.record`,
`python -m flatness.record_mujoco`) and **is** the benchmark run — recorded metrics
come out bit-identical to the plain runs. Per-recording metrics and commentary:
[`RECORDINGS.md`](RECORDINGS.md).

### Paper 1 — attitude benchmarks, in-engine MuJoCo renders
*(color-coded drone chasing the RGB reference triad, live quaternion error)*

| STEP — 1 rad staggered steps | SINE — 0.5 rad @ 1 rad/s, ±0.1 noise | FLIP — 360°, no gimbal lock |
|---|---|---|
| <img src="results/recordings/mujoco_step_RENDER.gif" width="270" alt="MuJoCo in-engine: step test"> | <img src="results/recordings/mujoco_sine_RENDER.gif" width="270" alt="MuJoCo in-engine: sine test"> | <img src="results/recordings/mujoco_flip_RENDER.gif" width="270" alt="MuJoCo in-engine: 360° flip"> |

How to read them: **red front arms / blue rear** = the drone; the dashed **RGB triad** = the
paper's reference attitude — perfect tracking means the arms align with the triad. The
attitude-only controller drifts by design, so the render puppet holds the vehicle at a
fixed stage point while its live attitude and body rates play out; the drift itself is
visible in the path-trace GIFs below (and the CSV logs). Recorded runs are
**bit-identical** to the benchmark runs.

- **STEP** — 1 rad steps staggered on roll/pitch/yaw (t = 1, 5, 9 s): each axis settles
  in ≈ 2 s with essentially no overshoot (paper Figs. 3–4); torque briefly saturates at
  each step onset, then stays linear.
- **SINE** — 0.5 rad amplitude at 1 rad/s on all axes under the paper's ±0.1 noise:
  measured lag ≈ 0.4–0.5 s, matching the paper's "about 0.5 second" (paper Figs. 5–6).
- **FLIP** — 0→2π ramp with shortest-path correction disabled: the flip **completes on
  MuJoCo contact physics** and φ settles in ≈ 2.9 s; raw q₀, q₁ stay smooth through the
  full rotation — no gimbal-lock artifact (paper Figs. 7–8).

### Paper 1 — attitude benchmarks, path traces (two engines)

| STEP | SINE | FLIP |
|---|---|---|
| <img src="results/recordings/gym_pybullet_step.gif" width="270" alt="gym-pybullet-drones: step"> | <img src="results/recordings/gym_pybullet_sine.gif" width="270" alt="gym-pybullet-drones: sine"> | <img src="results/recordings/gym_pybullet_flip.gif" width="270" alt="gym-pybullet-drones: flip"> |
| <img src="results/recordings/mujoco_step.gif" width="270" alt="MuJoCo: step path trace"> | <img src="results/recordings/mujoco_sine.gif" width="270" alt="MuJoCo: sine path trace"> | <img src="results/recordings/mujoco_flip.gif" width="270" alt="MuJoCo: flip path trace"> |
| <img src="results/recordings/gym_step.gif" width="270" alt="gym-pybullet-drones: step (6-DOF stack)"> |  |  |

Each trace animates the logged states of the benchmark run (reference triad vs the
drone's frame over time):

- **step — gym-pybullet-drones vs MuJoCo**: RMS tracking error **17.68° vs 17.71°**
  (max 70.7° / 71.2°) — **0.03° cross-engine agreement**, the strongest evidence the
  loop belongs to the controller, not the engine.
- **sine — both engines**: RMS **17.52° / 17.61°**, max ≈ 41°. The ≈ 17.5° RMS floor is
  the paper's own sensor noise (±0.1 on quaternion components ≈ ±11° of measured
  attitude), not controller error.
- **flip — both engines**: MuJoCo completes (settles ≈ 2.9 s); gym-pybullet enters a
  limit cycle near φ ≈ 2.1 rad and unwinds — the documented engine-dependent
  knife-edge of the sampled fast pole (Key finding 2), not a tuning flaw.
- **gym_step** — the same step test flown through the **6-DOF PX4-style flight stack**
  (`pysitl`: rotors, mixer, scheduler, arming/failsafes) with the controller unmodified.

### Paper 2 — study case: circle, radius 1 m @ 1 m altitude

| Ideal Eq. (8) model | MuJoCo contact physics (in-engine) | MuJoCo contact physics (path trace) |
|---|---|---|
| <img src="results/recordings/flatness_ideal_circle.gif" width="270" alt="Flatness circle, ideal model"> | <img src="results/recordings/mujoco_flatness_RENDER.gif" width="270" alt="Flatness circle, MuJoCo in-engine"> | <img src="results/recordings/flatness_mujoco_circle.gif" width="270" alt="Flatness circle, MuJoCo path trace"> |
| <img src="results/recordings/mujoco_flatness_circle_RENDER.gif" width="380" alt="Flatness circle, MuJoCo in-engine with reference ring" colspan="3"> |  |  |

What each shows:

- **Ideal Eq. (8) model** — the study case on the paper's own equations (RK4 @ 1 kHz):
  reference vs actual position/attitude animated from the logged states. Radial
  tracking **RMS 2.7 cm** on the 1 m circle.
- **MuJoCo in-engine** — recorded straight off the MuJoCo renderer: **solid drone =
  actual**, **RGB axes = the flatness map's desired attitude q_d**, **yellow ball =
  the reference point on the path right now**, orange ring = the 1 m circle at 1 m
  altitude. Radial RMS **2.8 cm**, zero rotor saturation; the recorded run's metrics
  are bit-identical to the plain benchmark.
- **MuJoCo path trace** — the same run animated from its log: X/Y/Z vs reference
  (the paper's Figs. 4–7 reproduced) — **2.8 cm** radial, matching the ideal plant
  to a millimetre-level, confirming the flatness map does the work, not the integrator.
- **Interactive-app circle** — the same circle mission flown **live in the interactive
  MuJoCo city app** (keyboard-controllable simulator, U key) with its reference ring.

### Paper 2 — extensions (E1/E2): new flat-output paths, same frozen controller

In-engine MuJoCo renders:

| Figure-8 | Waypoint mission (min-snap) |
|---|---|
| <img src="results/recordings/mujoco_flatness_figure8_RENDER.gif" width="380" alt="Figure-8, MuJoCo in-engine"> | <img src="results/recordings/mujoco_flatness_waypoints_RENDER.gif" width="380" alt="Waypoints, MuJoCo in-engine"> |

Path traces (ideal Eq. (8) model):

| Figure-8 (E2) | Vertical orbit (E2) |
|---|---|
| <img src="results/recordings/flatness_figure8.gif" width="380" alt="Figure-8 path trace"> | <img src="results/recordings/flatness_orbit.gif" width="380" alt="Vertical orbit path trace"> |

| Waypoints — min-snap (E1) | Waypoints — min-jerk (A/B control) |
|---|---|
| <img src="results/recordings/flatness_waypoints_minsnap.gif" width="380" alt="Waypoints min-snap"> | <img src="results/recordings/flatness_waypoints_minjerk.gif" width="380" alt="Waypoints min-jerk"> |

What each shows:

- **In-engine renders** — the E2 figure-8 and the E1 min-snap waypoint tour flown on
  MuJoCo contact physics with the same frozen controller. Markers: orange = figure-8
  path, red/blue spheres = waypoints, small amber = the min-snap path between them.
  Metrics bit-identical to the benchmark runs.
- **figure-8 (E2)** — Lissajous path (A = 1.5/B = 1) with the yaw-follow option:
  3D RMS **4.7 cm** on the ideal model and **identical 4.7 cm** on MuJoCo contact
  physics.
- **vertical orbit (E2)** — cone-sweep climb-and-dive path: **3.9 cm** 3D RMS, the
  same accuracy as the flat circle — the flatness map generalizes beyond planar paths.
- **waypoints min-snap vs min-jerk (E1)** — identical 5-waypoint tour and times, A/B:
  **6.1 cm vs 8.1 cm** 3D RMS. Min-snap wins by 25% because snap (the 4th derivative)
  drives the torques — the flatness argument, measured. Waypoints are hit to 1e-10
  with velocity/acceleration continuous at every knot.

---

# Installation

**Prerequisites:** Python 3.10+ (3.10 tested) and Git. MATLAB R2024b is optional —
only needed to regenerate the reports in `matlab/`. Gazebo and ArduPilot SITL run
under WSL2 ([`sim/README.md`](sim/README.md)).

```bash
git clone https://github.com/bodapatisaikrishna/Drones_S5_CD12_Full_Quaternion_Based_Attitude_control_for_a_Quadrotor.git
cd Drones_S5_CD12_Full_Quaternion_Based_Attitude_control_for_a_Quadrotor

python -m venv .venv
source .venv/bin/activate                 # Windows: .venv\Scripts\activate

# core stack: quaternion controller, 6-DOF sim, flatness ideal model, tests
pip install -r requirements.txt
pip install -e .

# optional backends — install only what you will run:
pip install mujoco                        # sim/mujoco + python -m flatness.run_mujoco
pip install pybullet gym-pybullet-drones  # sim/gym_pybullet
pip install keyboard                      # sim/interactive keyboard apps
pip install -r sim/ardupilot/requirements.txt   # ArduPilot SITL bridge (pymavlink)
# report/PDF build tooling (optional): pip install reportlab pypdf
```

Sanity check: `pytest tests/ -v` collects **85 tests — 83 pass, 2 stack-gated**
(Gazebo/ArduPilot need WSL2).

# Running everything

```bash
pytest tests/ -v                          # 85 collected: 83 pass, 2 stack-gated

# ---- Paper 1: base reproduction ------------------------------------------
python -m quat_sitl.simulator --scenario step --duration 15 --seed 0 --noise 0.1
python -m quat_sitl.visualize3d           # animated 3D replay

# ---- 6-DOF extension ------------------------------------------------------
python -m pysitl.run --gcs                # browser ground station → 127.0.0.1:8765
python -m pysitl.run --mode auto_flip --duration 5 --log

# ---- Four-simulator deployment --------------------------------------------
python -m sim.mujoco.run --scenario flip --gui
python -m sim.gym_pybullet.run --scenario step
python -m sim.compare                     # results/comparison.md (all stacks)

# ---- Paper 2: path planning -----------------------------------------------
python -m flatness.run_paper              # ideal Eq.(8) model → results/flatness/
python -m flatness.run_mujoco             # MuJoCo contact physics
python -m flatness.run_mujoco --gui       # live viewer
python -m flatness.record_mujoco          # in-engine recording GIF

# ---- Recordings -------------------------------------------------------------
python make_recordings.py                 # attitude-test + circle animations
python -m sim.mujoco.record               # in-engine attitude-test renders
```

## Repository contents

| Path | Contents |
|---|---|
| [`quat_sitl/`](quat_sitl/) | **Paper 1 reproduction** — quaternion algebra (eqs. 1–16), plant (17–18), P² controller (19–21), noise model, three benchmarks, plots, 3D replay |
| [`pysitl/`](pysitl/) | **6-DOF PX4-style extension** — rotors, mixer, scheduler, arming/failsafes, autopilot, browser ground station |
| [`sim/`](sim/) | **Four-simulator deployment** (MuJoCo, gym-pybullet-drones, Gazebo/WSL2, ArduPilot SITL/WSL2) + interactive apps + recorders |
| [`flatness/`](flatness/) | **Paper 2: path planning** — Eq. (8) model + Eq. (9) mixer, flat outputs (13), flatness map (15, 19–21), double-loop LQR (17–18, 22–23), study case on ideal + MuJoCo, in-engine recorder |
| [`make_recordings.py`](make_recordings.py) · [`RECORDINGS.md`](RECORDINGS.md) | Recording generators · gallery of every result, animated |
| [`scenarios/`](scenarios/) · [`tests/`](tests/) · [`docs/figures/`](docs/figures/) | Runnable benchmarks · 85 tests · result figures |
| [`docs/PROJECT_REPORT.pdf`](docs/PROJECT_REPORT.pdf) · [`REPORT.md`](REPORT.md) · [`FRP.md`](FRP.md) | Project report (full, typed equations, both papers) · paper-1 technical report · four-stack deployment rationale |
| [`matlab/`](matlab/) · [`docs/NOTES_Flatness_Path_Planning.pdf`](docs/NOTES_Flatness_Path_Planning.pdf) | MATLAB Live Scripts — `project_report.mlx` (both papers, runnable) and `quadrotor_math_derivations.mlx` (full derivations, block-verified against source: [`MLX_VERIFICATION.md`](matlab/MLX_VERIFICATION.md)) · path-planning notes |
| [`docs/REPO_AUDIT.md`](docs/REPO_AUDIT.md) | Repository audit log — corrections, de-duplication, verification evidence |
| [deploy/](deploy/) · [`deploy/simulator.html`](deploy/simulator.html) | Vercel pages · standalone zero-dependency browser simulator |

## Testing

| Suite | Tests | Covers |
|---|---|---|
| `test_quaternion.py` | 7 | Quaternion algebra, DCM round-trips, closed-loop sign consistency |
| `test_sitl.py` | 20 | 6-DOF stack: frames, eq. 18 parity, hover, mixer + desaturation, scheduler, arming/failsafes |
| `test_sim_bridge.py` | 8 | The sim/ bridge: conjugate conventions, priority mixer, altitude hold, scenario parity |
| `test_sim_adapters.py` | 6 | MuJoCo + gym-pybullet adapter smoke tests, actuation calibration, model integrity |
| `test_sim_interactive.py` | 15 | Interactive apps: key handling, position pin, paper tests from any mode |
| `test_bridge_mavlink.py` | 1 | ArduPilot bridge end-to-end against the MAVLink mock-SITL |
| **`test_flatness.py`** | **13** | **Paper 2: flatness-map exactness (1e-10), feedforward consistency, mixer round-trip + desaturation, LQR pole stability, closed-loop circle tracking < 5 cm radial RMS on both plants** |
| **`test_flatness_extensions.py`** | **15** | **E1/E2: waypoint hits to 1e-10, C2 knots, rest ends, min-snap < min-jerk objective, closed-loop figure-8 / orbit / waypoint mission < 10 cm 3D RMS, yaw-follow tangent** |
| `test_sim_interactive.py` (PyBullet) | (incl. above) | paper tests pinned: step 57.4°, sine 30.5°, flip full inversion; position hold ≤ 4 µm |

Several are regressions for bugs found and fixed during development (findings 1–5 above).

## Future work

- DONE (Phase A): trajectory library — figure-8, vertical orbit, min-snap/min-jerk
  waypoint polynomials (E1/E2 above), 15 new tests, benchmark table.
- Port the flatness stack to PyBullet and the interactive sims (waypoint/circle missions
  driven by the paper-2 outer loop, attitude core unchanged).
- Phase B: E3 flatness feedforward (omega_d/tau_ff from the now-available jerk/snap)
  and E4 geometric SE(3) controller benchmark; Phase C: A*/RRT* planning through the
  MuJoCo city. Full plan: docs/EXTENSIONS_Proposal.pdf.
- Wind/disturbance robustness runs for the path planner (noise row exists; wind does not).
- MAVLink bridge so the same unmodified laws drive hardware-in-the-loop SITL.

## References

[1] E. Fresk and G. Nikolakopoulos, "Full Quaternion Based Attitude Control for a
Quadrotor," in *Proc. European Control Conference (ECC)*, Zürich, Switzerland, 2013,
pp. 3864–3869. doi: [10.23919/ECC.2013.6669617](https://doi.org/10.23919/ECC.2013.6669617)

[2] K. Choutri, M. Lagha, L. Dala, and M. Lipatov, "Quadrotors Trajectory Tracking
using a Differential Flatness–Quaternion based Approach," in *Proc. 7th International
Conference on Modeling, Simulation, and Applied Optimization (ICMSAO)*, Sharjah, UAE,
2017, pp. 1–6. doi: [10.1109/ICMSAO.2017.7934901](https://doi.org/10.1109/ICMSAO.2017.7934901)

[3] A. Chovancová, T. Fico, P. Hubinský, and F. Duchoň, "Comparison of various
quaternion-based control methods applied to quadrotor with disturbance observer and
position estimator," *Robotics and Autonomous Systems*, vol. 79, pp. 104–116, 2016.
doi: [10.1016/j.robot.2016.01.011](https://doi.org/10.1016/j.robot.2016.01.011)
(paper 2's quaternion-cascade source)

[4] I. D. Cowling, J. F. Whidborne, and A. K. Cooke, "Optimal Trajectory Planning and
LQR Control for a Quadrotor UAV," in *Proc. UKACC International Conference on
Control 2006*, Glasgow, UK, paper 211.

[5] A. Tayebi and S. McGilvray, "Attitude Stabilization of a VTOL Quadrotor Aircraft,"
*IEEE Transactions on Control Systems Technology*, vol. 14, no. 3, pp. 562–571, 2006.
doi: [10.1109/TCST.2006.872519](https://doi.org/10.1109/TCST.2006.872519)

[6] PX4 Development Team, *PX4 Autopilot Documentation* —
[docs.px4.io](https://docs.px4.io/) (architecture reference for `pysitl`).

[7] Simulation stacks: MuJoCo — [mujoco.org](https://mujoco.org) ·
gym-pybullet-drones —
[github.com/utiasDSL/gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) ·
Gazebo — [gazebosim.org](https://gazebosim.org) ·
ArduPilot SITL — [ardupilot.org](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html).

[8] This repository — equation-to-function maps above; full methodology in
[`REPORT.md`](REPORT.md); recordings in [`RECORDINGS.md`](RECORDINGS.md).

[9] This repository, MATLAB Live Script reports:
[`matlab/project_report.mlx`](matlab/project_report.mlx) — the runnable report
reproducing both papers end-to-end (PDF export:
[`matlab/published/project_report_final.pdf`](matlab/published/project_report_final.pdf));
[`matlab/quadrotor_math_derivations.mlx`](matlab/quadrotor_math_derivations.mlx) —
the complete equation-by-equation derivations of paper 2, verified block-by-block
against the source document
([`matlab/MLX_VERIFICATION.md`](matlab/MLX_VERIFICATION.md)).
