# Nonlinear Backstepping Control for Quadrotor-Based Map Scanning

![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-orange)
![Simulink](https://img.shields.io/badge/Simulink-model-blue)
![Control](https://img.shields.io/badge/control-nonlinear%20backstepping-informational)

A MATLAB/Simulink implementation of a **nonlinear backstepping controller** that flies a quadrotor
over a rectangular area in a **zig-zag (boustrophedon) scanning pattern** — the kind of path used for
aerial mapping with a camera, LiDAR or HD-map sensor — and then returns it to its starting point.

Stability of every control loop is derived with Lyapunov functions, and the full closed-loop system
(rigid-body dynamics, rotor dynamics, position/altitude/attitude/heading controllers) is simulated in Simulink.

> Course project, Engineering Physics — Institut Teknologi Bandung.
> Full write-up (Indonesian): [`docs/Report.pdf`](docs/Report.pdf)

<p align="center">
  <img src="docs/images/results/tracking_animation.gif" width="620" alt="Quadrotor tracking the zig-zag scanning path">
</p>

---

## Contents

- [Overview](#overview)
- [System architecture](#system-architecture)
- [Simulink model](#simulink-model)
- [Results](#results)
- [Repository structure](#repository-structure)
- [Getting started](#getting-started)
- [Parameters](#parameters)
- [References](#references)

## Overview

Collecting geospatial data manually is slow and risky in places such as mines, disaster areas or large
plantations. A quadrotor can cover these areas quickly, but it needs a controller that keeps it stable
and accurate while it follows the scanning path.

**Mission**

1. Take off from the initial position `(x₀, y₀)` and climb to the scanning altitude.
2. Enter *scan mode*: sweep a `panjang × lebar` (length × width) rectangle in a zig-zag pattern,
   stepping `belokan` metres sideways after every pass, at a constant ground speed.
3. When the whole area is covered, fly back to the start point and hover.

**Assumptions:** the simulation models only the vehicle motion (sensor behavior is not simulated), in an
obstacle-free environment without significant external disturbances.

<p align="center">
  <img src="docs/images/report/01_free_body_diagram.png" width="360" alt="Quadrotor free-body diagram">
  <br><em>Quadrotor free-body diagram, NED earth frame and body frame.</em>
</p>

## System architecture

The controller is a cascade: an **outer position loop** turns the scanning path into desired roll/pitch
angles, and **inner backstepping loops** stabilize altitude, attitude and heading. Their virtual inputs
`U1…U4` are mapped to rotor speeds, passed through a first-order rotor model, and applied to the nonlinear
6-DOF quadrotor dynamics.

```mermaid
flowchart LR
    subgraph Guidance
        TRAJ["Zig-zag trajectory<br/>generator<br/>lintasan_drone"]
    end

    subgraph Outer["Outer loop"]
        POS["Position controller<br/>(k1, k2, k3, k4)"]
    end

    subgraph Inner["Inner backstepping loops"]
        ALT["Altitude controller<br/>(c7, c8)"]
        ATT["Attitude & heading controller<br/>roll (c1, c2) · pitch (c3, c4) · yaw (c5, c6)"]
    end

    subgraph Actuation
        MSC["Motor speed<br/>calculation"]
        ROT["Rotor dynamics<br/>0.936 / (0.178 s + 1)"]
        SIC["System input<br/>calculation"]
    end

    PLANT(["Quadrotor dynamics<br/>(Newton–Euler, 12 states)"])

    TRAJ -- "x_d, y_d" --> POS
    POS -- "φ_d, θ_d" --> ATT
    ZD["z_d, ψ_d"] --> ALT
    ZD --> ATT
    ALT -- U1 --> MSC
    ATT -- "U2, U3, U4" --> MSC
    MSC -- "Ω1…Ω4 desired" --> ROT
    ROT -- "Ω1…Ω4 actual" --> SIC
    SIC -- "U1…U4, Ω_r" --> PLANT
    PLANT -- "x, y, z, ẋ, ẏ, ż" --> POS
    PLANT -- "z, ż" --> ALT
    PLANT -- "φ, θ, ψ, rates" --> ATT
```

**Backstepping in one line per loop.** For each channel with tracking error `z₁ = x_d − x` the
Lyapunov function `V₁ = ½z₁²` is augmented with a virtual-control error `z₂ = ẋ − ẋ_d − c₁z₁`,
`V₂ = V₁ + ½z₂²`, and the control input is chosen so that `V̇₂ = −c₁z₁² − c₂z₂² ≤ 0`, which guarantees
stability for any `c₁, c₂ > 0`. The resulting laws (see report, Eq. 40–53):

| Loop | Control input | Gains |
|---|---|---|
| Altitude | `U1 = m / (cos φ cos θ) · (−z₇ + g − ẍ₇d − c₇ẋ₇d + c₇x₈ + c₈z₈)` | `c7, c8` |
| Roll | `U2 = (1/b₁) · (−c₂z₂ + z₁ − x₄x₆a₁ + x₄Ω_r a₂ + ẍ₁d + c₁ẋ₁d − c₁x₂)` | `c1, c2` |
| Pitch | `U3 = (1/b₂) · (−c₄z₄ + z₃ − x₂x₆a₃ − x₂Ω_r a₄ + ẍ₃d + c₃ẋ₃d − c₃x₄)` | `c3, c4` |
| Yaw | `U4 = (1/b₃) · (−c₆z₆ + z₅ − x₂x₄a₅ + ẍ₅d + c₅ẋ₅d − c₅x₆)` | `c5, c6` |
| Position x | `θ_d = −(m/U1) · (ẍ_d + k₁(x_d − x) + k₃(ẋ_d − ẋ))` | `k1, k3` |
| Position y | `φ_d = (m/U1) · (ÿ_d + k₂(y_d − y) + k₄(ẏ_d − ẏ))` | `k2, k4` |

**Gain tuning.** The gains were tuned by trial and error to minimize steady-state error while keeping a
fast response without significant overshoot. For the altitude loop, `scripts/ga_tune_altitude.m` searches
`c7, c8 ∈ [0.1, 10]` with a Genetic Algorithm (population 20, 50 generations) that minimizes the step-response
cost

$$J = t_s + 10 \cdot OS + 100 \cdot e_{ss}$$

where $t_s$ is the 2 % settling time, $OS$ the percent overshoot and $e_{ss}$ the steady-state error.

**Energy estimate.** `scripts/plot_state_response.m` also accumulates a simple energy index by
integrating the squared change of the translational and angular velocities over time.

## Simulink model

<p align="center">
  <img src="docs/images/simulink/top_level.png" alt="Top-level Simulink diagram">
  <br><em>Top level of <code>model/QuadrotorModel.slx</code>.</em>
</p>

<table>
  <tr>
    <td width="50%"><img src="docs/images/simulink/quadrotor_dynamics.png" alt="Quadrotor dynamics"><br><em>Quadrotor dynamics</em></td>
    <td width="50%"><img src="docs/images/simulink/position_controller.png" alt="Position controller"><br><em>Position controller</em></td>
  </tr>
  <tr>
    <td><img src="docs/images/simulink/altitude_controller.png" alt="Altitude controller"><br><em>Altitude controller</em></td>
    <td><img src="docs/images/simulink/attitude_heading_controller.png" alt="Attitude and heading controller"><br><em>Attitude &amp; heading controller</em></td>
  </tr>
  <tr>
    <td><img src="docs/images/simulink/motor_speed_calculation.png" alt="Motor speed calculation"><br><em>Motor speed calculation</em></td>
    <td><img src="docs/images/simulink/system_input_calculation.png" alt="System input calculation"><br><em>System input calculation</em></td>
  </tr>
</table>

<details>
<summary>Individual attitude / heading loops and rotor dynamics</summary>

| Roll controller | Pitch controller |
|---|---|
| ![Roll](docs/images/report/06_roll_controller.png) | ![Pitch](docs/images/report/07_pitch_controller.png) |

| Heading controller | Rotor dynamics |
|---|---|
| ![Heading](docs/images/report/08_heading_controller.png) | ![Rotor dynamics](docs/images/simulink/system_input_calculation_rotor_dynamics.png) |

</details>

## Results

Scan area 50 m × 55 m, 10 m pass spacing, 5 m/s ground speed, 10 m altitude, 120 s simulation.

<table>
  <tr>
    <td width="55%"><img src="docs/images/results/trajectory_3d.png" alt="3D trajectory"></td>
    <td width="45%"><img src="docs/images/results/trajectory_xy.png" alt="Top view"></td>
  </tr>
</table>

The quadrotor follows the whole scanning pattern, reaches the 10 m scanning altitude (within 2 %) after about 6 s, and after
that the altitude error stays below 5 cm. The corners are rounded because the desired path has
sharp 90° turns that a physical vehicle cannot follow instantaneously.

![Position tracking](docs/images/results/position_tracking.png)

![Attitude tracking](docs/images/results/attitude_tracking.png)

![Position error](docs/images/results/position_error.png)

![Control inputs](docs/images/results/control_inputs.png)

**Root-mean-square error**

| State | RMSE |
|---|---|
| x | 7.8285 m |
| y | 3.7936 m |
| z | 1.1712 m |
| φ (roll) | 0.038027 rad |
| θ (pitch) | 0.035950 rad |
| ψ (yaw) | 0.000967 rad |

The position RMSE is computed between desired and actual positions *at the same time instant*, so it is
dominated by the time lag of a trajectory-following system rather than by path deviation — the ±10 m
plateaus in `e_x` are this lag, not an offset from the path. The attitude errors are small, showing that the
inner loops track their references well. The z RMSE is mostly the initial climb from the ground.

<details>
<summary>Original figures from the report</summary>

| Zig-zag path (planner output) | State and input plots |
|---|---|
| ![](docs/images/report/11_zigzag_path_xy.png) | ![](docs/images/report/12_state_input_plots.png) |

| 3D trajectory | 3D animation frame |
|---|---|
| ![](docs/images/report/13_3d_trajectory.png) | ![](docs/images/report/14_3d_animation_frame.png) |

</details>

## Repository structure

```
.
├── main_simulation.mlx            # Main live script: parameters, gains, runs the model, RMSE
├── main_simulation.m              # Same as above as a plain .m file
├── model/
│   └── QuadrotorModel.slx         # Simulink model (dynamics + controllers + trajectory)
├── scripts/
│   ├── zigzag_trajectory.m        # Stand-alone zig-zag path generator with 2D animation
│   ├── ga_tune_altitude.m         # Genetic-algorithm tuning of the altitude gains c7, c8
│   ├── animate_quadrotor_3d.m     # 3D animation of the saved simulation
│   ├── plot_state_response.m      # Animated state / input / energy plots
│   ├── generate_figures.m         # Exports the result figures in docs/images/results
│   └── export_simulink_diagrams.m # Exports the block diagrams in docs/images/simulink
├── data/
│   └── scanning_mode_results.mat  # Saved 120 s simulation of the scanning mission
└── docs/
    ├── Report.pdf                 # Full report (Indonesian)
    └── images/                    # results/, simulink/, report/
```

## Getting started

**Requirements:** MATLAB R2023b (or newer) with Simulink. `ga_tune_altitude.m` also needs the
Global Optimization Toolbox.

```matlab
% 1. Run the full simulation (open the live script, or the .m version)
main_simulation

% 2. Visualise the saved results without re-simulating
run scripts/animate_quadrotor_3d.m
run scripts/plot_state_response.m

% 3. Regenerate the README figures
run scripts/generate_figures.m
run scripts/export_simulink_diagrams.m
```

To change the scanning mission, edit the *Lintasan Drone* section of `main_simulation`
(`panjang`, `lebar`, `belokan`, `kecepatan`, `waktu_simulasi`).

## Parameters

| Quadrotor | Value | Backstepping gains | Value |
|---|---|---|---|
| Mass `m` | 0.650 kg | `c1`, `c3`, `c5` | 2 |
| `Ixx`, `Iyy` | 7.5 × 10⁻³ kg·m² | `c2`, `c4`, `c6` | 1 |
| `Izz` | 1.3 × 10⁻² kg·m² | `c7` | 0.308 |
| Arm length `l` | 0.23 m | `c8` | 3 |
| Rotor inertia `Jr` | 6 × 10⁻⁵ kg·m² | `k1`, `k2` | 0.5 |
| Thrust constant `kf` | 3.13 × 10⁻⁵ | `k3`, `k4` | 1 |
| Moment constant `km` | 7.5 × 10⁻⁷ | | |

| Scanning mission | Value |
|---|---|
| Length `panjang` × width `lebar` | 50 m × 55 m |
| Pass spacing `belokan` | 10 m |
| Ground speed `kecepatan` | 5 m/s |
| Simulation time | 120 s |

## References

1. H. M. N. ElKholy, *Dynamic Modeling and Control of a Quadrotor Using Linear and Nonlinear Approaches*, M.Sc. thesis, 2014.
2. B. B. Rao and S. P. Kothapalli, "Sliding Mode Control Based on Backstepping Approach for an UAV Type-Quadrotor."
3. Ferdinandus and I. P. Putrawiyanta, "Pemanfaatan Teknologi Unmanned Aerial Vehicle untuk …," *Jurnal Teknik Pertambangan (JTP)*, pp. 15–23, 2023.
4. I. P. H. Prayogo, F. J. Manoppo and L. I. R. Lefrandt, "Pemanfaatan Teknologi UAV Quadcopter dalam Pemetaan Digital (Fotogrametri) Menggunakan Kerangka Ground Control Point (GCP)," *Jurnal Ilmiah Media Engineering*, 2024.

## Author

**Izma Alhazmi Herdian** — Engineering Physics, Institut Teknologi Bandung
