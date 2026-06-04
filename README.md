<div align="center">

![Project Banner](res/images/logo.png)

# 2D Robot Path Planning using Trajectory Optimization (MPC)

**A continuous, optimization-based approach to 2D robot path planning using Model Predictive Control and SQP**

Numerical Optimization • Faculty of Electrical Engineering (ETF) • University of Sarajevo

[![C++](https://img.shields.io/badge/C%2B%2B-17-blue?logo=c%2B%2B&logoColor=white)](https://en.cppreference.com/)
[![natID](https://img.shields.io/badge/natID-Framework-lightgrey)](https://github.com/idzafic/natID.git)
[![CMake](https://img.shields.io/badge/CMake-3.18-green?logo=cmake&logoColor=white)](https://cmake.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

</div>

---

## 📚 Table of Contents

- [🔍 Overview](#-overview)
- [✨ Features](#-features)
- [⚙️ Algorithm Formulation](#️-algorithm-formulation)
- [🛣️ Simulation Scenarios](#️-simulation-scenarios)
- [🏗️ Architecture & Project Structure](#️-architecture--project-structure)
- [🔨 Build & Run](#-build--run)
- [👥 Team](#-team)

---

## 🔍 Overview

- **Course:** Numerical Optimization
- **Professor:** Prof. Dr. Izudin Džafić
- **Academic Year:** 2025/2026
- **Student:** Emin Hadžiabdić (19960)

This project formulates 2D robot path planning as a continuous trajectory optimization (Model Predictive Control) problem. The solver generates a smooth, time-indexed trajectory that minimizes deviation from a reference lane, reduces steering/acceleration efforts, and respects dynamic limits and obstacle-avoidance clearance.

---

## ✨ Features

### Core Functionality

- **Continuous MPC Formulation:** Minimizes a quadratic cost function tracking reference states $(x, y, \psi, v)$ while penalizing steering angles and acceleration inputs.
- **Sequential Quadratic Programming (SQP):** Solves non-convex obstacle avoidance problems by iteratively linearizing clearance constraints and solving sparse Quadratic Programming (QP) subproblems.
- **Stale Warm-Start Detection:** Resets nominal trajectories if vehicle telemetry drifts significantly ($> 2.0\text{m}$) from the previous solver step's warm start, preventing diverging basins of attraction.
- **Plateau Freezing (`freezeAtPeak`):** Correctly plateaus reference trajectories when the robot transitions to flat segments after polynomial peaks.
- **Dynamic Lookahead Window:** Adaptively configures the forward planning horizon per-scenario (e.g. $15.0\text{m}$ for lane changes, $20.0\text{m}$ for S-Curves).

### User Experience

- **Interactive 2D GUI (natID framework):** Live rendering of reference paths, planned trajectories, obstacles, and vehicle telemetry.
- **Real-Time Obstacle Interaction:** Move and resize circular obstacles dynamically to trigger active replanning.
- **Simulation Control Panel:** Step forward/backward through iterations, adjust playback speed, reset, and toggle between planning scenarios.

---

## ⚙️ Algorithm Formulation

### Mathematical Model

The MPC solver optimizes the following discrete-time optimal control problem over a horizon $N$:

```text
Minimize:
  J = ∑_{k=0}^{N} (z_k - z_ref,k)^T Q (z_k - z_ref,k) + ∑_{k=0}^{N-1} u_k^T R u_k

Subject to:
  x_{k+1} = x_k + v_k * cos(ψ_k) * dt            (Kinematic X Dynamics)
  y_{k+1} = y_k + v_k * sin(ψ_k) * dt            (Kinematic Y Dynamics)
  ψ_{k+1} = ψ_k + (v_k / L_f) * δ_k * dt         (Kinematic Heading Dynamics)
  v_{k+1} = v_k + a_k * dt                        (Velocity Dynamics)
  -δ_max <= δ_k <= δ_max                         (Steering Limits)
  -a_max <= a_k <= a_max                         (Acceleration Limits)
  (x_k - x_obs)^2 + (y_k - y_obs)^2 >= r_clear^2 (Obstacle Avoidance Clearance)
```

### Use Case Description
Autonomous navigation in structured lanes or cluttered corridors. The robot shifts lanes to bypass static obstacles, then smoothly rejoins the lane centerline while keeping control actions within vehicle limits.

---

## 🛣️ Simulation Scenarios

- **Straight Line:** Simplest track keeping the reference path flat at $y=0$ with no obstacles.
- **Lane Change:** A polynomial reference path rising smoothly from $y=0$ to $y=2.0$ over $25\text{m}$ and then plateauing using `freezeAtPeak = true`. Two obstacles at $x=15\text{m}$ and $x=35\text{m}$ force the robot to weave between lanes.
- **S-Curve:** A smooth arch-like reference rising to $y=2.0$ at $x=40\text{m}$ and descending back to $y=0$ at $x=60\text{m}$ using `maxLookahead = 20.0` and no obstacles.

---

## 🏗️ Architecture & Project Structure

The codebase is split into mathematical optimization logic and native UI visualization:

```txt
RobotTrajectoryOpt/
├── src/                          # Source code files
│   ├── main.cpp                  # Application entry point and natID view launcher
│   ├── Application.h             # Application configuration loader
│   ├── MainWindow.h              # MainWindow containing the top menu bar
│   ├── MainView.h                # Main GUI layout orchestrating solver threads and play controls
│   ├── MpcLayout.h               # Indexing layout mapping states, controls, and slack variables
│   ├── MpcCost.h                 # Trajectory tracking objective function (Q/R weights)
│   ├── MpcConstraints.h          # Dynamics and obstacle clearance constraints
│   ├── MpcEngine.h               # High-level solver engine managing warm-starts and diagnostics
│   ├── MpcSqp.h                  # Sequential Quadratic Programming loop for non-convex optimization
│   ├── MpcKkt.h                  # Karush-Kuhn-Tucker (KKT) sparse system assembler
│   ├── MpcKktSolver.h            # Linear solver for sparse symmetric KKT matrices
│   ├── MpcObstacle.h             # Dynamic circular obstacle representation
│   ├── MpcPathCanvas.h           # Canvas rendering the path, vehicle, and obstacles
│   ├── MpcActuationCanvas.h      # Canvas plotting steering and acceleration commands
│   ├── MpcSidebarView.h          # Telemetry and solver convergence sidebar panel
│   ├── MpcToolBar.h              # Simulation playback toolbar (Play, Pause, Step)
│   ├── MpcVizAdapter.h           # Adapter translating state trajectories to rendering structures
│   ├── MpcSettingsPopup.h        # Configuration panel for parameters and boundaries
│   ├── DialogSettings.h          # Dialog wrapper around config settings
│   └── MpcSolverStub.h           # Initial linear planning stub solver
├── res/                          # Application resources
│   ├── DevRes.xml                # Development resource XML catalog
│   ├── main.xml                  # Main resource XML catalog
│   ├── images/                   # Graphics assets
│   │   └── logo.png              # Project header banner image
│   ├── tr/                       # Translation dictionaries (EN, BA, DE, ES, FR, JP)
│   └── appIcon/                  # OS-specific application bundle icons
├── build/                        # Build files and compiled binaries (generated)
├── docs/                         # Derivations and design notes
├── CMakeLists.txt                # Root CMake build configuration
├── RobotTrajectoryOpt.cmake      # Target compiler and linker configuration
├── LICENSE                       # MIT License
└── README.md                     # This file
```

---

## 🔨 Build & Run

### Windows (Recommended for natID integration)
Building requires Microsoft Visual Studio and the `natID.SDK` package set up in your user directory:

```powershell
mkdir build
cd build
cmake -G "Visual Studio 18 2026" ..
cmake --build . --config Release
```

### Cross-Platform (CMake)
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

---

## 👥 Contributors

- **Emin Hadžiabdić** - Lead Developer & Optimization Engineer (Faculty of Electrical Engineering, University of Sarajevo)
- **Prof. Dr. Izudin Džafić** - Academic Mentor & natID Framework Creator

---

⭐ Star this repo if you found it helpful!
