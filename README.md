
<div align="center">

![Banner](res/images/banner.png)

# 2D Robot Path Planning using Trajectory Optimization (MPC)

**A continuous, optimization-based approach to 2D robot path planning**

**Academic Project** • Numerical Optimization • Data Science and AI • ETF Sarajevo

![C++](https://img.shields.io/badge/C%2B%2B-20-blue?logo=c%2B%2B)
![natID](https://img.shields.io/badge/natID-Framework-lightgrey)
![License](https://img.shields.io/badge/License-MIT-green)

</div>

## 📚 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Algorithm](#algorithm)
- [Project Structure](#project-structure)
- [Build & Run](#build--run)
- [NatID GUI Integration](#natid-gui-integration)
- [Development Notes](#development-notes)
- [Team](#team)
- [License & CTA](#license--cta)

## 🧭 Overview

Course: Numerical Optimization

Professor: Prof. Dr. Izudin Džafić

Student: Emin Hadžiabdić (19960)

This project formulates 2D robot path planning as a continuous trajectory optimization (MPC) problem. The solver produces a time-indexed trajectory that respects dynamics, inequality constraints (speed, obstacle clearance), and produces smooth control sequences.

## ✨ Features
- **Continuous MPC formulation** with quadratic cost and constraint handling
- **Sparse constraint assembly** for efficient long-horizon problems
- **Sequential Quadratic Programming (SQP)** solver loop for non-convex obstacle avoidance
- **Interactive NatID-based 2D GUI** to visualize and manipulate obstacles in real time

## 📐 Algorithm

- Objective: Quadratic cost penalizing distance-to-goal and control effort.
- Dynamics: discrete-time kinematic chain enforced as equality constraints.
- Constraints: box constraints on speed/acceleration and nonlinear obstacle-clearance inequalities handled via SQP.

Mathematical notation and derivations are kept in the `docs/` folder (see `docs/algorithm.md`).

## 🗂 Project Structure

```txt
./                        # repo root
	├─ src/                 # C++ sources (solver, matrix types, natID GUI hooks)
	├─ include/             # headers for Dense/Sparse matrix utilities
	├─ examples/            # scenario configs and sample runs
	├─ docs/                # algorithm derivations and design notes
	├─ assets/              # images used in README and GUI
	├─ CMakeLists.txt       # build configuration for native platforms
	└─ project_proposal.md  # original project proposal (academic context)
```

## 🛠 Build & Run

Windows (recommended for natID integration):

```powershell
mkdir build
cd build
cmake -G "Visual Studio 17 2022" ..
cmake --build . --config Release
```

Cross-platform (CMake):

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

Run an example scenario from `examples/` after building. See `docs/run.md` for platform-specific notes.

## 🖥 NatID GUI Integration

This project uses the natID framework for the interactive 2D visualization. See `docs/natid_integration.md` for development notes and the official natID repo: https://github.com/idzafic/natID.git

## 📣 License & CTA

MIT — See LICENSE. Star this repo if you found it helpful!


