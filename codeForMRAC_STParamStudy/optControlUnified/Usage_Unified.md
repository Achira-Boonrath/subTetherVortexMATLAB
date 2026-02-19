# Unified Optimal Control Framework

This directory now contains a unified framework for solving single-shooting optimal control problems, replacing the multiple standalone scripts.

## Core Script
`optControl_singleShoot_Unified.m`
This is the main driver. It takes a `problem` structure as input and handles:
- Symbolic generation of Hamiltonian and Costate equations.
- Shooting method (finding initial costates `lambda0`).
- Integration and results handling.

## Wrapper Scripts
Use these scripts to run the specific problems:

### 1. 2-Body Orbit Transfer (Unconstrained)
**Script:** `run_2body.m`
**Equivalent to:** `optControl_singleShoot_2body.m`
- Solves a simple orbit transfer using CWH dynamics.
- Uses symbolic differentiation to generate costate dynamics automatically.

### 2. 2-Body Orbit Transfer (Constrained Thrust)
**Script:** `run_2body_constrained.m`
**Equivalent to:** `optControl_singleShoot_2body_constrainedT.m`
- Solves an orbit transfer with bang-bang control (Throttle 0 or 1).
- Uses a **custom ODE function** to handle the switching logic during integration.
- Demonstrates how to override the automated symbolic ODEs when complex logic is required.

### 3. Attitude Reorientation
**Script:** `run_att.m`
**Equivalent to:** `optControl_singleShoot_att.m`
- Solves a coupled position + attitude problem (13 states).
- Uses symbolic differentiation for the complex quaternion kinematics and rotational dynamics.

## How to Create a New Problem
1. Create a `run_myproblem.m` script.
2. Define `problem.x0`, `problem.xf`, `problem.tf`.
3. Define `problem.setupSyms` handle. This function must:
   - Define symbolic variables for State `X`, Control `U`, Costates `Lvec`.
   - Define Dynamics `f`.
   - Return them in the expected order.
4. Call `optControl_singleShoot_Unified(problem)`.
