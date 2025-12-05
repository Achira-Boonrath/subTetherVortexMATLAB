# Sub-Tether Debris Removal with MRAC Elongation Control

This folder contains MATLAB scripts and data files for simulating and analyzing a sub-tethered space debris removal system using Model Reference Adaptive Control (MRAC).

## Main Project File

### `nehaST_MRAC_script.m`
This is the primary entry point for running the simulation routines. While defined as a function accepting input parameters (`wIn`), it is often configured internally for standalone execution.

#### Detailed Workflow
1.  **Parameter Configuration**:
    -   The script selects between different parameter sets (`wIn`) defining sub-tether stiffness, damping, and length.
    -   It initializes the **Model Reference Adaptive Control (MRAC)** parameters based on the selected configuration.

2.  **State Initialization (`s0`)**:
    -   Constructs the initial state vector `s0` for the ODE solver, comprising:
        -   **Chaser State** (Indices 1-13): Position, Velocity, Quaternion, Angular Velocity.
        -   **Debris State** (Indices 14-26): Position, Velocity, Quaternion, Angular Velocity.
        -   **Connection Point State** (Indices 27-32): Position, Velocity.
        -   **Adaptive States** (Indices 33+): Estimates for system control parameters, initialized based on `MRAC_v` (Version 1 or 2).

3.  **Simulation (ODE Integration)**:
    -   Uses `ode23` to solve the equations of motion over the time span defined in `data.IntegrationTime`.
    -   Switch logic (`MRAC_v`) determines which dynamics function is called:
        -   `stateDeriv_withGrav_LiamSet_MRAC1_args_mex` (MRAC v1)
        -   `stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_mex` (MRAC v2)

4.  **Post-Processing & Reconstruction**:
    -   After integration, the script iterates through the time history to reconstruct control inputs not saved directly in the state vector.
    -   Recomputes **Tether Tension** (`TmagST`) and **Control Thrust** (`FmagST`) by re-evaluating the system dynamics at each time step.
    -   Calculates perturbations (e.g., J2 effect) to isolate the control effort.

5.  **Analysis & Visualization**:
    -   Computes the **Cost Function**: Root Mean Square Error (RMSE) between actual and desired tether elongation.
    -   Generates plots for:
        -   **Tether Elongation**: Comparing actual vs. desired/reference elongation.
        -   **Thrust Profile**: Showing control effort.
        -   **Adaptive Parameter Convergence**: Tracking system control parameters.
    -   Saves results to `.mat` files (e.g., `subTetherMRAC_v2.mat`) and plots to `.png` files.

## Key Support Files

### State Derivative Functions
These functions define the system dynamics and control laws used by the ODE solver.
- **`stateDeriv_withGrav_LiamSet_MRAC1_args.m`**: Implements the system dynamics with MRAC Version 1 control logic. It calculates forces and torques on the chaser, target, and connection points, including tether tension, gravity (with J2 perturbation), and adaptive control thrust.
- **`stateDeriv_withGrav_LiamSet_AdaptiveLinear_args.m`**: Implements the system dynamics with MRAC Version 2 control logic. (MRAC Version 2).

### Post-Processing and Visualization
- **`subTetherPostProcessing_vidOnly_ABtarget_v2.m`**: Handles post-processing and visualization. It generates a video (`VideoVortexPostCapture_LVLH_Correct_FORCE.avi`) showing the motion of the chaser, target, and tethers in the LVLH frame.
- **`postAlignAng.m`**: Computes the alignment angle and elongation of the main tether.
- **`postCompConnPt.m`**: Computes the position and velocity of the connection points.
- **`postCompTension_Python.m`**: Calculates tether tension components.
- **`postEngAng.m`**: Computes energy or angular quantities.

### Visualization Helpers
Helper scripts for drawing 3D objects in the visualization:
- **`ZenitBody.m`**, **`ZenitRotated.m`**, **`ZenitTranslated.m`**, **`ZenitPlot.m`**: Functions to define, transform, and plot the "Zenit" body.
- **`cylinder2P.m`**: Utility to generate cylinder graphics.

## Data Files
- **`dataIC_ForSTrun_Neha.mat`**, **`dataIC_ForSTrun_Neha_002.mat`**, **`dataIC_ForSTrun_Neha_007.mat`**: MAT-files containing initial conditions and simulation data required to initialize the scripts.
- **`subTetherMRAC_v2.mat`**: Output file where simulation results are saved.

## Usage
1.  Ensure all files are in the MATLAB path.
2.  Open `nehaST_MRAC_script.m`.
3.  Adjust parameters or select the desired `wIn` configuration if necessary.
4.  Run the script to execute the simulation and generate plots.
5.  Run `subTetherPostProcessing_vidOnly_ABtarget_v2.m` to visualize the results and generate a video.
