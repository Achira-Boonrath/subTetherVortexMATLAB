# Sub-Tether Debris Removal with MRAC Elongation Control

This folder contains MATLAB scripts and data files for simulating and analyzing a sub-tethered space debris removal system using Model Reference Adaptive Control (MRAC).

## Main Project File

### `nehaST_MRAC_script.m`
This is the primary script for running the simulation. It performs the following key functions:
- **Initialization**: Sets up simulation parameters, including tether properties (stiffness, damping), chaser and target mass/inertia, and initial conditions.
- **Optimization/Parameter Study**: Can be configured to run optimization loops or parameter studies (currently set up for single runs with specific `wIn` parameters).
- **Dynamics Integration**: Solves the system equations of motion using `ode23`. It calls specific state derivative functions based on the selected control mode (`MRAC_v`).
- **Post-Processing**: Calculates tension, elongation, thrust, and adaptive parameters after the simulation.
- **Plotting**: Generates plots for tether elongation, thrust magnitude, adaptive parameters, and state variables.

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
