# Tactical And Strategic Missile Guidance

MATLAB and Simulink simulations for missile guidance, engagement dynamics, filtering, covariance analysis, and adjoint-based performance analysis.

## Overview

This project contains engineering simulations related to tactical and strategic missile guidance. The work includes two-dimensional missile-target engagement models, linearised engagement dynamics, low-pass filtering, covariance analysis, Monte Carlo simulations, and single-lag adjoint methods.

## Main Contents

- `MATLAB Files/TwoD_Missile_Sim.m` - 2D missile-target engagement simulation
- `MATLAB Files/Linearised_engagement_sim.m` - linearised engagement model
- `MATLAB Files/Single_lag_adjoint_monte_carlo.m` - Monte Carlo adjoint analysis
- `MATLAB Files/Covariance Analysis/` - homing loop covariance analysis
- `MATLAB Files/Results/` - selected simulation outputs and figures

## Requirements

- MATLAB
- Simulink, for `.slx` models
- Control System Toolbox, if required by local scripts

## Usage

Open the project folder in MATLAB and run the relevant script from the `MATLAB Files` folder.

Example:

```matlab
TwoD_Missile_Sim