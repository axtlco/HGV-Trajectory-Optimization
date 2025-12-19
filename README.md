Hypersonic Gliding Vehicle Trajectory Optimization

This project presents a MATLAB-based trajectory optimization module for Hypersonic Gliding Vehicles (HGVs).
The module computes the optimal flight trajectory from launch to target impact based on predefined objective functions and the aerodynamic characteristics of the vehicle.

The trajectory optimization problem is formulated as an optimal control problem, allowing flexible selection and combination of mission objectives through a multi-objective cost function.

Objective Functions

The following objective functions are supported and can be used individually or in combination:

Minimum flight time

Minimum fuel consumption (or control effort)

Maximum downrange

Maximum crossrange

These objectives can be combined into a single multi-objective cost function using user-defined weighting factors.

Vehicle Models

Three different Hypersonic Gliding Vehicle configurations are considered in this project.
Each vehicle is characterized by its own aerodynamic properties, defined through lift and drag coefficients.

The aerodynamic coefficients are incorporated into the trajectory optimization to reflect shape-dependent flight characteristics.

Reference

Lin-shu He, et al.,
Optimal trajectory and heat load analysis of different shape lifting reentry vehicles for medium range application,
2015.

Requirements

MATLAB (latest version recommended)

GPOPS-II license

This project requires MATLAB GPOPS-II to run. Execution is not possible without a valid GPOPS-II license.