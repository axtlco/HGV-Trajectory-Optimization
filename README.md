# Hypersonic Gliding Vehicle Trajectory Optimization

**Hypersonic Gliding Vehicle (HGV) Optimal Trajectory Simulation using MATLAB**

This project presents a **MATLAB-based trajectory optimization framework** for **Hypersonic Gliding Vehicles (HGVs)**.  
The framework computes optimal flight trajectories from launch to target impact based on predefined objective functions and vehicle aerodynamic characteristics.

The problem is formulated as an **optimal control problem**, enabling flexible mission design through single-objective or multi-objective cost functions.

---

## Objective Functions

The following objective functions are supported and can be applied individually or in combination:

- **Minimum flight time**
- **Minimum fuel consumption** (or control effort)
- **Maximum downrange**
- **Maximum crossrange**

These objectives can be combined into a **multi-objective cost function** using user-defined weighting factors.

---

## Vehicle Models

This project considers **three different Hypersonic Gliding Vehicle configurations**.  
Each configuration is characterized by distinct aerodynamic properties defined by:

- Lift coefficient
- Drag coefficient

The aerodynamic coefficients are directly incorporated into the trajectory optimization process to reflect shape-dependent flight characteristics.

---

## Reference

> Lin-shu He, et al.  
> *Optimal trajectory and heat load analysis of different shape lifting reentry vehicles for medium range application*,  
> 2015.

---

## Requirements

- **MATLAB** (latest version recommended)
- **GPOPS-II license**

This project requires **MATLAB GPOPS-II** for execution.  
The simulation cannot be run without a valid GPOPS-II license.

---
