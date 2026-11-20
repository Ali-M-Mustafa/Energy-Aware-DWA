# Energy-Aware-DWA
### Energy-Aware Mobile Robot Path Planning Using Dynamic Window Approach

[![Paper](https://img.shields.io/badge/IEEE%20Xplore-Paper-blue)](https://doi.org/10.1109/ICAC70325.2026.11712285)

MATLAB code for a Dynamic Window Approach (DWA) planner that adds a motor energy model to its cost function, so a differential-drive robot picks paths that use less energy and move more smoothly.

<p align="center">
  <img src="path_comparison.png" alt="Classical DWA vs energy-aware DWA paths on two maps" width="900">
</p>

*Classical DWA (blue) and energy-aware DWA (orange) on map 1 (left) and map 2 (right), as in Figs. 3a and 4a of the paper.*

## Overview

The DWA is a widely used local planner for mobile robots. At every step it tries a set of velocity commands and picks the one that best balances heading toward the goal, distance from obstacles and speed. It ignores how much energy each choice costs. This project adds an estimate of the drive motors' energy use to that choice:

| Component | What it does | Files |
|---|---|---|
| Energy model | Converts (v, ω) to wheel speeds and estimates motor power from friction torque and coil losses (eqs. 8–15) | `CalcEnergy` inside each planner |
| Classical DWA | Scores candidates on heading, clearance and velocity (eq. 7) | `classical_dwa.m` |
| Energy-aware DWA | Adds a weighted energy term to the score (eq. 16) | `energy_aware_dwa.m` |
| Sensitivity analysis | Raises one weight at a time to 0.9 on map 3 (Fig. 5, Table IV) | `sensitivity_analysis.m` |
| Maps and plots | Test environments and the comparison figures | `get_map.m`, `plot_comparison.m`, `plot_sensitivity.m` |

## How it works

1. The robot is a differential-drive platform with a forward-Euler motion model (eq. 2) and a 0.1 s time step.
2. Every step, the DWA samples (v, ω) pairs inside the dynamic window (up to 0.5 m/s and 180°/s) and predicts each one 3 s ahead.
3. Candidates whose predicted path comes within 0.2 m of an obstacle are rejected. The rest are scored on heading, clearance and speed.
4. The energy-aware planner also estimates each candidate's energy: wheel speeds → friction torque → coil current → electrical power. The energy is expressed per metre travelled, and lower energy gives a higher score.
5. The best candidate is applied for one step, and the loop repeats until the robot is within 0.5 m of the goal.

## Results

Classical vs. energy-aware DWA with α = β = γ = δ = 0.5 (Table III of the paper):

- Total energy drops by 3.6% on map 1 and 8.6% on map 2, compared with the classical DWA.
- Average power goes up, but the robot finishes sooner, so total energy goes down.

Weight sensitivity on map 3, one weight raised to 0.9 (Table IV of the paper):

- Every case with the energy term moves much more smoothly than the case without it.
- α = 0.9 gives the shortest path, the lowest energy and the smoothest motion.
- Raising δ to 0.9 does not lower the energy further than δ = 0.5.


## Getting started

### 1. Requirements

- MATLAB (no toolboxes needed)

### 2. Install

```bash
git clone https://github.com/Ali-M-Mustafa/Energy-Aware-DWA.git
cd Energy-Aware-DWA/src
```

Run all scripts from inside `src/`.



## Repository structure

```
Energy-Aware-DWA/
├── src/
│   ├── classical_dwa.m           # Classical DWA baseline (Table III)
│   ├── energy_aware_dwa.m        # Energy-aware DWA, eq. 16 (Table III)
│   ├── sensitivity_analysis.m    # Weight sensitivity on map 3 (Table IV)
│   ├── get_map.m                 # The three test maps, start and goal points
│   ├── plot_comparison.m         # Path and energy figures (Figs. 3–4)
│   └── plot_sensitivity.m        # Trajectory and velocity figures (Fig. 5)
├── path_comparison.png
├── LICENSE
├── README.md
└── .gitignore
```

## Publication

If you use this code, please cite:

A. M. Mustafa and B. Brahmi, "Energy-Aware Mobile Robot Path Planning Using Dynamic Window Approach," in *2026 31st International Conference on Automation and Computing (ICAC)*, 2026, doi: 10.1109/ICAC70325.2026.11712285.

```bibtex
@inproceedings{mustafa2026energy,
  author    = {Mustafa, Ali M. and Brahmi, Brahim},
  title     = {Energy-Aware Mobile Robot Path Planning Using Dynamic Window Approach},
  booktitle = {2026 31st International Conference on Automation and Computing (ICAC)},
  year      = {2026},
  publisher = {IEEE},
  doi       = {10.1109/ICAC70325.2026.11712285}
}
```

## License

The project code is released under the [MIT License](LICENSE).

Credits:
- The planners are adapted from `DynamicWindowApproachSample.m` in Atsushi Sakai's [MATLABRobotics](https://github.com/AtsushiSakai/MATLABRobotics), Copyright (c) 2014 Atsushi Sakai.
- The energy model follows M. Yao et al., "Improved dynamic windows approach based on energy consumption management and fuzzy logic control for local path planning of mobile robots," *Computers & Industrial Engineering*, vol. 187, p. 109767, 2024.
- The Dynamic Window Approach is from D. Fox, W. Burgard and S. Thrun, "The Dynamic Window Approach to Collision Avoidance," *IEEE Robotics & Automation Magazine*, vol. 4, no. 1, pp. 23–33, 1997.
