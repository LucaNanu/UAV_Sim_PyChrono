# High-fidelity PyChrono-based Simulator
[![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE.txt)
[![Website](https://img.shields.io/badge/Website-acslstack.com-green)](https://www.acslstack.com/)


## Introduction

The **UAV_Sim_PyChrono** is a high-fidelity PyChrono-based simulator designed for multi-rotor UAVs (Uncrewed Aerial Vehicles).


## Outlook on the Control Architecture

Autonomous UAVs with collinear propellers are inherently under-actuated. For this reason, the software includes:

- **Inner Loop**: Handles the rotational dynamics.
- **Outer Loop**: Handles the translational dynamics.

Both loops are governed by nonlinear equations of motion.

### Available Control Solutions

This software currently offers two control solutions for the inner and outer loops:

1. **Continuous-Time Feedback-Linearizing Control Law** combined with a **PID (Proportional-Integral-Derivative) Control Law**.
2. The above control law is augmented by a **Robust Model Reference Adaptive Control (MRAC) System**, incorporating a simplified quadratic-in-the-velocity aerodynamic model.

For further details on these control architectures, refer to the publications found [here](https://www.acslstack.com/Journals).

Future versions of the software will include additional control systems.

### Available UAV assets
1. X8 copter Original: Original version of ACSL Lab
2. X8 copter modified for ThrustStand project: the generated .obj file is only 1 SolidWorks Part.
3. ThrustStand UAV
├── src
│   ├── main.py
│   └── utils.py
├── tests
│   └── test_main.py
└── README.md
## List of folders
- **Images/**: Contains all project-related images.
- **acsl_pychrono/**: Python modules or scripts related to the ACSL pychrono simulations.
- **assets/**: Environment and vehicle files.
- **install/**: Installation scripts.
- **logs/**: Logs generated during simulation for saving variables.
- **params/**: .JSON files to generate used-defined trajectories
- **main.py**: The main script to run the project.
- **installation_guide.md**: Instructions for installing 
- **History_of_Simulator.md**: Documentation of the simulator’s development history.
- **LICENSE.txt**: License information for the project.

## Maintenance Team

- [**Andrea L'Afflitto**](https://github.com/andrealaffly)
- [**Mattia Gramuglia**](https://github.com/mattia-gramuglia)

For more information, visit [acslstack.com](https://www.acslstack.com/).

[![ACSL Flight Stack Logo](https://lafflitto.com/images/ACSL_Logo.jpg)](https://lafflitto.com/ACSL.html)


---

This software is distributed under a permissive **3-Clause BSD License**.
