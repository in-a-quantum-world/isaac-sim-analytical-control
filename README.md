# isaac-sim-analytical-control

Analytical robotic controllers in NVIDIA Isaac Sim, built up from single-joint feedback control to contact-rich manipulation under sensing error.

In this repo, I compute every motion from dynamics, kinematics and geometry - intentionally there are no learned models, no reinforcement learning and no recorded demonstrations.

## What it is
Essentially this serves as a Python library and a set of standalone Isaac Sim examples, each with a written derivation:

- **Joint control.** PD and PID on a single joint treated as a second-order system, feedforward, gravity compensation and computed-torque control.
- **Kinematics.** Forward kinematics, the geometric Jacobian, singularity and manipulability analysis, and inverse kinematics in closed form and by damped least squares.
- **Cartesian control.** Resolved-rate control and straight-line tool motion.
- **Contact.** Grasping, pick and place as an explicit state machine, and peg-in-hole insertion with guarded moves, spiral search, impedance and admittance control.
- **Perception.** The pinhole camera model, lens distortion, calibration, and object pose from classical vision.
- **Uncertainty.** How sensing error propagates to task failure, and how to size a search from an error budget.
- **Tooling.** Robustness studies with fixed seeds, robot description validation, and per-step logging with replay.

Each example has a note in `notes/` that states the model, derives the controller, compares prediction with measurement and says when the method fails.

## Who this is for

First of all, for myself: I am using it to learn Isaac Sim and classical robot control properly, so I can check theory against simulation at every step. It should also be useful to anyone who has an  is starting with Isaac Sim, and wants small readable examples that do not depend on a training pipeline.

## Built with

- Python
- NVIDIA Isaac Sim 6.x, run through its standalone Python workflow
- NumPy for the control and kinematics code
- OpenCV for camera calibration and marker detection
- Matplotlib for plots
- pytest for the parts that do not need the simulator


Requirements:

- A machine that can run Isaac Sim (an NVIDIA RTX GPU is needed)
- Isaac Sim 6.x installed
- The Appropriate robot model (see "Robot model" below)




## Robot model

This repository does not include the robotic arm model files. The arm's URDF, MJCF and meshes are published by TheRobotStudio in [this directory](https://github.com/TheRobotStudio) 

## Licence

MIT. 
