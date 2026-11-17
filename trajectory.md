# Roadmap


A milestone is finished when its "Done when" line is true and I can answer its "Explain" questions on paper, without looking at the code.


| # | Milestone | Hours | Status |
| --- | --- | --- | --- |
| | **Part A. Simulation and joint control** | | |
| 0 | Set-up, first scene, determinism | 3 | In progress |
| 1 | One joint as a second-order system: PD and PID | 8 | Planned |
| 2 | Joint-space trajectories and feedforward | 5 | Planned |
| 3 | Gravity compensation and computed torque | 10 | Planned |
| | **Part B. Kinematics and Cartesian control** | | |
| 4 | Forward kinematics | 6 | Planned |
| 5 | The Jacobian, singularities and manipulability | 7 | Planned |
| 6 | Inverse kinematics, closed form and numerical | 9 | Planned |
| 7 | Resolved-rate control and straight-line motion | 6 | Planned |
| | **Part C. Contact and manipulation** | | |
| 8 | Grasp and lift | 6 | Planned |
| 9 | Pick and place as a state machine | 7 | Planned |
| 10 | Peg in hole: guarded moves, search and compliance | 14 | Planned |
| | **Part D. Perception and uncertainty** | | |
| 11 | Camera model, distortion and calibration | 9 | Planned |
| 12 | Object pose from a camera | 8 | Planned |
| 13 | Closing the loop from camera to task | 7 | Planned |
| | **Part E. Robustness and tooling** | | |
| 14 | Robustness study of the controllers | 8 | Planned |
| 15 | Robot description: import, convert, validate | 7 | Planned |
| 16 | Logging and replay | 6 | Planned |
| 17 | Porting to a 6-axis arm | 9 | Planned |
| 18 | Packaging, tests and first release | 10 | Planned |

---
(Note on notation: $q$ joint angles, $\tau$ joint torques, $x$ tool pose, $J$ the Jacobian, $e = q_d - q$ the tracking error.)

## Part A. Simulation and joint control

### 0. Set-up, first scene, determinism (3 h)

Build:
- `docs/setup.md`: Isaac Sim version, operating system, GPU, source of the SO-101 model.
- `examples/00_load_arm.py`: start the simulator, add a ground plane and the arm, step the physics at a fixed time step, print joint names, limits and positions.
- Run the same commanded motion twice and compare the joint logs.

Done when: the script runs from a clean checkout, and two runs with the same inputs give joint trajectories that agree to a stated tolerance.

Explain: what an articulation is, the difference between the physics step and the render step, and what has to be fixed for a run to be repeatable.

### 1. One joint as a second-order system: PD and PID (8 h)

Build:
- Drive a single joint in effort mode with the others locked. Model it as $I\ddot q + b\dot q + m g l \sin q = \tau$.
- Implement PD, $\tau = K_p e - K_d \dot q$, then PID with an integral clamp.
- Measure rise time, overshoot, settling time and steady-state error from step responses, and compare them with the prediction from $\omega_n = \sqrt{K_p / I}$ and $\zeta = (b + K_d) / (2\sqrt{K_p I})$.
- Identify $I$ and $b$ from a free response.

Done when: predicted and measured overshoot agree within 10% for three gain settings, and the PD steady-state error under gravity matches $m g l \sin q / K_p$.

Explain: why PD alone leaves a steady-state error under gravity, what the integral term fixes and what windup is, and how the closed-loop poles move as $K_p$ and $K_d$ change.

### 2. Joint-space trajectories and feedforward (5 h)

Build:
- `so101_control/trajectory.py`: cubic and quintic time scaling, $s(t) = 10\sigma^3 - 15\sigma^4 + 6\sigma^5$ with $\sigma = t/T$, and a trapezoidal velocity profile with velocity and acceleration limits.
- Track each profile with PD alone, then with velocity and acceleration feedforward.

Done when: feedforward reduces the peak tracking error by a measured factor, and no profile exceeds the joint velocity limits.

Explain: why the quintic has zero velocity and acceleration at both ends, and why feedback alone always lags a moving reference.

### 3. Gravity compensation and computed torque (10 h)

Build:
- Derive the gravity torque vector $g(q)$ from the link masses and centres of mass in the robot description. Check it against the torques the simulator needs to hold the arm still.
- PD with gravity compensation, $\tau = K_p e - K_d \dot q + g(q)$.
- Computed-torque control using $M(q)\ddot q + C(q,\dot q)\dot q + g(q) = \tau$: $\tau = M(q)(\ddot q_d + K_d \dot e + K_p e) + C(q,\dot q)\dot q + g(q)$.
- Compare tracking error for PD, PD with gravity compensation, and computed torque on the same trajectory.

Done when: my $g(q)$ agrees with measured holding torques within 5% at 20 poses, and the three controllers are ranked by measured error with an explanation.

Explain: what each term of the manipulator equation represents, why computed torque turns the arm into decoupled double integrators when the model is exact, and what happens when the model's masses are wrong by 20%.

---

## Part B. Kinematics and Cartesian control

### 4. Forward kinematics (6 h)

Build:
- `so101_control/kinematics.py`: tool pose from joint angles using homogeneous transforms built from the joint axes and link offsets in the robot description.
- Compare with the simulator's tool pose at 100 random configurations.

Done when: position agrees within 1 mm and orientation within 0.5 degrees at every sample.

Explain: which frame each transform is expressed in, and why the order of composition matters.

### 5. The Jacobian, singularities and manipulability (7 h)

Build:
- Derive the geometric Jacobian, $\dot x = J(q)\dot q$, column by column from the joint axes. Check it against finite differences of the forward kinematics.
- Compute singular values of $J$ and the manipulability $w = \sqrt{\det(J J^\top)}$ across the workspace, and plot where it collapses.
- Use $\tau = J^\top F$ to predict the joint torques for a force at the tool, and check in simulation.

Done when: analytic and numerical Jacobians agree to 1e-6, and the singular configurations found numerically match the ones identified by hand.

Explain: what a singularity means physically, why a 5-joint arm has a 6 by 5 Jacobian and what that implies about reachable tool velocities, and why forces map through the transpose.

### 6. Inverse kinematics, closed form and numerical (9 h)

Build:
- Closed form: base rotation from the target's direction, the shoulder and elbow as a two-link planar arm, the wrist for tool pitch. Both elbow branches.
- Numerical: damped least squares, $\Delta q = J^\top (J J^\top + \lambda^2 I)^{-1} \Delta x$, with joint-limit clamping.
- A reachability check that names the reason a target fails (too far, too close, joint limit, orientation not achievable).

Done when: for 200 reachable targets both solvers land within 1 mm, and the numerical solver stays stable when started near a singularity.

Explain: why damping trades accuracy for stability near singularities, how to choose $\lambda$, and which tool orientations five joints can never reach at a given position.

### 7. Resolved-rate control and straight-line motion (6 h)

Build:
- Resolved-rate control: command $\dot q = J^{+}(\dot x_d + K e_x)$ at every step to follow a Cartesian path.
- Compare with solving inverse kinematics at interpolated waypoints.
- `examples/07_cartesian_line.py`: trace a square and measure the deviation from each edge.

Done when: the tool stays within 2 mm of each edge with both methods, and the note says which method behaves better near a singularity.

Explain: why interpolating in joint space does not give a straight tool path, and what the feedback term $K e_x$ corrects.

---

## Part C. Contact and manipulation

### 8. Grasp and lift (6 h)

Build:
- Gripper commands with a check on the measured opening.
- A state machine (approach, descend, close, lift) for a cube at a known pose.
- Vary friction coefficient and grip force and find where the grasp starts to slip. Compare with the condition $2\mu F_n \ge m g$.

Done when: the grasp succeeds 10 times out of 10 at nominal settings, and the measured slip boundary is within 20% of the prediction.

Explain: the friction cone, and why the simulator's contact parameters can change the result as much as the controller does.

### 9. Pick and place as a state machine (7 h)

Build:
- Add carry, lower, release and retreat states.
- A postcondition check after every state, a timeout per state, one retry on a failed grasp, and a named failure reason on every exit path.

Done when: the sequence succeeds 10 times out of 10 at the nominal position and every failure injected by hand (object moved, object removed) ends in the correct failure reason.

Explain: what each state assumes when it starts, and how the script detects that the assumption is false.

### 10. Peg in hole: guarded moves, search and compliance (14 h)

Build:
- A peg and a hole with a stated radial clearance $c$.
- Position-only insertion, and the largest lateral offset it tolerates.
- Guarded moves: descend until the contact force passes a threshold, then stop.
- Search: an Archimedean spiral with pitch at most $2c$, so every point in the search disc passes within $c$ of the path. Compare with a raster search on time taken.
- Compliance: Cartesian impedance, $F = K(x_d - x) + D(\dot x_d - \dot x)$ with $\tau = J^\top F + g(q)$, and an admittance version in which measured force adjusts the commanded velocity of a position-controlled arm.
- For each strategy, measure the largest lateral offset it recovers from and the peak contact force.

Done when: there is a table of tolerated offset, time and peak force for the four strategies, and the search radius needed for a given position uncertainty is derived and confirmed.

Explain: why a tight clearance makes position control fail, how the spiral pitch bound follows from the clearance, when to choose impedance and when admittance, and what limits the stiffness $K$ in contact.

---

## Part D. Perception and uncertainty

### 11. Camera model, distortion and calibration (9 h)

Build:
- The pinhole model, $u = f_x X/Z + c_x$, $v = f_y Y/Z + c_y$, and first-order radial distortion, $x_d = x(1 + k_1 r^2)$.
- Calibrate intrinsics from checkerboard images with OpenCV, and the camera-to-robot transform from known tool positions.
- Leave the distortion uncorrected on purpose and measure the position error on the table as a function of distance from the image centre.

Done when: reprojection error is under 0.5 pixels after calibration, and the uncorrected position error is shown to grow as $r^3$.

Explain: the difference between intrinsics and extrinsics, why uncorrected radial distortion gives an error that grows with the cube of the distance from the centre, and why a calibration fitted near the centre is worst at the edges.

### 12. Object pose from a camera (8 h)

Build:
- Position by colour segmentation, pixel to ray, and intersection with the table plane.
- Position and rotation from a printed fiducial marker using OpenCV's classical detector and a perspective-n-point solve.
- Compare both with the simulator's true pose across the workspace.

Done when: there is an error map in millimetres and degrees for both methods, and an error budget that attributes the error to pixel noise, calibration and geometry.

Explain: how one pixel of error turns into millimetres on the table as a function of height and viewing angle.

### 13. Closing the loop from camera to task (7 h)

Build:
- Run pick and place and peg in hole from the camera estimate.
- Add controlled error to the estimate and find the point at which each task starts to fail.
- Set the search radius in milestone 10 from the measured error budget in milestone 12.

Done when: for each task there is a curve of success against injected pose error, and the search radius chosen from the error budget makes insertion succeed across the workspace.

Explain: which task is more sensitive to pose error and why, and how sensing error and control tolerance combine into a success condition.

---

## Part E. Robustness and tooling

### 14. Robustness study of the controllers (8 h)

Build:
- Vary object position and rotation, friction, object mass and sensor noise with fixed seeds, one factor at a time and then together.
- Record success or failure and the failure reason for each run.
- Rank the factors by how much they change the success rate.

Done when: the operating region of each controller is drawn, the most sensitive factor is identified, and every failure cluster has an explanation that refers to an earlier milestone.

Explain: why testing only near the nominal position overstates how well a controller works, and how many runs are needed before a success rate means something.

### 15. Robot description: import, convert, validate (7 h)

Build:
- Load the arm from its URDF, convert it to USD, and write a validation script that compares joint axes, limits, link masses and inertias between the two.
- Check that forward kinematics from the URDF matches the converted model in simulation.
- A checklist of the errors that conversions introduce (flipped axes, missing inertia, wrong units, collision meshes).

Done when: the validator passes on the SO-101 and catches three errors introduced by hand.

Explain: what information a URDF holds that a simulator needs, and which of it is most often lost or changed in conversion.

### 16. Logging and replay (6 h)

Build:
- A per-step log of joint state, tool pose, commanded action and contact force in a simple file format.
- Plots and a replay script that reconstructs a run from its log without rerunning the controller.

Done when: any run from milestones 9, 10 and 14 can be replayed and its failure located from the log alone.

Explain: what has to be logged for a failed run to be diagnosable afterwards.

### 17. Porting to a 6-axis arm (9 h)

Build:
- Run the kinematics, Cartesian control and pick and place on a 6-axis arm from Isaac Sim's asset library.
- Note which parts of the library were specific to the SO-101 and refactor them behind a robot description.

Done when: pick and place runs on both arms from the same controller code with only the robot description changed.

Explain: what a sixth axis adds to the set of reachable poses, and what changes in the Jacobian and the inverse kinematics.

### 18. Packaging, tests and first release (10 h)

Build:
- `so101_control` as an installable package with type hints and docstrings.
- Tests for trajectories, kinematics, the Jacobian and camera geometry that run without the simulator.
- A GitHub Actions workflow running those tests on every push.
- A documentation page per example and a reading order.
- Tag release v0.1.0.

Done when: a new user can install the package, run the tests and run example 0 by following the README alone.

---

## Ideas for later

- The same controllers on the physical SO-101, and a comparison of simulated and real step responses.
- LQR on the linearised joint model, compared with the PD gains from milestone 1.
- Collision checking and simple motion planning around obstacles.
- Time-optimal trajectories under velocity and torque limits.
