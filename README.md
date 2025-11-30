# robot_arm_
![tempFileForShare_20251130-230720](https://github.com/user-attachments/assets/f36578e4-f065-4a4f-b3b9-8798d160f84c)

README — Robot Arm Control (PyBullet Simulation)

This project demonstrates multiple control strategies for a 7-DOF KUKA iiwa robotic arm simulated in PyBullet.
The arm performs a sequence of target-tracking tasks using:

Admittance Control

Impedance Control

Static Control

Dynamic (Sinusoidal) Control

Inverse Kinematics (IK)

HJB-Inspired Optimal Control

All modes run automatically, one after another, while the simulation records a video
robot_arm_controls_hjb.mp4.

1. Overview

The simulation loads:

PyBullet physics engine

KUKA iiwa 7-DOF robot arm

Several predefined end-effector targets

The control loop switches modes every 10 seconds, letting observers see the difference in motion quality and stability.

2. Control Methods (with Formulas)

Below is a simple explanation of each controller and the formulas we approximate in code.

2.1 Admittance Control

Admittance control treats the arm like a mass–damper system responding to external forces:

𝑀
𝑥
¨
+
𝐵
𝑥
˙
=
𝐹
ext
M
x
¨
+B
x
˙
=F
ext
	​


Here we approximate velocity command as:

𝑥
˙
=
𝐾
(
𝑥
target
−
𝑥
ee
)
x
˙
=K(x
target
	​

−x
ee
	​

)

Code version:

velocity = (target_pos - ee_pos) * 0.5


This makes the arm move smoothly toward the target.

2.2 Impedance Control

Impedance control makes the robot behave like a spring-damper:

𝐹
=
𝐾
(
𝑥
target
−
𝑥
ee
)
+
𝐷
(
𝑥
˙
target
−
𝑥
˙
ee
)
F=K(x
target
	​

−x
ee
	​

)+D(
x
˙
target
	​

−
x
˙
ee
	​

)

We convert the virtual force to joint velocity:

𝑞
˙
∝
𝐹
q
˙
	​

∝F

Code approximation:

diff = target_pos - ee_pos
joint_vel = diff * 0.5


This creates stiffer, more precise motion.

2.3 Static Hold Control

Goal: keep the end-effector at a fixed position.

Simple proportional error:

𝑞
˙
=
𝐾
(
𝑥
hold
−
𝑥
ee
)
q
˙
	​

=K(x
hold
	​

−x
ee
	​

)

Lower gain → robot stays stable without oscillations.

2.4 Dynamic Control (Sinusoidal)

Used to demonstrate smooth periodic joint motion:

𝑞
˙
(
𝑡
)
=
𝐴
sin
⁡
(
2
𝜋
𝑓
𝑡
)
q
˙
	​

(t)=Asin(2πft)

Code:

amp * sin(2*pi*freq*t)

2.5 Inverse Kinematics (IK)

We solve:

𝑞
∗
=
IK
(
𝑥
target
)
q
∗
=IK(x
target
	​

)

Then control joint velocity as:

𝑞
˙
=
𝑘
(
𝑞
∗
−
𝑞
)
q
˙
	​

=k(q
∗
−q)

This makes motion direct and accurate.

2.6 HJB-Inspired Optimal Control

We minimize a cost:

𝐽
=
(
𝑥
target
−
𝑥
ee
)
2
+
𝑢
2
J=(x
target
	​

−x
ee
	​

)
2
+u
2

Approximate Value function:

𝑉
(
𝑥
)
=
∥
𝑥
target
−
𝑥
∥
2
V(x)=∥x
target
	​

−x∥
2

Therefore:

∇
𝑉
=
𝑥
−
𝑥
target
∇V=x−x
target
	​


Optimal control law:

𝑢
=
−
𝛼
∇
𝑉
u=−α∇V

Code:

gradV = diff
u = -alpha * gradV


This makes motion smooth but goal-directed, like an optimal controller.

3. Code Structure
Initialization

Load PyBullet

Load plane + robot

Set gravity

Prepare dt, joint count, and target points

Control Functions

Each function outputs a list of joint velocities (list[num_joints]).

Main Execution

Each mode runs for 10 seconds, cycling targets automatically.

Video Logging

Recorded using:

p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "robot_arm_controls_hjb.mp4")

4. How the Targets Are Sequenced

The arm moves through:

[0.5, 0.0, 0.3]
[0.5, 0.2, 0.3]
[0.3, 0.2, 0.5]
[0.3, -0.2, 0.5]
[0.5, -0.2, 0.3]


The index switches based on simulation steps to show different poses for each controller.

5. Why This Project Is Useful

This simulation clearly demonstrates:

Differences between force-based and position-based control

How IK converts task-space goals into joint motion

How an HJB (optimal control)–inspired strategy behaves

Smooth video output suitable for internship/project demonstrations

It shows a strong grasp of robotics fundamentals, control theory, and simulation tools.
