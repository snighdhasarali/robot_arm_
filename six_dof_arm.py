import pybullet as p
import pybullet_data
import time
import numpy as np

# -------------------------------
# Setup PyBullet
# -------------------------------
physicsClient = p.connect(p.GUI)  # GUI mode
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")

# Load 6-DOF KUKA arm
robotId = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

num_joints = p.getNumJoints(robotId)
dt = 1./240.
max_steps_per_target = 800

# Automated targets along XYZ axes
targets = [
    np.array([0.5, 0.0, 0.3]),
    np.array([0.5, 0.2, 0.3]),
    np.array([0.3, 0.2, 0.5]),
    np.array([0.3, -0.2, 0.5]),
    np.array([0.5, -0.2, 0.3])
]

# -------------------------------
# Control mode flag
# -------------------------------
automation = True  # Start in automated mode
current_target_index = 0
step_count = 0

# -------------------------------
# Simple automated control
# -------------------------------
def automated_control(robotId, target_pos):
    ee_state = p.getLinkState(robotId, num_joints-1)
    ee_pos = np.array(ee_state[0])
    diff = target_pos - ee_pos

    joint_velocities = [0.0]*num_joints
    for j in range(num_joints):
        joint_velocities[j] = np.clip(diff[0]*0.5 + diff[1]*0.5 + diff[2]*0.5, -0.1, 0.1)
    return joint_velocities

# -------------------------------
# Start recording
# -------------------------------
p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "6dof_arm_toggle_control.mp4")

# -------------------------------
# Simulation loop
# -------------------------------
print("Press 'm' for manual mode, 'a' for automated mode")

while True:
    keys = p.getKeyboardEvents()
    if ord('m') in keys:
        automation = False
        print("Manual mode ON (use mouse or sliders)")
    if ord('a') in keys:
        automation = True
        print("Automated mode ON")

    if automation:
        # Automated movement
        target = targets[current_target_index]
        joint_velocities = automated_control(robotId, target)
        for j in range(num_joints):
            p.setJointMotorControl2(robotId, j, p.VELOCITY_CONTROL, targetVelocity=joint_velocities[j])

        step_count += 1
        if step_count > max_steps_per_target:
            step_count = 0
            current_target_index = (current_target_index + 1) % len(targets)
    else:
        # Manual mode: disable motor control so mouse/sliders work
        for j in range(num_joints):
            p.setJointMotorControl2(robotId, j, p.VELOCITY_CONTROL, targetVelocity=0)

    p.stepSimulation()
    time.sleep(dt)
