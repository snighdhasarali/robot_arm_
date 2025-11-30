import pybullet as p
import pybullet_data
import time
import numpy as np

# -------------------------------
# Setup PyBullet
# -------------------------------
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
num_joints = p.getNumJoints(robotId)
dt = 1./240.

# -------------------------------
# Targets for inverse kinematics and HJB
# -------------------------------
ik_targets = [
    [0.5, 0.0, 0.3],
    [0.5, 0.2, 0.3],
    [0.3, 0.2, 0.5],
    [0.3, -0.2, 0.5],
    [0.5, -0.2, 0.3]
]

duration = 10  # seconds per mode
steps_per_mode = int(duration / dt)

# -------------------------------
# Control functions
# -------------------------------

def admittance_control(robotId, target_pos):
    ee_pos = np.array(p.getLinkState(robotId, num_joints-1)[0])
    velocity = (target_pos - ee_pos) * 0.5
    joint_velocities = [np.clip(np.linalg.norm(velocity), -0.1, 0.1)]*num_joints
    return joint_velocities

def impedance_control(robotId, target_pos):
    ee_pos = np.array(p.getLinkState(robotId, num_joints-1)[0])
    diff = target_pos - ee_pos
    joint_velocities = [np.clip(diff[j%3]*0.5, -0.1, 0.1) for j in range(num_joints)]
    return joint_velocities

def static_control(robotId, hold_pos):
    ee_pos = np.array(p.getLinkState(robotId, num_joints-1)[0])
    diff = hold_pos - ee_pos
    joint_velocities = [np.clip(diff[j%3]*0.2, -0.05, 0.05) for j in range(num_joints)]
    return joint_velocities

def dynamic_control(robotId, amplitude=0.1, frequency=0.5):
    t = time.time()
    joint_velocities = [amplitude*np.sin(2*np.pi*frequency*t)]*num_joints
    return joint_velocities

def inverse_kinematics_control(robotId, target_pos):
    joint_positions = p.calculateInverseKinematics(robotId, num_joints-1, target_pos)
    joint_velocities = []
    for j in range(num_joints):
        current_pos = p.getJointState(robotId, j)[0]
        velocity = (joint_positions[j] - current_pos)*2
        joint_velocities.append(np.clip(velocity, -0.2, 0.2))
    return joint_velocities

# -------------------------------
# Approximate HJB control
# -------------------------------
def hjb_control(robotId, target_pos, alpha=0.5):
    """
    Simple HJB-inspired discrete-time control:
    minimize cost = distance^2 + control effort^2
    """
    ee_pos = np.array(p.getLinkState(robotId, num_joints-1)[0])
    diff = target_pos - ee_pos
    
    # Value function derivative approximation: proportional to distance
    gradV = diff
    
    # Control law: u = -alpha * gradV (move opposite to cost gradient)
    joint_velocities = [np.clip(-alpha*gradV[j%3], -0.15, 0.15) for j in range(num_joints)]
    return joint_velocities

# -------------------------------
# Start video recording
# -------------------------------
p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "robot_arm_controls_hjb.mp4")

# -------------------------------
# Run all control modes sequentially
# -------------------------------
control_modes = [
    ("Admittance Control", admittance_control),
    ("Impedance Control", impedance_control),
    ("Static Control", static_control),
    ("Dynamic Control", dynamic_control),
    ("Inverse Kinematics", inverse_kinematics_control),
    ("HJB Control", hjb_control)
]

for mode_name, mode_func in control_modes:
    print(f"Running: {mode_name}")
    for step in range(steps_per_mode):
        if mode_name in ["Admittance Control", "Impedance Control", "Inverse Kinematics", "HJB Control"]:
            target_idx = step // (steps_per_mode // len(ik_targets))
            target_idx = min(target_idx, len(ik_targets)-1)
            target = np.array(ik_targets[target_idx])
            joint_velocities = mode_func(robotId, target)
        elif mode_name == "Static Control":
            hold_pos = np.array([0.4, 0.0, 0.4])
            joint_velocities = mode_func(robotId, hold_pos)
        else:  # Dynamic Control
            joint_velocities = mode_func(robotId)

        for j in range(num_joints):
            p.setJointMotorControl2(robotId, j, p.VELOCITY_CONTROL, targetVelocity=joint_velocities[j])
        p.stepSimulation()
        time.sleep(dt)

# Keep GUI open
print("All control modes finished! Video saved as 'robot_arm_controls_hjb.mp4'.")
for _ in range(20*240):
    p.stepSimulation()
    time.sleep(dt)

p.disconnect()

