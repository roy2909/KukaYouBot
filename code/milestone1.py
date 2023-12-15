"""cut and paste this to run the code but has to be in the same directory:
 python3 milestone1.py """
import numpy as np

def NextState(config, control, dt, max_speed_joint, max_speed_wheel):
    chassis_config = config[:3]
    arm_config = config[3:8]
    wheel_angles = config[8:]

    joint_speeds = control[4:]
    wheel_speeds = control[:4]

    # Applying speed limits
    joint_speeds = np.clip(joint_speeds, -max_speed_joint, max_speed_joint)
    wheel_speeds = np.clip(wheel_speeds, -max_speed_wheel, max_speed_wheel)

    # Calculate new joint angles and wheel angles
    new_arm_config = arm_config + joint_speeds * dt
    new_wheel_angles = wheel_angles + wheel_speeds * dt

    # Odometry calculations
    r = 0.0475
    l = 0.47 / 2
    w = 0.3 / 2

    # Calculate velocities of the wheels in the body frame
    Vb = r / 4 * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                           [1, 1, 1, 1], [-1, 1, -1, 1]]) @ wheel_speeds

    w_bz = Vb[0]
    v_bx = Vb[1]
    v_by = Vb[2]

    dqb = np.array([0., 0., 0.])
    if w_bz < 1e-3:
        dqb[1] = v_bx
        dqb[2] = v_by
    else:
        dqb[0] = w_bz
        dqb[1] = (v_bx * np.sin(w_bz) + v_by * (np.cos(w_bz) - 1)) / w_bz
        dqb[2] = (v_by * np.sin(w_bz) + v_bx * (1 - np.cos(w_bz))) / w_bz

    phi_k = chassis_config[0]
    dq = np.array([[1, 0, 0],
                   [0, np.cos(phi_k), -np.sin(phi_k)],
                   [0, np.sin(phi_k), np.cos(phi_k)]]) @ dqb
    

    q_next = chassis_config + dq * dt
    updated_chassis_config = q_next
    print(dq * dt)

    # Construct the next configuration
    next_config = np.concatenate([updated_chassis_config, new_arm_config, new_wheel_angles])

    return next_config

def main():
    initial_config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    controls = np.array([10, 10, 10, 10, 0, 0, 0, 0, 0])  # Example control speeds
    max_speed_joints = 10
    max_speed_wheels = 10
    time_step = 0.01

    num_steps = 100
    result_data = np.zeros([num_steps, 13])

    current_config = initial_config
    for i in range(num_steps):
        result_data[i, :-1] = NextState(current_config, controls, time_step, max_speed_joints, max_speed_wheels)
        result_data[i, 12] = 0  # Gripper state

        current_config = result_data[i, :-1]

    np.savetxt("Roy_Rahul_milestone1.csv", result_data, delimiter=",")

if __name__ == "__main__":
    main()

