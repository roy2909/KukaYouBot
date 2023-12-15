"""cut and paste this to run the code but has to be in the same directory:
 python3 milestone2.py """
import modern_robotics as mr
import numpy as np

def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k=1):
    # gripper states for each segment
    gripper_states = [0, 0, 1, 1, 1, 1, 0, 0]

    # durations for each segment
    n_values = [4 * k / 0.01, 4 * k / 0.01, 0.65 * k / 0.01, 4 * k / 0.01, 4 * k / 0.01,
                4 * k / 0.01,  0.65 * k / 0.01, 4 * k / 0.01]

    # transformation matrices for each segment
    T = [Tse_initial, Tsc_initial @ Tce_standoff, Tsc_initial @ Tce_grasp, Tsc_initial @ Tce_grasp,
         Tsc_initial @ Tce_standoff, Tsc_final @ Tce_standoff, Tsc_final @ Tce_grasp, Tsc_final @ Tce_grasp,
         Tsc_final @ Tce_standoff]

    # List to hold all configurations and gripper states
    traj_list = []

    for i in range(8):
        #  duration for particular segment of trajectory 
        duration = 0.65 if i in [2, 6] else 6    #4

        # Calculate the trajectory for the current segment
        traj_segment = mr.ScrewTrajectory(T[i], T[i + 1], duration, n_values[i], 5)
        # print(f"count,{i} , duration,{duration},n_values, {n_values}")

        # Extract configuration data and gripper state for each point in the trajectory
        for x in traj_segment:
            config = np.concatenate([x[:3, :3].flatten(), x[:3, 3], [gripper_states[i]]])
            traj_list.append(config)

    final_traj = np.array(traj_list)

    # Save the trajectory as a CSV file
    np.savetxt("Roy_Rahul_milestone2.csv", final_traj, delimiter=",")
    return final_traj


# initial configurations of cube and end effector
Tsc_initial = np.array([[1, 0, 0, 1],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.025],
                        [0, 0, 0, 1]])
Tse_initial = np.array([[0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.5],
                        [0, 0, 0, 1]])


# final configurations
Tsc_goal = np.array([[0, 1, 0, 0],
                      [-1, 0, 0, -1],
                      [0, 0, 1, 0.025],
                      [0, 0, 0, 1]])

# standoff and grasp configurations
Tce_standoff = np.array([[-0.707, 0, 0.707, 0],
                         [0, 1, 0, 0],
                         [-0.707, 0, -0.707, 0.3],
                         [0, 0, 0, 1]])

Tce_grasp = np.array([[-0.707, 0, 0.707, 0],
                      [0, 1, 0, 0],
                      [-0.707, 0, -0.707, 0],
                      [0, 0, 0, 1]])

# Generate final reference trajectory
final_trajectory = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k=1)



