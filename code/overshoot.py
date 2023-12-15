"""cut and paste this to run the code but has to be in the same directory:
 python3 overshoot.py """

from modern_robotics import *
from milestone1 import NextState
from milestone2 import TrajectoryGenerator
from milestone3 import FeedbackControl
import matplotlib.pyplot as plt
import datetime

def log(message):
    with open('logfile_overshoot.txt', 'a') as file:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        file.write(f"{timestamp}: {message}\n")

def overshoot():
    log("Starting the 'overshoot' function")
    initial_configuration = np.array([0.0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    
    
    # initial configurations of cube and end effector
    Tsc_initial = np.array([[1, 0, 0, 1],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.025],
                            [0, 0, 0, 1]])
    Tse_initial = np.array([[0, 0, 1, 0],
                            [0, 1, 0, 0],
                            [-1, 0, 0, 0.5],
                            [0, 0, 0, 1]])

    #gripper distance
    x=(3.5/100.0)/2
    # final configurations
    Tsc_goal = np.array([[0, 1, 0, 0],
                        [-1, 0, 0, -1],
                        [0, 0, 1, 0.025],
                        [0, 0, 0, 1]])

    # standoff and grasp configurations
    Tce_standoff = np.array([[-0.707, 0, 0.707, x],
                            [0, 1, 0, 0],
                            [-0.707, 0, -0.707, 0.3],
                            [0, 0, 0, 1]])

    Tce_grasp = np.array([[-0.707, 0, 0.707, x],
                        [0, 1, 0, 0],
                      [-0.707, 0, -0.707, 0],
                      [0, 0, 0, 1]])

    # the fixed offset from the chassis frame {b} to the base frame of the arm {0}
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0,      1]])

    # end-effector frame {e} relative to the arm base frame {0}
    M0e = np.array([[1, 0, 0,  0.033],
                   [0, 1, 0,      0],
                   [0, 0, 1, 0.6546],
                   [0, 0, 0,      1]])

    # the screw axes for the five joints in the end-effector frame {e}
    Blist = np.array([[0,  0, 1,       0, 0.033, 0],
                      [0, -1, 0, -0.5076,     0, 0],
                      [0, -1, 0, -0.3526,     0, 0],
                      [0, -1, 0, -0.2176,     0, 0],
                      [0,  0, 1,       0,     0, 0]]).T

    #The best proportional and integral gains for overshoot
    Kp = np.identity(6) * 2.0 
    Ki = np.identity(6) * 0.01


    k = 1
    max_speed_joints = 50  #50   
    max_speed_wheels = 50  #50   
    dt = 0.01 
    t = 20             
    log("Generating Trajectory")
    traj = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k)
    log("Trajectory Generated")
    config = np.zeros((len(traj), 13))
    Xerr_plot = np.zeros((len(traj), 6))
    config[0] = initial_configuration
    current_config = initial_configuration[:-1]
    Xerr_integ = np.zeros(6)
  
    for i in range(0, len(traj)-1):
        phi = current_config[0]
        x = current_config[1]
        y = current_config[2]
        Tsb = np.array([[np.cos(phi), -np.sin(phi), 0,      x],
                        [np.sin(phi),  np.cos(phi), 0,      y],
                        [          0,            0, 1, 0.0963],
                        [          0,            0, 0,      1]])
        current_joint_angle = current_config[3: 8]
        
        T0e = FKinBody(M0e, Blist, current_joint_angle)
        
        #current actual end-effector configuration
        X = Tsb@Tb0@T0e
        #current end-effector reference configuration
        Xd = np.array([[traj[i][0], traj[i][1], traj[i][2], traj[i][9]],
                       [traj[i][3], traj[i][4], traj[i][5], traj[i][10]],
                       [traj[i][6], traj[i][7], traj[i][8], traj[i][11]],
                       [              0,               0,               0,               1]])
        #end-effector reference configuration at the next timestep in the reference trajectory 
        Xd_next = np.array([[traj[i+1][0], traj[i+1][1], traj[i+1][2], traj[i+1][9]],
                            [traj[i+1][3], traj[i+1][4], traj[i+1][5], traj[i+1][10]],
                            [traj[i+1][6], traj[i+1][7], traj[i+1][8], traj[i+1][11]],
                            [                0,                 0,                 0,                 1]])
        V, control, Xerr, Xerr_integ = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt,current_config, Xerr_integ)
        log("Applying Feedbackcontrol") 
        current_config = NextState(current_config, control, dt, max_speed_joints, max_speed_wheels)
        config[i+1] = np.hstack((current_config, traj[i][12]))

        Xerr_plot[i] = Xerr
    log("Generating overshoot.csv file")
    np.savetxt("overshoot.csv", config, delimiter = ",")
    log("Generating X_error_overshoot.csv file")
    np.savetxt("X_error_overshoot.csv", Xerr_plot, delimiter = ",")

    log("Plotting the X error")
    x_lin = np.linspace(1, 20, len(traj))
    plt.figure()
    plt.plot(x_lin, Xerr_plot[:, 0], label='Xerror 0[Roll]')
    plt.plot(x_lin, Xerr_plot[:, 1], label='Xerror 1[Pitch]')
    plt.plot(x_lin, Xerr_plot[:, 2], label='Xerror 2[Yaw]')
    plt.plot(x_lin, Xerr_plot[:, 3], label='Xerror 3[X]')
    plt.plot(x_lin, Xerr_plot[:, 4], label='Xerror 4[Y]')
    plt.plot(x_lin, Xerr_plot[:, 5], label='Xerror 5[Z]')
    plt.title(f'X error with time with Kp= 2 and Ki= 0.01')
    plt.xlabel('Time(seconds)')
    plt.ylabel('Error[rad and m]')
    plt.legend(loc='best')
    plt.show()
 
    log("done")
if __name__ == "__main__":
    overshoot()

