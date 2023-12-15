"""cut and paste this to run the code but has to be in the same directory:
 python3 milestone3.py """
import numpy as np
from modern_robotics import *

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, config, Xerr_prev=np.zeros(6)):
    # Calculating error between actual and desired end-effector configuration
    X_err_se3 = MatrixLog6(TransInv(X) @ Xd)
    X_err = se3ToVec(X_err_se3)
    
    # Calculating desired end-effector twist
    Vd_se3 = MatrixLog6(TransInv(Xd) @ Xd_next) / dt
    Vd = se3ToVec(Vd_se3)
    
    # Calculating the actual end-effector twist
    Adjoint_X_Xd = Adjoint(TransInv(X) @ Xd)
    V = Adjoint_X_Xd @ Vd + Kp @ X_err + Ki @ (Xerr_prev + X_err * dt)
    r = 0.0475
    l = 0.47/2
    w = 0.3/2
    F = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                        [1, 1, 1, 1],
                        [-1, 1, -1, 1]])
    
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0, 1]])
    
    Mo = np.array([[1, 0, 0,  0.033],
                   [0, 1, 0,  0],
                   [0, 0, 1,  0.6546],
                   [0, 0, 0,  1]])
    
    Blist = np.array([[0,  0, 1,       0, 0.033, 0],
                      [0, -1, 0, -0.5076,     0, 0],
                      [0, -1, 0, -0.3526,     0, 0],
                      [0, -1, 0, -0.2176,     0, 0],
                      [0,  0, 1,       0,     0, 0]]).T
    
    thetalist = config[3:8]
    T0e = FKinBody(Mo, Blist, thetalist)
    
    F6 = np.array([[0, 0, 0, 0], 
                   [0, 0, 0, 0],
                   F[0],
                   F[1], 
                   F[2], 
                   [0, 0, 0, 0]])

    Jb = Adjoint(TransInv(T0e) @ TransInv(Tb0)) @ F6
    Jarm = JacobianBody(Blist, thetalist)
    Je = np.hstack([Jb, Jarm])
    speed = np.linalg.pinv(Je, 1e-3)@V
    return V,speed,X_err,Xerr_prev+X_err*dt

# def main():
#     # Example inputs for testing the function
#     X = np.array([[0.170, 0, 0.985, 0.387], [0, 1, 0, 0], [-0.985, 0, 0.170, 0.570], [0, 0, 0, 1]])
#     Xd = np.array([[0, 0, 1, 0.5], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])
#     Xd_next = np.array([[0, 0, 1, 0.6], [0, 1, 0, 0], [-1, 0, 0, 0.3], [0, 0, 0, 1]])
#     Kp = np.identity(6)
#     Ki = np.zeros([6, 6])
#     Xerr_integ = np.zeros(6)
#     config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
    
#     # Calling the feedback control function
#     V = FeedbackControl(X, Xd, Xd_next, Kp, Ki, 0.01, config, Xerr_integ)

# if __name__ == "__main__":
#     main()

