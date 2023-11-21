import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt
import logging
from numpy import sin, cos
from constants import R, L, W, T_B0, M_OE, B_LIST, F, F_6, TSE_INI, TCE_GRP, TCE_STA, TIMESTEP, MAX_SPEED

LOG_FILENAME = 'output.log'
logging.basicConfig(filename=LOG_FILENAME, level=logging.DEBUG)

def create_array(source, start, num):
    res = []
    for i in range(start, start+num):
        res.append([source[i]])
    return np.asarray(res)

'''Calculates the NextState based on current position, movement constraints, and desired next configuration.'''
def NextState(current_config, speed, timestep, max_speed):
    chasis_config = create_array(current_config, 0, 3)
    arm_config = create_array(current_config, 3, 5)
    wheel_config = create_array(current_config, 8, 4)         

    # Limit speed
    limited_speed = np.clip(speed, -max_speed, max_speed)

    # Update joint and wheel configurations
    next_joint_angles = arm_config + limited_speed[:5].reshape(-1, 1) * timestep
    det_u = limited_speed[5:].reshape(-1, 1) * timestep
    next_wheel_angles = wheel_config + det_u

    # Compute next chasis configuration
    V_b = np.dot(F,det_u).reshape(3,)
    
    w_bz = V_b[0]
    v_bx = V_b[1]
    v_by = V_b[2]

    det_qb = np.zeros(3,)
    
    if w_bz < 1e-3:
        det_qb = np.array([[0],
                           [v_bx],
                           [v_by]])
    else:
        det_qb = np.array([[w_bz],
                           [v_bx * sin(w_bz) + v_by * (cos(w_bz) - 1)/w_bz],
                           [v_by * sin(w_bz) + v_bx * (1 - cos(w_bz))/w_bz]])

    # transform matrix for det_qb
    trans = np.array([[1, 0, 0],
                      [0, cos(current_config[0]), -sin(current_config[0])],
                      [0, sin(current_config[0]), cos(current_config[0])]])
    det_q = np.dot(trans, det_qb)
 
    next_chasis_config = chasis_config + det_q
    next_config = np.concatenate((next_chasis_config,next_joint_angles,next_wheel_angles), axis=None)
    
    return next_config

def matrix2list(whole_traj, traj, N, gripper_state):
    traj = np.asarray(traj)
    for i in range(N):
        # Flatten the rotation matrix and translation vector, then append gripper state
        row = np.concatenate((traj[i, :3, :3].flatten(), traj[i, :3, 3], [gripper_state]))
        whole_traj.append(row.tolist())

    return whole_traj

'''Generates a trajectory based on desired initial and final configurations.'''
def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff):
    # Constants
    Tf = 3
    method = 5

    # Calculate intermediate configurations
    Tse_init_standoff = np.dot(Tsc_initial, Tce_standoff)
    Tse_init_grasp = np.dot(Tsc_initial, Tce_grasp)
    Tse_fin_standoff = np.dot(Tsc_final, Tce_standoff)
    Tse_fin_grasp = np.dot(Tsc_final, Tce_grasp)

    # Sequence of configurations and states
    config_sequence = [
        (Tse_initial, Tse_init_standoff, 500, 0),  # Initial to Initial Standoff
        (Tse_init_standoff, Tse_init_grasp, 100, 0),  # Initial Standoff to Initial Grasp
        (Tse_init_grasp, Tse_init_grasp, 100, 1),  # Close Gripper
        (Tse_init_grasp, Tse_init_standoff, 100, 1),  # Initial Grasp to Initial Standoff
        (Tse_init_standoff, Tse_fin_standoff, 600, 1),  # Initial Standoff to Final Standoff
        (Tse_fin_standoff, Tse_fin_grasp, 100, 1),  # Final Standoff to Final Grasp
        (Tse_fin_grasp, Tse_fin_grasp, 100, 0),  # Open Gripper
        (Tse_fin_grasp, Tse_fin_standoff, 100, 0)  # Final Grasp to Final Standoff
    ]

    # Generate Trajectories
    whole_traj = []
    for start, end, N, gripper_state in config_sequence:
        traj = mr.CartesianTrajectory(start, end, Tf, N, method)
        whole_traj = matrix2list(whole_traj, traj, N, gripper_state)

    return whole_traj

'''Calculates the end-effector twist V expressed in the end-effector frame {e}'''
def FeedbackControl(X, Xd, Xd_next, Kp, Ki, timestep, integral, robot_config):
    # Extract joint angles from robot configuration
    theta_list = robot_config[3:8]

    # Compute the current end-effector pose
    T_0e = mr.FKinBody(M_OE, B_LIST, theta_list)

    # Calculate the desired end-effector twist (Vd)
    Vd = mr.se3ToVec((1/timestep) * mr.MatrixLog6(np.dot(mr.TransInv(Xd), Xd_next)))

    # Compute the adjoint representation for the current end-effector pose
    Ad_xxd = mr.Adjoint(np.dot(mr.TransInv(X), Xd))

    # Compute the error twist (X_err)
    X_err = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X), Xd)))
    integral += X_err * timestep

    # Calculate the control input (V)
    V = np.dot(Ad_xxd, Vd) + np.dot(Kp, X_err) + np.dot(Ki, integral)

    # Compute the Jacobian matrices for the arm and the base
    J_arm = mr.JacobianBody(B_LIST, theta_list)
    J_base = np.dot(mr.Adjoint(np.dot(mr.TransInv(T_0e), mr.TransInv(T_B0))), F_6)

    # Concatenate the Jacobians to form the full body Jacobian (Je)
    Je = np.concatenate((J_base, J_arm), axis=1)
    
    # Calculate the commanded velocities (command_V)
    command_V = np.linalg.pinv(Je).dot(V)

    return command_V, X_err


def transform_from_trajectory(traj, index):
    return np.array([[traj[index][0], traj[index][1], traj[index][2], traj[index][9]],
                     [traj[index][3], traj[index][4], traj[index][5], traj[index][10]],
                     [traj[index][6], traj[index][7], traj[index][8], traj[index][11]],
                     [0, 0, 0, 1]])

def get_chassis_transformation(robot_config):
    return np.array([[cos(robot_config[0]), -sin(robot_config[0]), 0, robot_config[1]],
                     [sin(robot_config[0]), cos(robot_config[0]), 0, robot_config[2]],
                     [0, 0, 1, 0.0963],
                     [0, 0, 0, 1]])

'''Main function to call NextState, GenerateTrajectory, and FeedbackControl with desired parameters.'''
def main(Tsc_ini, Tsc_fin, Kp, Ki, robot_config):
    # Initialize arrays for storing robot trajectory and error
    robot_traj = []
    X_err_arr = []
    integral = np.zeros((6,), dtype=float) 

    # Generate the trajectory from initial to final configurations
    traj = np.asarray(TrajectoryGenerator(TSE_INI, Tsc_ini, Tsc_fin, TCE_GRP, TCE_STA))
    robot_traj.append(robot_config.tolist())  # Add initial robot configuration to trajectory

    for i in range(1699): # Generate 1700 frames for a 17-second task
        theta_list = robot_config[3:8] 

        # Transform trajectory points into end-effector configurations
        Xd = transform_from_trajectory(traj, i)
        Xd_next = transform_from_trajectory(traj, i + 1)

        # Calculate the transformation matrix of the chassis
        Tsb = get_chassis_transformation(robot_config)
        T0e = mr.FKinBody(M_OE, B_LIST, theta_list)  # Compute end-effector pose
        X = np.dot(Tsb, np.dot(T_B0, T0e))  # Full transformation to end-effector

        # Compute the control input and error
        command_V, X_err = FeedbackControl(X, Xd, Xd_next, Kp, Ki, TIMESTEP, integral, robot_config)
        X_err_arr.append(X_err.tolist()) 

        # Combine arm and wheel commands
        control = np.concatenate((command_V[4:9], command_V[:4]))
        # Update robot configuration for next timestep
        robot_config = NextState(robot_config[:12], control, TIMESTEP, MAX_SPEED)
        # Store the updated robot configuration
        robot_current_traj = np.concatenate((robot_config, [traj[i][12]]))
        robot_traj.append(robot_current_traj.tolist())

    saveDataAndPlotErrors(X_err_arr, robot_traj)

def saveDataAndPlotErrors(error_array, robot_trajectory):
    logging.debug("Saving error array and robot trajectory data to CSV files.")
    np.savetxt("Error_Array.csv", error_array, delimiter=',')
    np.savetxt("Robot_Trajectory.csv", robot_trajectory, delimiter=',')

    plotErrors(error_array)

def plotErrors(error_array):
    time_steps = np.linspace(0, 16.99, len(error_array))
    X_err = np.asarray(error_array)
    plt.plot(time_steps, X_err)
    plt.title("Error Plot Over Time")
    plt.xlim([0, 17])
    plt.xlabel("Time (in seconds)")
    plt.ylabel("Error Value")
    plt.legend([f'Error [{i}]' for i in range(6)])
    plt.grid(True)
    plt.savefig("Error Graph", dpi = 300)
    plt.show()

'''Example input'''
'''There are three versions, uncomment the one you would like to use.'''
'''Best'''
kp = 30 
ki = 0.75
Tsc_ini = np.array([[1, 0, 0, 1],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])
Tsc_fin = np.array([[0, 1, 0, 0],
                    [-1, 0, 0, -1],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])

'''Overshoot'''
# kp = 0.9
# ki = 0.001
# Tsc_ini = np.array([[1, 0, 0, 1],
#                     [0, 1, 0, 0],
#                     [0, 0, 1, 0.025],
#                     [0, 0, 0, 1]])
# Tsc_fin = np.array([[0, 1, 0, 0],
#                     [-1, 0, 0, -1],
#                     [0, 0, 1, 0.025],
#                     [0, 0, 0, 1]])

'''newTask'''
# kp = 25
# ki = 0.15
# Tsc_ini = np.array([[0, -1, 0, 1],
#                     [1, 0, 0, 1.0],
#                     [0, 0, 1, 0.025],
#                     [0, 0, 0, 1]])
# Tsc_fin = np.array([[2, 0, 0, 2],
#                     [0, 1, 0, 0],
#                     [0, 0, 1, 0.025],
#                     [0, 0, 0, 1]])

 
Kp = np.diag([kp,kp,kp,kp,kp,kp])
Ki = np.diag([ki,ki,ki,ki,ki,ki])

#Initial configuration of robot
robot_config = np.array([0.1, -0.2, 0.2, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])

#Call main function
main(Tsc_ini,Tsc_fin,Kp,Ki,robot_config)
