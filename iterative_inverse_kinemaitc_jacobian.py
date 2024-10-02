import numpy as np
from copy import deepcopy
from qpsolvers import solve_qp
from general_robotics_toolbox import *
from general_robotics_toolbox import tesseract as rox_tesseract
from general_robotics_toolbox import robotraconteur as rr_rox

# Define the robot
with open('ABB_1200_5_90_robot_default_config.yml', 'r') as file:
    robot = rr_rox.load_robot_info_yaml_to_robot(file)

# forward kinematics
# Generate random joint angles within the robot's joint limits
joint_angles = np.random.uniform(robot.joint_lower_limit+0.3, robot.joint_upper_limit-0.3)
flange_T = fwdkin(robot, joint_angles)

print("Forward kinematics:")
print(f'Joint angles: {np.round(np.degrees(joint_angles),2)}')
print('Flange pose:', flange_T)

print("=============================================================")
# inverse kinematics
# last_joints=np.zeros(6)
last_joints=None
q_all=robot6_sphericalwrist_invkin(robot,flange_T,last_joints)

print("Inverse kinematics:")
print('Flange pose:', flange_T)
print('Joint angles:')
for i in range(len(q_all)):
    print(np.round(np.degrees(q_all[i]),2))

### inverse kinematics using jacobian
target_flange_T = deepcopy(flange_T)
q_init = q_all[0] + np.random.uniform(-0.01, 0.01, 6)

q_iter = q_init
for i in range(100):
    print(f'Joint angles: {np.round(np.degrees(q_iter),2)}')
    flange_T = fwdkin(robot, q_iter)
    vd = target_flange_T.p-flange_T.p
    Rd = flange_T.R.T@target_flange_T.R
    k,theta = R2rot(Rd)
    omega_d = k*theta
    nu_d = np.concatenate((omega_d,vd))

    print("Iteration:", i, "Error:", np.linalg.norm(vd), 'Theta:', np.degrees(theta))
    if np.linalg.norm(vd) < 1e-6:
        break
    J = robotjacobian(robot, q_iter)

    H = J.T @ J
    H = (H + H.T) / 2
    f = -J.T @ nu_d
    qdot = solve_qp(H, f, solver="quadprog")

    # qdot = np.linalg.pinv(J) @ nu_d
    
    q_iter = q_iter + 0.1*qdot
    
