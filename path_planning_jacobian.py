import numpy as np
from copy import deepcopy
from qpsolvers import solve_qp
from general_robotics_toolbox import *
from general_robotics_toolbox import tesseract as rox_tesseract
from general_robotics_toolbox import robotraconteur as rr_rox
from matplotlib import pyplot as plt

from traj_gen import get_trajectory

# Define the robot
with open('ABB_1200_5_90_robot_default_config.yml', 'r') as file:
    robot = rr_rox.load_robot_info_yaml_to_robot(file)

# Define a curve in 3D space with postion and orientation in the rbots base frame
# sample a parabolic curve in 3D space
final_l = 300 # 300 mm curve
x_sample_range = np.linspace(0, final_l, int(final_l/0.1)+1) # sample every 0.1 mm
curve_p, curve_n = get_trajectory(x_sample_range)

# fix orientation R
curve_R = []
for i in range(len(curve_p)):
    if len(curve_R)==0:
        curve_R.append(np.array([[0, 0, -1], [0, 1, 0], [0, 0, -1]]).T)
    else:
        y_axis = curve_p[i]-curve_p[i-1]
        y_axis = y_axis/np.linalg.norm(y_axis)
        z_axis = curve_n[i]
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis/np.linalg.norm(x_axis)
        curve_R.append(np.vstack((x_axis, y_axis, z_axis)).T)

# visualize the 3D curve
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(curve_p[:,0], curve_p[:,1], curve_p[:,2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

# get the initial joint angles
q_init = robot6_sphericalwrist_invkin(robot,Transform(curve_R[0], curve_p[0]),np.zeros*6)[0]

# get joint trajectory using 6 dof constraints (curve_p and curve_R, i.e. position and orientation)
q_iter = q_init
for i in range(len(curve_p)):
    flange_T = fwdkin(robot, q_iter)
    vd = target_flange_T.p-flange_T.p
    Rd = target_flange_T.R@flange_T.R.T
    # Rd = flange_T.R@target_flange_T.R.T
    k,theta = R2rot(Rd)
    if theta<1e-10:
        omega_d = np.zeros(3)
    else:
        omega_d=np.sin(theta/2)*k # You can use any s error function here
    nu_d = np.concatenate((omega_d*angle_weight,vd))