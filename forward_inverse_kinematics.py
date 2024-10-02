import numpy as np
from general_robotics_toolbox import *
from general_robotics_toolbox import tesseract as rox_tesseract
from general_robotics_toolbox import robotraconteur as rr_rox

# Define the robot
with open('ABB_1200_5_90_robot_default_config.yml', 'r') as file:
    robot = rr_rox.load_robot_info_yaml_to_robot(file)

# forward kinematics
# Generate random joint angles within the robot's joint limits
joint_angles = np.random.uniform(robot.joint_lower_limit, robot.joint_upper_limit)
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

                
