import numpy as np
from copy import deepcopy
from general_robotics_toolbox import *
from general_robotics_toolbox import tesseract as rox_tesseract
from general_robotics_toolbox import robotraconteur as rr_rox
import abb_motion_program_exec as abb
from matplotlib import pyplot as plt

# Define the robot
with open('ABB_1200_5_90_robot_default_config.yml', 'r') as file:
    robot = rr_rox.load_robot_info_yaml_to_robot(file)

tool_T = Transform(np.eye(3), np.array([0, 0, 100]))

# print(fwdkin(robot, np.radians([0,0,0,0,0,0]))*tool_T)

# square center at [600,0,550], side length 150
# Rotation are all np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]]).T

# center_T = Transform(np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]]).T, np.array([525, -75, 550]))
# print(np.degrees(robot6_sphericalwrist_invkin(robot,center_T*tool_T.inv())))

# Run it on the robot
my_tool = abb.tooldata(True,abb.pose([0,0,0.1],[1,0,0,0]),abb.loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0))
my_wobj = abb.wobjdata(False,True,"",abb.pose([600,0,550],[1,0,0,0]),abb.pose([0,0,0],[1,0,0,0]))

def quadrant(q,robot):
	cf146=np.floor(np.array([q[0],q[3],q[5]])/(np.pi/2))
	eef=fwdkin(robot,q).p
	
	REAR=(1-np.sign((rot([0,0,1],q[0])@np.array([1,0,0]))@np.array([eef[0],eef[1],eef[2]])))/2

	LOWERARM= q[2]<-np.pi/2
	FLIP= q[4]<0

	return np.hstack((cf146,[4*REAR+2*LOWERARM+FLIP])).astype(int)

mp = abb.MotionProgram(tool=my_tool,wobj=my_wobj)

### You may want to use moveabsj to move to the initial joint angles
### Provide the initial joint angles
# jt_init = abb.jointtarget(np.degrees(curve_js[0]),[0]*6) # 
# mp.MoveAbsJ(jt_init,abb.v1000,abb.fine)
###

# move robot to the four corners of the square
corner_p = np.array([[-75, -75, 0], [-75, 75, 0], [75, 75, 0], [75, -75, 0], [-75, -75, 0]])
corner_R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]]).T
for i in range(5):
    robt = abb.robtarget(corner_p[i],R2q(corner_R),abb.confdata(0,-1,-1,0),[0]*6) # create the robtarget, position (mm), orientation (quaternion)
    if i==0 or i==4:
        mp.MoveL(robt,abb.v150,abb.fine) # last zone is fine
    else:
        mp.MoveL(robt,abb.v150,abb.z50) # using zone = z50

client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")
log_results = client.execute_motion_program(mp) # run on the robot/robotstudio and log the results

cmd_num=log_results.data[:,1] # command number
start_id = np.where(cmd_num==2)[0][0]
curve_js_exe = np.radians(log_results.data[start_id:,2:8]) # executed joint angles
curve_p_exe = [] # executed flange position
for i in range(len(curve_js_exe)):
    curve_p_exe.append(fwdkin(robot, curve_js_exe[i]).p)
lam_exe = np.cumsum(np.linalg.norm(np.diff(curve_p_exe, axis=0), axis=1)) # executed path length
lam_exe = np.insert(lam_exe, 0, 0)

# visualize the joint trajectory
fig = plt.figure()
for i in range(6):
    plt.plot(lam_exe,curve_js_exe[:,i], label=f'Joint {i+1} executed', linestyle='--')
plt.legend()
plt.xlabel('Sample')
plt.ylabel('Joint angle (rad)')
plt.title('Joint trajectory')
plt.show()