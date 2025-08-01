import numpy as np
from general_robotics_toolbox import *
import abb_motion_program_exec as abb
from matplotlib import pyplot as plt

def quadrant(q,robot):
	cf146=np.floor(np.array([q[0],q[3],q[5]])/(np.pi/2))
	eef=fwdkin(robot,q).p
	
	REAR=(1-np.sign((rot([0,0,1],q[0])@np.array([1,0,0]))@np.array([eef[0],eef[1],eef[2]])))/2

	LOWERARM= q[2]<-np.pi/2
	FLIP= q[4]<0

	return np.hstack((cf146,[4*REAR+2*LOWERARM+FLIP])).astype(int)

# Run it on the robot
my_tool = abb.tooldata(True,abb.pose([0,0,0.1],[1,0,0,0]),abb.loaddata(0.001,[0,0,0.001],[1,0,0,0],0,0,0))

mp = abb.MotionProgram(tool=my_tool)
jt_init = abb.jointtarget(np.degrees(curve_js[0]),[0]*6) # using moveabsj to move to the initial joint angles

target0 = abb.robtarget(abb.pose([0.5,0,0.5],[1,0,0,0]),abb.confdata(0,0,0,0),[0]*6)
target1 = abb.robtarget(abb.pose([0.5,0.5,0.5],[1,0,0,0]),abb.confdata(0,0,0,0),[0]*6)
target2 = abb.robtarget(abb.pose([0,0.5,0.5],[1,0,0,0]),abb.confdata(0,0,0,0),[0]*6)
target3 = abb.robtarget(abb.pose([0,0,0.5],[1,0,0,0]),abb.confdata(0,0,0,0),[0]*6)

speed = abb.v50
zone = abb.z50

mp.MoveAbsJ(jt_init,abb.v1000,abb.fine)
mp.MoveL(target0, speed, zone)
mp.MoveL(target1, speed, zone)
mp.MoveL(target2, speed, zone)
mp.MoveL(target3, speed, abb.fine)

# execution
client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")
log_results = client.execute_motion_program(mp) # run on the robot/robotstudio and log the results

# data extraction and visualization
cmd_num=log_results.data[:,1] # command number
start_id = np.where(cmd_num==2)[0][0]
curve_js_exe = np.radians(log_results.data[start_id:,2:8]) # executed joint angles

# visualize the joint trajectory
fig = plt.figure()
for i in range(6):
    plt.plot(curve_js_exe[:,i], label=f'Joint {i+1} executed', linestyle='--')
plt.legend()
plt.xlabel('Sample')
plt.ylabel('Joint angle (rad)')
plt.title('Joint trajectory')
plt.show()