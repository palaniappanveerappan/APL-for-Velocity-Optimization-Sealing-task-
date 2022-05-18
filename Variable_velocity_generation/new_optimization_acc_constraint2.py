# SPDX-License-Identifier: AGPL-3.0-or-later

from cProfile import label
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from genCurve import funCurve
from spiral_path import spiral


x_in = 298.234# 400.0 # [mm]
y_in = -157.108 # 50.0  # [mm]0.121476,-0.264584,0.406037
z_in =  402.773# 300.0  # [mm] 0.119557,-0.262372,0.402319      0.298234,-0.157108,0.402779

# gun inclination: here it is selected a vertical orientation
inclination = 0 * np.pi / 180 # [rad] - inclination angle of the gun
dt=0.001
acc_limit=50

y_teach = np.transpose(np.load("/home/franka/palani_ws/src/codes/scripts/VelocityPlanning_DMP_FL/Python_code/path_generation/test_19k_points.npz",'r+b')['arr_0'])
X=y_teach[:,0]
Y=y_teach[:,1]

plt.plot(X,Y)
plt.xlabel("X Axis")
plt.ylabel("Y Axis")
plt.title("Path")
plt.show()

x_t = np.gradient(X)
y_t = np.gradient(Y)

xx_t = np.gradient(x_t)
yy_t = np.gradient(y_t)

curv = (np.abs(xx_t * y_t - x_t * yy_t) / (x_t * x_t + y_t * y_t)**1.5)
radius_curv=np.zeros(len(curv))
for i in range(len(curv)-1):
    if curv[i]==0:
        radius_curv[i]=100
    else:
        radius_curv[i]=1/curv[i]
radius_curv[-2]=radius_curv[-3]
radius_curv[-1]=radius_curv[-2]
radius_curv[1]=radius_curv[2]
radius_curv[0]=radius_curv[1]


absc_teach = np.array([0])    #creating the set of absc
for i in range(len(X)-1):

    absc_teach = np.append(absc_teach,absc_teach[-1] + np.sqrt(((X[i+1]-X[i])**2) + ((Y[i+1]-Y[i])**2)))

length=absc_teach[-1]

for i in range(len(radius_curv)):    
    if radius_curv[i]>100:
        radius_curv[i]=100
    elif radius_curv[i]<11:
        radius_curv[i]=8
    else:
        radius_curv[i]=(radius_curv[i])
plt.plot(absc_teach,radius_curv,'b', label="radius_curv")
plt.legend()
plt.show()


velocity_optimized = np.transpose(np.load("/home/franka/palani_ws/src/codes/scripts/VelocityPlanning_DMP_FL/Python_code/path_generation/2022-03-16 04_09_24.733616.npy", 'r')).tolist()
plt.plot(velocity_optimized,'r',label="velocity implied, starting")
plt.legend()
plt.show()
#initial acceleration(Trapezoidal motion profile)
v_lim=velocity_optimized[0]

acc_lim=120    #accelaration limit for trapezoidal alone
t_acc=v_lim/acc_lim
t_tot = t_acc + length / v_lim
t = np.linspace(0,t_tot,int(t_tot/dt))
for i in range(len(t)):
    if t[i] < t_acc:
        velocity_optimized[i]=v_lim*t[i]


absc_des = np.cumsum(velocity_optimized) * dt   #desired absc values

# definition of the x and y desired points
x1_des = np.interp(absc_des,absc_teach,X)
x2_des = np.interp(absc_des,absc_teach,Y)

#y_des = np.column_stack((x1_des,x2_des))
x=(x1_des + x_in)/1000
y=(x2_des + y_in)/1000
z = (np.zeros(len(x)) + z_in)/ 1000 # [m]

plt.plot(x*1000, label="x after")
plt.plot(y*1000,label="y after")
plt.plot(X,label="X")
plt.plot(Y,label="Y")
plt.title("how x,y,X,Y changes after reffitting")
plt.legend()
plt.show()

absc_teach = np.array([0])    #creating the set of absc
vel=np.array([0])
for i in range(len(x)-1):

    absc_teach = np.append(absc_teach,absc_teach[-1] + np.sqrt(((x[i+1]-x[i])**2) + ((y[i+1]-y[i])**2)))
    vel = np.append(vel,((absc_teach[i]-absc_teach[i-1])/0.001))


plt.plot(x,y)
plt.plot(x[0],y[0],'x')
plt.plot(x[-3000],y[-3000],'o')
plt.title("Final path After reffitting")
plt.show()
#plt.plot(vel*1000,label="final velocity after converting to coordinates")

plt.plot(velocity_optimized,'r',label="velocity implied, starting")
plt.xlim([0,24500])
plt.ylim([0,85])
plt.ylabel("velocity in mm/sec")
#plt.legend()
plt.show()

x = np.append(x,x[-1] * np.ones(3000))
y = np.append(y,y[-1] * np.ones(3000))
z = np.append(z,np.linspace(z_in/1000,z_in/1000 + 0.04,3000))    #final liftting after task completion,4cms (ensure robot initial config allows this to happen)

############################################
##########euler to quarternion##############
############################################

def euler_to_quaternion(yaw, pitch, roll): # Z Y X
    qx = round(np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - 
               np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2),4)
    qy = round(np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + 
               np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2),4)
    qz = round(np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - 
               np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2),4)
    qw = round(np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + 
               np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2),4)
    return [qx, qy, qz, qw]

# Euler angles ZYX to quaternions
qx = qy = qz = qw = []

for i in range(len(x)-1):
    z_rot = np.arctan2((y[i+1]-y[i]),(x[i+1]-x[i])) + np.pi/4 # [rad]
    y_rot = np.pi + inclination # [rad]

    [qxi,qyi,qzi,qwi] = euler_to_quaternion(z_rot, y_rot, 0) # Z Y X
    
    qx = np.append(qx,qxi)
    qy = np.append(qy,qyi)
    qz = np.append(qz,qzi)
    qw = np.append(qw,qwi)

qx = np.append(qx,qxi)
qy = np.append(qy,qyi)
qz = np.append(qz,qzi)
qw = np.append(qw,qwi)


##################
######ROSNODE#####
##################

import rospy
import serial
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

ard = serial.Serial('/dev/ttyUSB0',115200,timeout=0)
time.sleep(2.5)
ard.write('90\n') #ard.write('99\n') 
time.sleep(1.2)
count = 0 # we don't give serial values each time step
pwm = 1

try:
    pub = rospy.Publisher('/DMP_pose', PoseStamped, queue_size=1)
    rospy.init_node('DMP_planner', anonymous=True)
    rate = rospy.Rate(1/dt) # 1kHz
    
    p = PoseStamped()

    task_ended = False
    
    while not rospy.is_shutdown():
        if not task_ended:
            for i in range(len(x)):
                p.pose.position.x = round(x[i],5);
                p.pose.position.y = round(y[i],5);
                p.pose.position.z = round(z[i],5);
                p.pose.orientation.x = round(qx[i],6);
                p.pose.orientation.y = round(qy[i],6);
                p.pose.orientation.z = round(qz[i],6);
                p.pose.orientation.w = round(qw[i],6);
                p.header.stamp.secs=rospy.get_time()
        	
            
                if i < (len(x)-3000): #5000
                    ard.write(str(90)) #63
                    ard.write('\n')
                    #if pwm is 1:
    		        #      ard.write(str(25))
    		        #      ard.write('\n')
    		        #      if count > duty[i] * 200:
    		        #          pwm = 0
    		        #          count = 0
                    #else:
    		        #      ard.write(str(8))
    		        #      ard.write('\n')
    		        #      if count > (1-duty[i]) * 200:
    		        #          pwm = 1
    		        #          count = 0    
                    #count += 1
                else:
                    ard.write('0\n')
                    
                pub.publish(p)
                rate.sleep()

            task_ended=True
            ard.write('0\n')
            ard.close()

        pub.publish(p)
        rate.sleep()

except rospy.ROSInterruptException:
    pass
