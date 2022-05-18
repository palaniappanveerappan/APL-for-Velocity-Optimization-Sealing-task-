#! /usr/bin/env python
# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import math
import matplotlib.pyplot as plt
#from scipy import interpolate


dt = 0.001  # [s] - time step of the robot controller

x_off =  224.873 # 400.0 # [mm]
y_off =  -469.487 # 50.0  # [mm]
z_off =  406.61 # 300.0 # [mm]0.227453,-0.470547,0.400436  0.224873,-0.469487,0.40661
#0.224885,-0.469695,0.408302

inclination = 0 * np.pi / 180 # [rad] - inclination angle of the gun


v_target = 10.0 # [mm/s]
#acc = 80.0      # [mm/s^2]
space = 500.0   # [mm] - total distance

t_acc = 1 #(v_target / acc)
t_tot = t_acc + space / v_target
t = np.linspace(0,t_tot,int(t_tot/dt))


########
# PATH #
########

path = np.zeros(len(t)) # straight movement
#flow_path = np.zeros(len(t)) # flow for the caulking gun [values: 0-9]
vel=np.zeros(len(t))

for i in range(1,len(t)):
    
    if t[i] < t_acc:
        vel_acc=v_target* t[i]
        path[i] = path[i-1] + vel_acc*dt
        vel[i]=vel_acc
    elif t[i] > t_tot - t_acc:
        vel_dec=v_target* (t_tot-t[i])
        path[i] = path[i-1] + vel_dec* dt
        vel[i]=vel_dec
    else:
        vel_cons=v_target 
        path[i] = path[i-1] + vel_cons* dt	
        vel[i]=vel_cons


###################################
#####generating straight line######
###################################

def Straight(x_in,y_in,x_fin,y_fin,t):
    
    length = np.sqrt((x_fin-x_in)**2 + (y_in-y_fin)**2)
    #N = int(round(length*100)) # n points to generate for this segment
    
    #t = np.linspace(0,length,N)
    x = np.interp(t,[0,length],[x_in,x_fin])
    y = np.interp(t,[0,length],[y_in,y_fin])

    return x,y

X,Y = Straight(x_off,y_off,(x_off-space),(y_off),t) 

#X,Y=np.transpose(X,Y)

plt.figure(0)
plt.plot(X,Y)
plt.plot(X[1],Y[1], 'x')


x1=(X-path)/1000
x2=(Y)/1000
x3 = (np.zeros(len(x1)) + z_off) / 1000 # [m]
#print(len(x1))
#print(len(x2))
#print(len(t))

plt.figure(1)
plt.axis('equal')
plt.grid()
plt.plot(t,vel) #to see the trapezoidal profile
plt.title('velocity profile')
plt.show()

plt.figure(2)
plt.grid()
plt.plot(x1)       #to see the smooth transition in the starting and ending
plt.title('path smoothness, just x')
#plt.show()

plt.figure(3)
plt.grid()
plt.plot(x1,x2) 
plt.plot(x1[1],x2[1],'X')
plt.title('Path')      
#plt.show()


x = np.append(x1,x1[-1] * np.ones(3000))
y = np.append(x2,x2[-1] * np.ones(3000))
z = np.append(x3,np.linspace(z_off/1000,z_off/1000 + 0.04,3000))    #final liftting after task completion,4cms (ensure robot initial config allows this to happen)

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


####################
####SILICONFLOW#####
####################

#v_x_fin = np.diff(x) / dt
#v_y_fin = np.diff(y) / dt
#v_x_fin = np.append(v_x_fin,v_x_fin[-1])
#v_y_fin = np.append(v_y_fin,v_y_fin[-1])

#v_final = np.sqrt(v_x_fin**2 + v_y_fin**2)

# EXPERIMENTAL DUTY CYCLE - VELOCITY RELATIONSHIP
#duty_ref = np.array([0.02, 0.08, 0.22, 0.80, 0.9, 1.0]) # [%] 
#ee_v_ref = np.array([0.0, 20.0, 35.0, 50.0, 65.0, 80.0]) / 1000 # [mm]

#f_duty = interpolate.interp1d(ee_v_ref,duty_ref,kind="slinear")
#duty = f_duty(v_final)


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
ard.write('70\n') #ard.write('99\n')
time.sleep(0.9)
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
        	
            
                if i < (len(x)-3200):
                    ard.write(str(80)) #63
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


