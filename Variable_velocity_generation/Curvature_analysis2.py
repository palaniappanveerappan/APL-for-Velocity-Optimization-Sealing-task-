# SPDX-License-Identifier: AGPL-3.0-or-later

from cProfile import label
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from genCurve import funCurve
from spiral_path import spiral


x_in = 121.313# 400.0 # [mm]
y_in = -264.579 # 50.0  # [mm]0.121476,-0.264584,0.406037
z_in =  403.043# 300.0  # [mm] 0.451493,-0.348231,0.402338

# gun inclination: here it is selected a vertical orientation
inclination = 0 * np.pi / 180 # [rad] - inclination angle of the gun
dt=0.001

###################
#######path########
###################
#X,Y=spiral()
y_teach = np.transpose(np.load("test1.npz",'r+b')['arr_0'])
X=y_teach[:,0]
Y=y_teach[:,1]

plt.plot(X,Y)
plt.xlabel("X Axis")
plt.ylabel("Y Axis")
plt.title("Path")
plt.show()

absc_teach = np.array([0])    #creating the set of absc
for i in range(len(X)-1):

    absc_teach = np.append(absc_teach,absc_teach[-1] + np.sqrt(((X[i+1]-X[i])**2) + ((Y[i+1]-Y[i])**2)))

length=absc_teach[-1]
###########################
#####Curvature_analysis####
###########################


x_t = np.gradient(X)
y_t = np.gradient(Y)

#so_called_vel = np.array([ [x_t[i], y_t[i]] for i in range(x_t.size)])

#so_called_speed = np.sqrt(x_t * x_t + y_t * y_t)

#tangent = np.array([1/so_called_speed] * 2).transpose() * so_called_vel

#ss_t = np.gradient(so_called_speed)
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

#plt.plot(radius_curv)
#plt.xlabel("Path Points")
#plt.ylabel("Curve Radius in mm")
#plt.title("Radius of Curvature value in each point of the Path")
#plt.show()

#####################
######APL_Data#######
#####################

rad_x=np.linspace(10,100,16)
rad_x_new=np.linspace(10,100,91)

vel_y1=[40,44,52,54,60,62,62,66,64,60,70,62,72,68,68,60]    #y1=[60,68,68,72,62,70,60,64,66,62,62,60,54,52,44,40]

vel_y2=[52,56,58,62,76,74,78,78,84,84,80,80,88,82,86,84]    #y2=[84,86,82,88,80,80,84,84,78,78,74,76,62,58,56,52]

vel_y3=[46,48,56,54,60,64,66,74,76,74,76,78,80,78,80,80]    #y3=[80,80,78,80,78,76,74,76,74,66,64,60,54,56,48,46]

f=interp1d(rad_x,vel_y3, kind='cubic')
velocity=np.zeros(len(radius_curv))
for i in range(len(radius_curv)):
    if radius_curv[i]>100:
        velocity[i]=85
    elif radius_curv[i]<11:
        velocity[i]=40
    elif radius_curv[i]<2:
        velocity[i]=70
    else:
        velocity[i]=f(radius_curv[i])


    
#initial acceleration
v_lim=velocity[0]
v_lim_dec=velocity[-1]
acc_lim=120
t_acc=v_lim/acc_lim
t_tot = t_acc + length / v_lim
t = np.linspace(0,t_tot,int(t_tot/dt))
for i in range(len(t)):
    if t[i] < t_acc:
        velocity[i]=v_lim*t[i]
    
    
#just for printing corresponding velocity for the radius of curvature
for i in range(len(radius_curv)):    
    if radius_curv[i]>100:
        radius_curv[i]=100
    elif radius_curv[i]<11:
        radius_curv[i]=8
    else:
        radius_curv[i]=(radius_curv[i])
    
plt.plot(velocity,'r', label="velocity")
plt.plot(radius_curv,'b', label="radius_curv")
plt.xlabel("Path Points")
plt.ylabel("velocity in mm/sec")
plt.title("Velocity value in each point of the Path for the corresponding Radius of Curvature")
plt.legend()
plt.show()


#########################
###Velocity_reference####
#########################

absc_des = np.cumsum(velocity) * dt   #desired absc values

# definition of the x and y desired points
x1_des = np.interp(absc_des,absc_teach,X)
x2_des = np.interp(absc_des,absc_teach,Y)

#y_des = np.column_stack((x1_des,x2_des))
a=(x1_des + x_in)/1000
b=(x2_des + y_in)/1000
c = (np.zeros(len(a)) + z_in)/ 1000 # [m]

#plt.plot(a*1000, label="a")
#plt.plot(b*1000,label="b")
#plt.plot(X,label="x")
#plt.plot(Y,label="y")
#plt.title("how a,b,x,y changes after reffitting")
#plt.legend()
#plt.show()

x = np.append(a,a[-1] * np.ones(3000))
y = np.append(b,b[-1] * np.ones(3000))
z = np.append(c,np.linspace(z_in/1000,z_in/1000 + 0.04,3000))    #final liftting after task completion,4cms (ensure robot initial config allows this to happen)

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
plt.plot(vel*1000)
plt.plot(velocity,'r')
plt.show()
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
        	
            
                if i < (len(x)-3300):
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