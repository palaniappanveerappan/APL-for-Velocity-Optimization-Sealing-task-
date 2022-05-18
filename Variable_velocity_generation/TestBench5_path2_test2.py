# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import math
import matplotlib.pyplot as plt
from genStraight import funStraight
#from genSawTooth import funSawtooth
from genCurve import funCurve 
from scipy.interpolate import interp1d

dt=0.001
# straight edges
[a,b] = funStraight(0,0,50,0)
X_pts = a
Y_pts = b
#X_pts = np.array([0,50])
#Y_pts = np.array([0,0])

[a,b] = funCurve(50,-5,5,np.pi/2,-np.pi/2)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

# curve up-down
updown_rep = 4
straight = 20
radius = 5
for i in range(updown_rep):
    
    [a,b] = funStraight(X_pts[-1],Y_pts[-1],X_pts[-1],Y_pts[-1]-straight)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    
    [a,b] = funCurve(X_pts[-1]+radius,Y_pts[-1],radius,-np.pi,np.pi)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    
    [a,b] = funStraight(X_pts[-1],Y_pts[-1],X_pts[-1],Y_pts[-1]+straight)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    
    [a,b] = funCurve(X_pts[-1]+radius,Y_pts[-1],radius,-np.pi,-np.pi)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)

# smooth curves
[a,b] = funCurve(150,-5,15,-np.pi,np.pi/2)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

[a,b] = funCurve(150,-60,40,np.pi/2,-np.pi)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

[a,b] = funStraight(X_pts[-1],Y_pts[-1],X_pts[-1]-50,Y_pts[-1])
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)
#X_pts = np.append(X_pts, 100)
#Y_pts = np.append(Y_pts, -100)

[a,b] = funCurve(100,-90,10,-np.pi/2,-np.pi)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)
    
[a,b] = funCurve(100,-70,10,-np.pi/2,np.pi)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

[a,b] = funStraight(X_pts[-1],Y_pts[-1],X_pts[-1]-25,Y_pts[-1])
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

# Sawtooth characteristics
base = 20.0
height = 40.0
repetition = 3
# 0 for Horizontal, 1 for Vertical
direction = 1
# 0 for positive verso, 1 for negative verso
verso = 1

#(x,y) = funSawtooth(base,height,repetition,direction,verso,X_pts[-1],Y_pts[-1])
#X_pts = np.append(X_pts,x)
#Y_pts = np.append(Y_pts,y)
for i in range(repetition):
    [a,b] = funStraight(X_pts[-1],Y_pts[-1],X_pts[-1]-height,Y_pts[-1]-base)
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
    [a,b] = funStraight(X_pts[-1],Y_pts[-1],X_pts[-1]+height,Y_pts[-1])
    X_pts = np.append(X_pts, a)
    Y_pts = np.append(Y_pts, b)
   

[a,b] = funStraight(X_pts[-1],Y_pts[-1],75,-150,div=10)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

[a,b] = funStraight(X_pts[-1],Y_pts[-1],10,-150,div=10)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

[a,b] = funCurve(10,-140,10,-np.pi/2,-np.pi/2)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)
[a,b] = funStraight(X_pts[-1],Y_pts[-1],0,0,div=10)
X_pts = np.append(X_pts, a)
Y_pts = np.append(Y_pts, b)

np.savez('test2_1',[X_pts,Y_pts])
plt.figure(1)
plt.grid()
plt.title('Generic Path [dim: mm]')
plt.plot(X_pts,Y_pts,'o-')

plt.get_current_fig_manager().window.showMaximized()
plt.show()
y_teach = np.transpose(np.load("test2_1.npz",'r+b')['arr_0'])
for i in range(len(y_teach[0,:])):
    if y_teach[0,i] == y_teach[-1,i]: y_teach[-1,i] += 0.05 # [mm]

absc_teach = np.array([0]) # curvilinea abscissa of the teached path
for i in range(len(y_teach[:,0])-1):
    
    absc_teach = np.append(absc_teach,absc_teach[-1] + np.sqrt(
        (y_teach[i+1,0]-y_teach[i,0])**2 + (y_teach[i+1,1]-y_teach[i,1])**2))
    
    # check that 2 consecutive abscissa evaluations are not equal:
    # this don't allow to properly use the interpolation tool
    if absc_teach[-1] == absc_teach[-2]: absc_teach[-1] += 1e-5
    
# notice that the final point of the curvilinea abscissa is also the path length
len_teach = absc_teach[-1]
print("\n The teached path length is: ",np.round(len_teach,2)," mm") 

pt_density = len_teach/100 # [pts/mm] - initial refitting density 67


n_new = int(round(len_teach*pt_density)) # total number of points wished after refitting

# we want to refit the points in order to ensure that they are EQUALLY spaced 
# along the curvilinea abscissa (x1 = cartesian X, x2 = cartesian Y)

# EQUAL SPACING ALONG ABSCISSA, no time
x1 = np.interp(np.linspace(0,len_teach,n_new),absc_teach,y_teach[:,0])
x2 = np.interp(np.linspace(0,len_teach,n_new),absc_teach,y_teach[:,1])

np.savez('test_568_points',[x1,x2])
# final plotting
print(len(x1))
plt.figure(2)
plt.grid()
plt.title('Generic Path [dim: mm]')
plt.plot(x1,x2,'o-')

plt.get_current_fig_manager().window.showMaximized()
plt.show()


x_t = np.gradient(x1)
y_t = np.gradient(x2)

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


#####################
######APL_Data#######
#####################

rad_x=np.linspace(10,100,16)
rad_x_new=np.linspace(10,100,91)

vel_y1=[40,44,52,54,60,62,62,66,64,60,70,62,72,68,68,60]    #y1=[60,68,68,72,62,70,60,64,66,62,62,60,54,52,44,40]
vel_y2=[52,56,58,62,76,74,78,78,84,84,80,80,88,82,86,84]    #y2=[84,86,82,88,80,80,84,84,78,78,74,76,62,58,56,52]
vel_y3=[46,48,56,54,60,64,66,74,76,74,76,78,80,78,80,80]    #y3=[80,80,78,80,78,76,74,76,74,66,64,60,54,56,48,46]

f=interp1d(rad_x,vel_y3, kind='cubic')
f_lower=interp1d(rad_x,vel_y1, kind='cubic')
velocity=np.zeros(len(radius_curv))
velocity_lower=np.zeros(len(radius_curv))
for i in range(len(radius_curv)):
    if radius_curv[i]>100:
        velocity[i]=80
        velocity_lower[i]=60
    elif radius_curv[i]<11:
        velocity[i]=46
        velocity_lower[i]=40
    else:
        velocity[i]=f(radius_curv[i])
        velocity_lower[i]=f_lower(radius_curv[i])

#initial acceleration(Trapezoidal motion profile)
v_lim=velocity[0]
acc_lim=120    #accelaration limit for trapezoidal alone
t_acc=v_lim/acc_lim
t_tot = t_acc + len_teach / v_lim
t = np.linspace(0,t_tot,200)      #t = np.linspace(0,t_tot,int(t_tot/dt))
print(len(t))
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
plt.plot(radius_curv,'b', label="radius_curv")
plt.legend()
plt.show()
plt.plot(velocity,'r', label="velocity")
plt.xlabel("Path Points")
plt.ylabel("velocity in mm/sec")
plt.title("Velocity value before applying acceleration constarints")
plt.legend()
plt.show()