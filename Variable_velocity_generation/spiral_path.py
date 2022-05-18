
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d


def funCurve(x_c,y_c,R,th_start,th_arc):
    
    length = R * abs(th_arc) # arc length
    points = round(length)*19 # n points to generate for this arc segment
    
    x = []
    y = []
    
    for i in  range(int(points)):
        th = th_start + th_arc/points * i
        x = np.append(x, x_c + R * np.cos(th))
        y = np.append(y, y_c + R * np.sin(th))

    return x,y

def spiral():

    X1,Y1=funCurve(0,0,10,np.pi/2,np.pi/2)
    X2,Y2=funCurve(5,0,15,np.pi,np.pi/2)
    X=np.append(X1,X2)
    Y=np.append(Y1,Y2)
    X3,Y3=funCurve(5,5,20,np.pi*3/2,np.pi/2)
    X=np.append(X,X3)
    Y=np.append(Y,Y3)
    X4,Y4=funCurve(0,5,25,np.pi*0,np.pi/2)
    X=np.append(X,X4)
    Y=np.append(Y,Y4)
    X5,Y5=funCurve(0,0,30,np.pi/2,np.pi/2)
    X=np.append(X,X5)
    Y=np.append(Y,Y5)
    X6,Y6=funCurve(5,0,35,np.pi,np.pi/2)
    X=np.append(X,X6)
    Y=np.append(Y,Y6)
    X7,Y7=funCurve(5,5,40,np.pi*3/2,np.pi/2)
    X=np.append(X,X7)
    Y=np.append(Y,Y7)
    X8,Y8=funCurve(0,5,45,np.pi*0,np.pi/2)
    X=np.append(X,X8)
    Y=np.append(Y,Y8)
    X9,Y9=funCurve(0,0,50,np.pi/2,np.pi/2)
    X=np.append(X,X9)
    Y=np.append(Y,Y9)
    X10,Y10=funCurve(5,0,55,np.pi,np.pi/2)
    X=np.append(X,X10)
    Y=np.append(Y,Y10)
    X11,Y11=funCurve(5,5,60,np.pi*3/2,np.pi/2)
    X=np.append(X,X11)
    Y=np.append(Y,Y11)
    

    return X,Y
X,Y=spiral()
np.savez('spiral',[X,Y])