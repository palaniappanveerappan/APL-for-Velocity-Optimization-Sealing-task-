from cProfile import label
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

x=np.linspace(10,100,16)
x_new=np.linspace(10,100,91)
print(x_new)
y1=[40,44,52,54,60,62,62,66,64,60,70,62,72,68,68,60]    #y1=[60,68,68,72,62,70,60,64,66,62,62,60,54,52,44,40]

y2=[52,56,58,62,76,74,78,78,84,84,80,80,88,82,86,84]    #y2=[84,86,82,88,80,80,84,84,78,78,74,76,62,58,56,52]

y3=[46,48,56,54,60,64,66,74,76,74,76,78,80,78,80,80]    #y3=[80,80,78,80,78,76,74,76,74,66,64,60,54,56,48,46]

f=interp1d(x,y3, kind='cubic')
fig, ax = plt.subplots()


ax.fill_between(x, y1, y2, alpha=.5, linewidth=0, label="acceptable")
ax.plot(x, y3, linewidth=2, label="optimal")
ax.plot(x_new,f(x_new),'--', label="cubic interpolated") 
ax.set(xlim=(10, 100), xticks=np.arange(10, 100, 6), ylim=(30, 100), yticks=np.arange(30, 100, 5))
plt.xlabel("Curve Radius in mm")
plt.ylabel("Velocity of end effector in mm/sec")
plt.title("Optimal and acceptable velocity limits for different radius curves")
plt.legend()
plt.show()






