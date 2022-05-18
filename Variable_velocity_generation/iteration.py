import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd

x=np.arange(11)
print(len(x))
pref=[0,1,0,1,0,1,1,1,0,1,1,0]
y=np.zeros(len(x))
for i in range(len(x)):
    if pref[i]==1:
        y[i]=i
    else:
        y[i]=y[i-1]

plt.figure()
plt.plot(x,y)
plt.xlabel('Iterations')
plt.ylabel('Best Iteration')
plt.show()

y1=[0,0,0,0,0,1,1,1,1,1,1]
plt.figure()
plt.plot(x,y1,'*')
plt.xlabel('Iterations')
plt.ylabel('Acceptability')
plt.show()

