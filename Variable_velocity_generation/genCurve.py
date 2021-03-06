# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import math
import matplotlib.pyplot as plt



def funCurve(x_c,y_c,R,th_start,th_arc):
    
    length = R * abs(th_arc) # arc length
    points = round(length)*25 # n points to generate for this arc segment
    
    x = []
    y = []
    
    for i in  range(int(points)):
        th = th_start + th_arc/points * i
        x = np.append(x, x_c + R * np.cos(th))
        y = np.append(y, y_c + R * np.sin(th))

    return x,y


