#!/usr/bin/env python

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FixedLocator, FormatStrFormatter
import matplotlib.pyplot as plt
import numpy as np

x,y,thetas,tangent = np.loadtxt("find_overshoot_contours.csv", delimiter=",", unpack=True)

fig = plt.figure()
ax = Axes3D(fig)

u_x = np.unique(y)
u_y = np.unique(thetas)

x_data = y
y_data = thetas

#surf = ax.plot_surface(x_data, y_data, t_data)
ax.scatter3D(x_data, y_data, tangent)
ax.set_xlabel("Lateral Offset (m)")
ax.set_ylabel("Initial Heading Error (rads)")
ax.set_zlabel("Tangential Dist to less than acceptable error (m)")

x_data, y_data = np.meshgrid(u_x, u_y)
t_data = tangent.reshape(x_data.shape)

fig2 = plt.figure()
CS = plt.contourf(x_data, y_data, t_data, 3)
#plt.clabel(CS, inline=1, fontsize=10)
plt.xlabel("Lateral Offset (m)")
plt.ylabel("Initial Heading Error (rads)")

plt.show()
