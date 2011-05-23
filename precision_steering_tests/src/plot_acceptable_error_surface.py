#!/usr/bin/env python

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FixedLocator, FormatStrFormatter
import matplotlib.pyplot as plt
import numpy as np

x,y,thetas,tangent = np.loadtxt("find_error_contours.csv", delimiter=",", unpack=True)

fig = plt.figure()
ax = Axes3D(fig)

#x_data, y_data = np.meshgrid(y,thetas)
x_data = y
y_data = thetas

#surf = ax.plot_surface(x_data, y_data, tangent)
ax.scatter3D(x_data, y_data, tangent)
ax.set_xlabel("Lateral Offset (m)")
ax.set_ylabel("Initial Heading (rads)")
ax.set_zlabel("Tangential Dist to less than acceptable error (m)")

plt.show()
