import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

fig_1 = plt.figure(1, figsize=[5, 5])
pts = np.array([[0, 2*(np.sqrt(1**2 - 0.5**2))/3], [-0.5, -(np.sqrt(1**2 - 0.5**2)) / 3], [0.5, -(np.sqrt(1**2 - 0.5**2))/3]])
p = Polygon(pts, closed=False)
p.set_color('r')
grid_1 = fig_1.add_subplot()
grid_1.add_patch(p)
plt.xlabel("x")
plt.ylabel("y")
plt.axis([-5, 5, -5, 5])
plt.grid()
plt.title("Pioneer P3DX - Simulator")
plt.show()

"""
pts = np.array([[0, 0.866], [-0.5, -0.289], [0.5, -0.289]])
p = Polygon(pts, closed=False)
ax = plt.gca()
ax.add_patch(p)
ax.set_xlim(-7, 7)
ax.set_ylim(-8, 8)
plt.show()
"""