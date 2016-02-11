from mpl_toolkits.mplot3d.axes3d import Axes3D
import matplotlib.pyplot as plt


# imports specific to the plots in this example
import numpy as np
from matplotlib import cm
from mpl_toolkits.mplot3d.axes3d import get_test_data
from matplotlib.mlab import griddata

# Twice as wide as it is tall.
fig = plt.figure(figsize=plt.figaspect(0.5))
ax = fig.gca(projection='3d')
#read file
data = np.genfromtxt('out.txt')
x = data[200:,0]
y = data[200:,1]
e1 = data[200:,2]
e2 = data[200:,3]

xi = np.linspace(min(x), max(x))
yi = np.linspace(min(y), max(y))

X, Y = np.meshgrid(xi, yi)



e1 = e1/max(e1)

Z = griddata(x, y, e1, xi, yi, interp='linear')


surf = ax.plot_surface(X, Y, Z, rstride=5, cstride=5, cmap=cm.jet,
                       linewidth=1, antialiased=True)

ax.set_zlim3d(np.min(Z), np.max(Z))
fig.colorbar(surf)

ax.set_zlim3d(-1.01, 1.01)



plt.show()