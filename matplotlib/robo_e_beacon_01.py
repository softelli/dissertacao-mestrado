import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, FancyArrowPatch
from matplotlib.lines import Line2D


sR = 2.0 #sensor Radius
sA = 270 #sensor Angle
dR = 1.5 #desired Radius
bR = 0.1 #beacon Radius
rR = 0.15 #robot Radius
mD = 0.3 #min dist between robots
robXY = ( 0, 0)
beaXY = (-2, 1)

dominio = [-3, 3, -3, 3]
plt.axis(dominio)

fig = plt.gcf()

fig.set_size_inches(10, 10, forward=True)



#beacon
beacon1 = plt.Circle(beaXY,bR, color='g', fill=True)
fig.gca().add_artist(beacon1)

bealin1 = FancyArrowPatch((beaXY[0] - 2 * bR, beaXY[1]),(beaXY[0] + 2*bR, beaXY[1]), arrowstyle = '-', mutation_scale=1)
fig.gca().add_artist(bealin1)
bealin2 = FancyArrowPatch((beaXY[0], beaXY[1] - 2 * bR),(beaXY[0], beaXY[1] + 2 * bR), arrowstyle = '-', mutation_scale=1)
fig.gca().add_artist(bealin2)

desiredRadius = plt.Circle(beaXY, dR, color='b', fill=False, linestyle='dashdot')
fig.gca().add_artist(desiredRadius)


# Arc( xy, width, height, angle, theta1, theta2, **kwargs)
laserField1 = Wedge(robXY,  sR, 90 - sA/2,  90 + sA/2, width=sR-mD, color='gray', linestyle='dotted', alpha=0.25)
fig.gca().add_patch(laserField1)

robot = plt.Circle(robXY, rR, color='r', fill=True)
fig.gca().add_patch(robot)

minimalDist = plt.Circle(robXY, mD, color='r', fill=False)
fig.gca().add_patch(minimalDist)

axesX = FancyArrowPatch((robXY[0] - sR * 0.5, robXY[1]),(robXY[0] + sR * 0.5, robXY[1]), arrowstyle = '-|>', mutation_scale=20)
fig.gca().add_artist(axesX)

axesY = FancyArrowPatch((robXY[0], robXY[1] - sR * 0.5),(robXY[0], robXY[1] + sR * 0.5), arrowstyle = '-|>', mutation_scale=20)
fig.gca().add_artist(axesY)

plt.show()