#!/usr/bin/env python
 
# import useful modules
import matplotlib as plt
from matplotlib.patches import Wedge, FancyArrowPatch
from numpy import *
from pylab import *

sR = 2.5 #sensor Radius
sA = 270 #sensor Angle
dR = 1.5 #desired Radius
bR = 0.1 #beacon Radius
rR = 0.15 #robot Radius
mD = 0.3 #min dist between robots
dD = 0.45 # desired Distance between robots
robXY = (  0, 2) #posicao do robot mais proximo em relacao ao frame do robot
beaXY = ( -2, 1) #posicao do beacon em relacao ao frame do robot



 
# use LaTeX, choose nice some looking fonts and tweak some settings
matplotlib.rc('font', family='serif')
matplotlib.rc('font', size=16)
matplotlib.rc('legend', fontsize=16)
matplotlib.rc('legend', numpoints=1)
matplotlib.rc('legend', handlelength=1.5)
matplotlib.rc('legend', frameon=False)
matplotlib.rc('xtick.major', pad=7)
matplotlib.rc('xtick.minor', pad=7)
#matplotlib.rc('text', usetex=True)
#matplotlib.rc('text.latex', 
#              preamble=[r'\usepackage[T1]{fontenc}',
#                        r'\usepackage{amsmath}',
#                        r'\usepackage{txfonts}',
#                        r'\usepackage{textcomp}'])
 
close('all')
figure(figsize=(6, 6))
 
# generate grid
minX, maxX, minY, maxY = -5, 3, -5, 3
numpts = 32
x=linspace(minX, maxX, numpts)
y=linspace(minY, maxY, numpts)
x,y = meshgrid(x, y)

# calculate vector field
#vx = -y/sqrt(x**2+y**2)*exp(-(x**2+y**2))
#vy =  x/sqrt(x**2+y**2)*exp(-(x**2+y**2))


#atrator do beacon
def atrator(x, y, x_alvo, y_alvo, distAlvo, sensorRadius):
    va_x = -x + x_alvo
    va_y = -y + y_alvo
    
    E = sqrt(va_x**2 + va_y**2)
    k = find(E.flat[:] > sensorRadius)
    m = find(E.flat[:] < distAlvo)
    
    va_x.flat[k]=0
    va_y.flat[k]=0
    va_x.flat[m]=0
    va_y.flat[m]=0
    
    return va_x, va_y
  
#vx_atrator = -x + beaXY[0]
#vy_atrator = -y + beaXY[1]
    
def repulsor(x, y, x_alvo, y_alvo, distAlvo):
    vr_x = x - x_alvo
    vr_y = y - y_alvo
    
    E = sqrt(vr_x**2 + vr_y**2)

    m = find(E.flat[:] >= distAlvo)
    
    vr_x.flat[m]=0
    vr_y.flat[m]=0
    
    return vr_x, vr_y

#vx_repulsor = x - beaXY[0]
#vy_repulsor = y - beaXY[1]

def rotacionador(x, y, x_beacon, y_beacon, desiredRadius, rotMargin):
    vr_x = -(y - y_beacon)
    vr_y =   x - x_beacon
    
    #E = sqrt(vr_x**2 + vr_y**2)
    #k = find(E.flat[:] < desiredRadius - rotMargin)
    #m = find(E.flat[:] > desiredRadius + rotMargin)
    
    #vr_x.flat[m]=0
    #vr_y.flat[m]=0
    #vr_x.flat[k]=0
    #vr_y.flat[k]=0
    
    return vr_x, vr_y
    

#rotacao no beacon anti-horaria
#vx_rotacao = -(y - beaXY[1])
#vy_rotacao =   x - beaXY[0]

#repulsor de bordas
#vx = x * cos(arctan2(y, x) + pi/2) 
#vy = y * sin(arctan2(y, x) - pi/2)

#vx, vy = atrator(x, y, beaXY[0], beaXY[1], dR, sR)
#vx, vy = repulsor(x, y, beaXY[0], beaXY[1], dR)

bea_rotx, bea_roty = rotacionador(x, y, beaXY[0], beaXY[1], dR, 0.2)
bea_atrx, bea_atry = atrator(x, y, beaXY[0], beaXY[1], dR, sR)
bea_repx, bea_repy = repulsor(x, y, beaXY[0], beaXY[1], dR)

rob_atrx, rob_atry = atrator(x, y, robXY[0], robXY[1], dD, sR)
rob_repx, rob_repy = repulsor(x, y, robXY[0], robXY[1], dD)

vx = bea_rotx + bea_atrx + bea_repx + rob_atrx #+ rob_repx
vy = bea_roty + bea_atry + bea_repy + rob_atry #+ rob_repy

Emax = 3
E = sqrt(vx**2 + vy**2)
k = find(E.flat[:]>Emax)
vx.flat[k] = nan
vy.flat[k] = nan

# plot vecor field
quiver(x, y, vx, vy, pivot='middle', headwidth=4, headlength=6)
xlabel('$x$')
ylabel('$y$')
axis('image')

fig = plt.gcf()

beacon1 = plt.Circle(beaXY,bR, color='g', fill=True)
fig.gca().add_artist(beacon1)

bealin1 = FancyArrowPatch((beaXY[0] - 2 * bR, beaXY[1]),(beaXY[0] + 2*bR, beaXY[1]), arrowstyle = '-', mutation_scale=1)
fig.gca().add_artist(bealin1)
bealin2 = FancyArrowPatch((beaXY[0], beaXY[1] - 2 * bR),(beaXY[0], beaXY[1] + 2 * bR), arrowstyle = '-', mutation_scale=1)
fig.gca().add_artist(bealin2)

desiredRadius = plt.Circle(beaXY, dR, color='b', fill=False, linestyle='dashdot')
fig.gca().add_artist(desiredRadius)

sensorRadius = plt.Circle(beaXY, sR, color='g', fill=False, linestyle='dashdot')
fig.gca().add_artist(sensorRadius)

desiredDistToRobot = plt.Circle(robXY, dD, color='b', fill=False, linestyle='dashdot')
fig.gca().add_artist(desiredDistToRobot)

sensorRadiusRobot = plt.Circle(robXY, sR, color='g', fill=False, linestyle='dashdot')
fig.gca().add_artist(sensorRadiusRobot)

show()
savefig('visualization_vector_fields_1.png')