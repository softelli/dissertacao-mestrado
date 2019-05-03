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
robXY = ( 0, 0)
beaXY = (-2, 1)



 
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
minX, maxX, minY, maxY = -3, 3, -3, 3

x=linspace(minX, maxX, 32)
y=linspace(minY, maxY, 32)
x,y = meshgrid(x, y)

# calculate vector field
#vx = -y/sqrt(x**2+y**2)*exp(-(x**2+y**2))
#vy =  x/sqrt(x**2+y**2)*exp(-(x**2+y**2))


#atrator do beacon
def atrator(x, y, x_beacon, y_beacon, desiredRadius, sensorRadius):
    va_x = -x + x_beacon
    va_y = -y + y_beacon
    
    E = sqrt(va_x**2 + va_y**2)
    k = find(E.flat[:] > sensorRadius)
    m = find(E.flat[:] < desiredRadius)
    
    va_x.flat[k]=0
    va_y.flat[k]=0
    va_x.flat[m]=0
    va_y.flat[m]=0
    
    return va_x, va_y
  
#vx_atrator = -x + beaXY[0]
#vy_atrator = -y + beaXY[1]
    
def repulsor(x, y, x_beacon, y_beacon, desiredRadius):
    vr_x = x - x_beacon
    vr_y = y - y_beacon
    
    E = sqrt(vr_x**2 + vr_y**2)

    m = find(E.flat[:] >= desiredRadius)
    
    vr_x.flat[m]=0
    vr_y.flat[m]=0
    
    return vr_x, vr_y

#vx_repulsor = x - beaXY[0]
#vy_repulsor = y - beaXY[1]

def rotacionador(x, y, x_beacon, y_beacon, desiredRadius, rotMargin):
    vr_x = -(y - y_beacon)
    vr_y =   x - x_beacon
    
    E = sqrt(vr_x**2 + vr_y**2)
    k = find(E.flat[:] < desiredRadius - rotMargin)
    m = find(E.flat[:] > desiredRadius + rotMargin)
    
    vr_x.flat[m]=0
    vr_y.flat[m]=0
    vr_x.flat[k]=0
    vr_y.flat[k]=0
    
    return vr_x, vr_y
    

#rotacao no beacon anti-horaria
#vx_rotacao = -(y - beaXY[1])
#vy_rotacao =   x - beaXY[0]

#repulsor de bordas
#vx = x * cos(arctan2(y, x) + pi/2) 
#vy = y * sin(arctan2(y, x) - pi/2)

#vx, vy = atrator(x, y, beaXY[0], beaXY[1], dR, sR)
#vx, vy = repulsor(x, y, beaXY[0], beaXY[1], dR)

rotx, roty = rotacionador(x, y, beaXY[0], beaXY[1], dR, 0.1)
atrx, atry = atrator(x, y, beaXY[0], beaXY[1], dR, sR)
repx, repy = repulsor(x, y, beaXY[0], beaXY[1], dR) 

vx = rotx + (atrx + repx) / 2
vy = roty + (atry + repy) / 2

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

show()
savefig('visualization_vector_fields_1.png')