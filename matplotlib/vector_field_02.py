#!/usr/bin/env python
 
# import useful modules
import matplotlib as plt
from numpy import *
from pylab import *

sR = 2.0 #sensor Radius
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
x=linspace(-3, 3, 32)
y=linspace(-3, 3, 32)
x,y = meshgrid(x, y)

# calculate vector field
#vx = -y/sqrt(x**2+y**2)*exp(-(x**2+y**2))
#vy =  x/sqrt(x**2+y**2)*exp(-(x**2+y**2))

#repulsor do beacon
#vx = x - beaXY[0]
#vy = y - beaXY[1]

#atrator do beacon
#vx = -x + beaXY[0]
#vy = -y + beaXY[1]

#rotacao no beacon anti-horaria
vx = -(y - beaXY[1])
vy =   x - beaXY[0]

# plot vecor field
quiver(x, y, vx, vy, pivot='middle', headwidth=4, headlength=6)
xlabel('$x$')
ylabel('$y$')
axis('image')

fig = plt.gcf()

beacon1 = plt.Circle(beaXY,bR, color='g', fill=True)
fig.gca().add_artist(beacon1)

show()
savefig('visualization_vector_fields_1.png')