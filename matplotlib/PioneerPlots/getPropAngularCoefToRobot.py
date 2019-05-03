#!/usr/bin/env python
 
# import useful modules
import matplotlib 
from numpy import *
from pylab import *
 
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
figure(figsize=(10, 10))

#angle_to_robot = (self.obtainedAngleToRobot**2)**0.5  #apenas valores positivos
#print "pIc =", 1 - angle_to_robot / (sp.sensor_cone_angle / 2.0)
#return 1 - angle_to_robot / (sp.sensor_cone_angle / 2.0)

sA = pi * 3/2 # 270

# generate grid
x=linspace(-sA/2, sA/2, 64)
atr = (x**2)**0.5
y = (1 - atr / (sA/2.0))**2 

plot(x,y)
xticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi],[r'$-\pi$', r'$-\pi/2$', r'$0$', r'$+\pi/2$', r'$+\pi$'])
xlabel('$detected$ $angle$')
ylabel('$coef$')
axis('image')
show()

