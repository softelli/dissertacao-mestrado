#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import useful modules
import matplotlib as plt
from numpy import *
from pylab import *
 
# use LaTeX, choose nice some looking fonts and tweak some settings
matplotlib.rc('font', family='sanserif')
matplotlib.rc('font', size=16)
matplotlib.rc('legend', fontsize=16)
matplotlib.rc('legend', numpoints=1)
matplotlib.rc('legend', handlelength=1.5)
matplotlib.rc('legend', frameon=False)
matplotlib.rc('xtick.major', pad=7)
matplotlib.rc('xtick.minor', pad=7)
matplotlib.rc('text', usetex=True)
#matplotlib.rc('text.latex', 
#              preamble=[r'\usepackage[T1]{fontenc}',
#                        r'\usepackage{amsmath}',
#                        r'\usepackage{txfonts}',
#                        r'\usepackage{textcomp}'])
 
close('all')
figure(figsize=(10, 10))

#angulo desejado
dA = pi/2.0

#angulo do sensor
sA = pi * 3/2 # 270



# generate grid
x=linspace(-3.5, 3.5, 64)

#numerador
num = dA - x
#denominador
den = sA / 2.0

y = num/den
 
plt.plot(x,y)
plt.plot(np.pi/2, 0.0, "bs")
#plt.plot(-np.pi, 0.0, np.pi/2, 0.0)
plt.plot([-3.5,3.5],[0,0], color="g")
plt.plot([np.pi/2.0,np.pi/2.0],[2.1,-1.1], color="g")
plt.plot([np.pi,np.pi],[-1.1,2.1], color="black", linestyle="dotted")
plt.plot([-np.pi,-np.pi],[-1.1,2.1], color="black", linestyle="dotted")
plt.xticks([-np.pi, -np.pi/2, 0, np.pi/2,np.pi],[r'$-\frac{\alpha_S}{2}$', r'$-90^{\circ}$', r'$0^{\circ}$', r'$90^{\circ}$',r'$\frac{\alpha_S}{2}$'])
plt.xlabel(r'$\alpha_T$')
plt.ylabel(r'$\beta$')
plt.axis('image')
plt.show()

