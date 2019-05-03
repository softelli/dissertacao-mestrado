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


sA = pi * 3/2 # 270
dA = pi / 2


# generate grid
x=linspace(-3.5, 3.5, 64)
mpa = dA - (sA/2 - dA) #min proportional angle

k = find(x[:] <= mpa)
r = find(x[:] > sA/2)

y = (x - mpa)/(sA/2 - mpa)

y = 2 * y - 1 #para retorno [-1.0,1,0]
y[k] = -1
y[r] =  1

#y = 2 * y #para retorno [0.0,2.0]
#y[k] = 0.0
#y[r] = 2.0

#y = 2 * (1 / (1 + exp(-(x - pi/2)))) - 1



#x, y=meshgrid(x, y)
 
plt.plot(x,y)
plt.plot(mpa, -1.0, "ro")
plt.plot(np.pi/2, 0.0, "bs")
plt.plot(sA/2, 1.0, "ro")
plt.plot([-3.5,3.5],[0,0], color="g")
plt.plot([np.pi/2.0,np.pi/2.0],[-1.1,1.1], color="g")
plt.plot([np.pi*3/4,np.pi*3/4],[-1.1,1.1], color="black", linestyle="dotted")
plt.plot([np.pi/4,np.pi/4],[-1.1,1.1], color="black", linestyle="dotted")
plt.xticks([-np.pi, -np.pi/2, 0, np.pi/4, np.pi/2,np.pi*3/4,np.pi],[r'$-\frac{\alpha_S}{2}$', r'$-90^{\circ}$', r'$0^{\circ}$', r'$\alpha_{\min}$', r'$90^{\circ}$',r'$\alpha_{\max}$', r'$\frac{\alpha_S}{2}$'])
plt.xlabel(r'$\alpha_T$')
plt.ylabel(r'$\delta$')
plt.axis('image')
plt.show()

