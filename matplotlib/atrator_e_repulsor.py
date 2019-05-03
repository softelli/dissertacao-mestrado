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
figure(figsize=(6, 4.5))
 
# generate grid
x=linspace(-2, 2, 32)
y=linspace(-1.5, 1.5, 24)
x, y=meshgrid(x, y)
 
def E(q, a, x, y):
    return q*(x-a[0])/((x-a[0])**2+(y-a[1])**2)**(1.5), \
        q*(y-a[1])/((x-a[0])**2+(y-a[1])**2)**(1.5)
 
# calculate vector field
Ex1, Ey1=E(1, [-1, 0], x, y)
Ex2, Ey2=E(-1, [1, 0], x, y)
Ex=Ex1+Ex2
Ey=Ey1+Ey2
# remove vector with length larger than E_max
E_max=2
E=sqrt(Ex**2+Ey**2)
k=find(E.flat[:]>E_max)
Ex.flat[k]=nan
Ey.flat[k]=nan
# plot vecor field
quiver(x, y, Ex, Ey, pivot='middle', headwidth=4, headlength=6)
xlabel('$x$')
ylabel('$y$')
axis('image')
show()