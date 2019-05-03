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


sA = pi * 3/2 # 270
dA = pi / 2


# generate grid
x=linspace(-3.5, 3.5, 64)
mpa = dA - (sA/2 - dA) #min proportional angle

k = find(x[:] <= mpa)
r = find(x[:] > sA/2)

y = (x - mpa)/(sA/2 - mpa)
y = 2 * y - 1
#y = 2 * (1 / (1 + exp(-(x - pi/2)))) - 1
y[k] = -1
y[r] =  1



#x, y=meshgrid(x, y)
 
plot(x,y)
xticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi],[r'$-\pi$', r'$-\pi/2$', r'$0$', r'$+\pi/2$', r'$+\pi$'])
xlabel('$detected$ $angle$')
ylabel('$coef$')
axis('image')
show()

