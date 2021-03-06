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


sR = 3 # 
dR = 1.5 #
mD = 0.45 #


# generate grid
x=linspace(0, sR, 64)


y = 2 * (x - dR)/sR

#y = 2 * y - 1

#y = 2 * (1 / (1 + exp(-(x - dR)))) - 1
#y = 2 * y - 1
#y[k] = -1
#y[r] =  1



#x, y=meshgrid(x, y)
 
plot(x,y)
xlabel('$detected$ $radius$')
ylabel('$coef$')
axis('image')
show()

