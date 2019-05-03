#!/usr/bin/env python
 
# import useful modules
import matplotlib 
from numpy import *
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
 
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


Rs = 3 # 
Rd = 1.5 #
Dmin = 0.65 #


# generate grid
x=linspace(0, 2 * Rs, 64)

#calculo da curva
absVal = ((x - Rd)**2)**0.5
y = 1 - (absVal/Dmin)

#poda dos valores fora do intervalo de controle
s = find(x.flat[:] < (Rd - Dmin))
y.flat[s] = 0.0
s = find(x.flat[:] > (Rd + Dmin))
y.flat[s] = 0.0

#plotagem
plot(x,y)
xlabel('$detected$ $radius$')
ylabel('$coef$')
axis('image')

show()



