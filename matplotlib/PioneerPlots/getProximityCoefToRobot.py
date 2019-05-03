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
dD = 0.65

maxC = dD / (2 * dD)
minC = - dD / (2 * dD)

# generate grid
x=linspace(0, 2 * dD, 64)

y = (x - dD) / (2 * (dD - mD))
m = find(y.flat[:] > maxC)
y.flat[m] = maxC
n = find(y.flat[:] < minC)
y.flat[n] = minC
#Velocidade linear soh eh controlada para a distancia no intervalo [mD, 2 * (dD - mD)]


 
plot(x,y)
plot(dD,0,'ro')
plot(mD,-0.5,'bs')
plot(2 * dD - mD, 0.5, 'bs')
xticks([mD, dD, 2*dD - mD],[r'$mD$', r'$dD$', r'$2dD - mD$'])
xlabel('$detected$ $distance$')
ylabel('$coef$')
axis('image')
show()

