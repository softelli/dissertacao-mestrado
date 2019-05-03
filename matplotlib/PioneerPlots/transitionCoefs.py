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

def boltzman(x, xmid, tau):
    """
    evaluate the boltzman function with midpoint xmid and time constant tau
    over x
    """
    return 1. / (1. + exp(-(x-xmid)/tau))

sR = 3 # 
dR = 1.0 #
mD = 0.45 #


# generate grid
x = linspace(0, sR, 64)

#high = find(x.flat[:] >= dR)
#low =  find(x.flat[:] <  dR)
#y.flat[high] = (x - dR) / (sR - dR)
#y.flat[low]  = 0.0

less = find(x.flat[:] < dR)
y = (x - dR) / (sR - dR)
y.flat[less] = 0.0

z = boltzman(x, dR, 0.05)
y = 1 - z

#z = 1 / (1 + exp(-y))

#y = 2 * (x - dR)/sR #para valores entre [-1.0, 1.0]

#y = 2 * (x - dR)/sR + 1 #para valores entre [0.0, 2.0]
#y = 2 * y - 1





#x, y=meshgrid(x, y)
 
plot(x,y)
#plot(x, 1 - y)
plot(x,z)
plot(dR, 0.5, 'ro')
xlabel('$detected$ $radius$')
ylabel('$kpA$')
axis('image')
show()

