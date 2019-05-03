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


sensorRadius = 3 # 
desiredRadius = 1.5 #
desiredDistance = 1.3 #
minDbR = 0.65 #


# generate grid
x=linspace(0, sensorRadius, 64)

#distancia desejada, minima e obtida ao robo detectado

#calculo da curva
y = 1 - (x - minDbR)/(desiredDistance - minDbR)

#poda dos valores fora do intervalo de controle
s = find(y.flat[:] < 0.0)
y.flat[s] = 0.0
s = find(y.flat[:] > 1.0)
y.flat[s] = 1.0


#plotagem
plot(x,y)
plot(minDbR, 1.0, 'bs')
plot(desiredDistance, 0.0, 'bs')
xlabel('$detected$ $radius$')
ylabel('$coef$')
axis('image')


#distancia do beacon ao robot - raioDesejado

#apenas valores positivos (modulo)
m = x - desiredRadius
k=find(m.flat[:]<0)
m.flat[k] = m.flat[k] * -1.0

#calculo da curva
z = 1 - m / minDbR

#poda de valores fora do intervalo de controle.
s = find(z.flat[:] < 0.0)
z.flat[s] = 0.0
s = find(z.flat[:] > 1.0)
z.flat[s] = 1.0

t = ((y + z)**2)/2

plot(x,z)
plot(x,t)

plot(minDbR, 0.0, 'ro')
plot(desiredRadius, 1.0, 'ro')
plot(2 * desiredRadius - minDbR , 0.0, 'ro')
xlabel('$detected$ $radius$')
ylabel('$coef$')
axis('image')
show()

#??????
#fig = figure()
#ax = Axes3D(fig)
#ax.plot_wireframe(x,y,z)
#show()

