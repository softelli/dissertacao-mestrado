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

sA = 270 # 270
dA = 90
mD = 0.65
sR = 3.0
dR = 1.7
oR = sR - dR
vlin = 0.1
maxVang = 1.5


# generate grid radio based
x=linspace(0, sR, 64)
iA = 0.2
oA = -sA/2
rant = 0.0

while(oA < sA/2): 

  #coordenadas atuais
  xt = x * cos(oA * pi/180)
  yt = x * sin(oA * pi/180)
  vang = x*100000
  r = (xt**2 + yt**2 - dR**2) / (2 * (dR + yt))
  k = find(r.flat[:] > 4 * sR)
  r.flat[k] = 0.0
  l = find(r.flat[:] < -4 * sR)
  r.flat[l] = 0.0
  m = find(r.flat[:] == -dR)
  r.flat[m] = 0.0
  n = find(r.flat[:] <> 0.0)
  vang.flat[n] = vlin / r 
  o = find(vang.flat[:] > maxVang)
  vang.flat[o] = maxVang
  p = find(vang.flat[:] < -maxVang)
  vang.flat[p] = -maxVang 
  print "oA:", oA, "r:", r
  plot(x,vang, label=oA)
  oA = oA + 15
  

#xticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi],[r'$-\pi$', r'$-\pi/2$', r'$0$', r'$+\pi/2$', r'$+\pi$'])
legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,ncol=2, mode="expand", borderaxespad=0.)
#xlabel('$detected$ $angle$')
#ylabel('$coef$')
axis('image')
show()
