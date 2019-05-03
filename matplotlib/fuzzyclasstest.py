from fuzzyclass import *

fav = fuzzyAv()
fac = fuzzyAc()

sR = 3.0
oRtB = 1.3
dRtB = 1.5
oDtR = 0.75
minDbR = 0.3

for od in range(0, 30):
  oDfBtR = od / 10.0
  print "oDfBtr", oDfBtR, "Av:", fav.rankAvoidation(sR, oRtB, dRtB, oDtR, minDbR, oDfBtR), "Ac:", fac.rankAccelleration(sR, oRtB, dRtB, oDtR, minDbR, oDfBtR) 