from ParametersServer import staticParameters
from Circumnavigation import circum
from polar_coord import LinAng

sp = staticParameters

c = circum()

#teste das funcoes

if c.rDiR(1.0,0.0) == 1.0:
   print "c.rDir p/ oRtB = 0.0:", c.rDiR(1.0, 0.0), "OK"
else:
   print "c.rDir p/ oRtB = 0.0:", c.rDiR(1.0, 0.0), "Error"
   
if c.rDiR(1.0,1.0) == 0.0:
   print "c.rDir p/ oRtB = 1.0:", c.rDiR(1.0, 1.0), "OK"
else:
   print "c.rDir p/ oRtB = 1.0:", c.rDiR(1.0, 1.0), "Error"
   
if c.rDiR(1.0,1.5) > 0.0:
   print "c.rDir p/ oRtB = 1.0:", c.rDiR(1.0, 1.5), "OK"
else:
   print "c.rDir p/ oRtB = 1.0:", c.rDiR(1.0, 1.5), "Error"