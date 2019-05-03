from numpy import *
from Circumnavigation import circum 
from ParametersServer import staticParameters 

sp = staticParameters
c = circum()

sp.desired_radius = 1.0

#setar raio e angulo (graus)
#linear = 1.95397537229
#angular = -52.9857734443
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()


#setar raio e angulo (graus)
#linear = 1.95397537229
#angular = 52.9857734443
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 0.50990
#angular = 90
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 1.441
#angular = 90
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 0.36913
#angular = -90
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 1.0
#angular = -90
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 1.1725
#angular = -121
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 1.1097
#angular = -64
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 0.645234597551
#angular = -101.217639406
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 1.0
#angular = 90
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 1.0
#angular = 89.9
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 1.0
#angular = 90.1
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 1.01
#angular = 90
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#print "decrementando..."
#linear = 1.1
#angular = 90
#while(True):
#      c.beacon.setPolarAndCalcRetCoords([linear,angular])
#      linear = c.getApproachRadius()
#      if(linear == 1.0):
#	break
      
#print "incrementando..."
#linear = 0.9
#angular = 90
#while(True):
#      c.beacon.setPolarAndCalcRetCoords([linear,angular])
#      linear = c.getApproachRadius()
#      if(linear == 1.0):
#	break
      
#linear = 0.9312
#angular = 72.41
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

#linear = 0.92998
#angular = 97
#c.beacon.setPolarAndCalcRetCoords([linear,angular])
#radius = c.getApproachRadius()

a = [-135,-90.0, -45.0, 0.0, 45.0, 89.9, 90.0, 90.1, 135.0]
l = [ 0.5,  1.0,  1.5]
msg = ""

lx = [ 1.50,  1.00,  0.50, 0.00,-0.25]
ly = [-1.25, -1.00, -0.75, 0.00, 0.75, 1.00, 1.25]
xy = []
for x in lx:
    for y in ly:
	xy.append([x,y])
	
#print xy
c.linearVelocity = sp.linear_velocity
c.angularVelocity = sp.angular_velocity
cont = 0
erros = 0
for p in xy:
    cont += 1
    #print "---", p , "---"
    #c.beacon.setPolarAndCalcRetCoords([linear,angular])
    c.beacon.setRetAndCalcPolarCoords(p)
    KpRadial = c.getPropRadialCoefToBeacon()
    KpAngular = c.getPropAngularCoefToBeacon()
    c.angularVelocity = (c.linearVelocity / sp.desired_radius) * (1 + (KpRadial + KpAngular)/2)/2
    radius = c.linearVelocity / c.angularVelocity
    
    msg = p,c.beacon.angular, c.beacon.linear, radius, "v", c.linearVelocity, "w", c.angularVelocity
    #msg = ""
    if c.beacon.angular <= 0.0:
      if radius < 0.0:
	print msg #, "Ok	(1) a <= 0.0, radius < 0.0"
      else:
	print msg, "\n \t Erro(1): radius >= 0.0 (positive)"
	erros += 1
    elif c.beacon.angular > 90.0:
      if radius > 0.0:
	print msg #, "Ok	(2) a > 90, radius > 0.0"
      else:
	print msg, "\n \t Erro(2): radius <= 0.0 (negative)"
	erros += 1
    elif c.beacon.angular == 90.0:
      if c.beacon.y < sp.desired_radius and radius > sp.desired_radius:
	print msg #, "Ok	(3a) b.y < rd, radius > rd"
      elif c.beacon.y == sp.desired_radius and radius == sp.desired_radius:
	print msg #, "Ok	(3b) b.y = rd, radius = rd"
      elif c.beacon.y > sp.desired_radius and radius > 0.0: #and radius < sp.desired_radius:
	print msg #, "Ok	(3c) b.y > rd, 0 < radius < rd"
      else:
	print msg, "\n Erro(3):"
	print "\t b.y < rd?[", c.beacon.y < sp.desired_radius, "] radius > rd?[", radius > sp.desired_radius,"]"
	print "\t b.y = rd?[", c.beacon.y == sp.desired_radius,"] radius = 0.0[", radius == sp.desired_radius,"]"
	print "\t b.y > rd?[", c.beacon.y > sp.desired_radius,",] radius > 0.0[", radius > 0.0, "] radius < rd?[", radius < sp.desired_radius,"]"
	erros += 1
    else: #0 < angular < 90
      if c.beacon.y < sp.desired_radius and radius < 0.0:
	print msg #, "Ok	(4a) b.y < rd, radius < 0"
      elif c.beacon.y == sp.desired_radius and radius == 0.0:
	print msg #, "Ok	(4b) b.y = rd, radius = 0.0"
      elif c.beacon.y > sp.desired_radius and radius > sp.desired_radius:
	print msg #, "Ok	(4c) b.y > rd, radius > 0.0"
      else:
	print msg, "\n Erro(4):"
	print "\t c.beacon.y < sp.desired_radius[",c.beacon.y < sp.desired_radius, "] radius < 0.0[", radius < 0.0,"]"
	print "\t c.beacon.y == sp.desired_radius[", c.beacon.y == sp.desired_radius, "] radius == 0.0[" , radius == 0.0,"]"
	print "\t c.beacon.y > sp.desired_radius[", c.beacon.y > sp.desired_radius, "] radius > 0.0[",radius > 0.0,"]"
	erros += 1
print "erros: ", erros, "/", cont