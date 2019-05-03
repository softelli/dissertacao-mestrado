from fuzzy import *


beacon_radius_id = 0.10
robot_radius_id = 0.15
sensor_radius = 3.0
radius_diff = robot_radius_id - beacon_radius_id

void   = [0.0, beacon_radius_id - radius_diff, beacon_radius_id]
beacon = [beacon_radius_id - radius_diff, beacon_radius_id, robot_radius_id]
robot  = [beacon_radius_id, robot_radius_id, robot_radius_id + radius_diff]
alien  = [robot_radius_id, robot_radius_id + radius_diff, sensor_radius]

aa = fuzzySignal('radius')

aa.addRegion('Void',trimf(void))
aa.addRegion('Beacon',trimf(beacon))
aa.addRegion('Robot',trimf(robot))
aa.addRegion('Alien', trimf([robot_radius_id,sensor_radius,2*sensor_radius]))


ff = fuzzy('Dupa')

ff.addInSignal(aa)
ff.addOutSignal(aa)

ff.addRule(rule({'radius':'Void'},'Void'))
ff.addRule(rule({'radius':'Beacon'},'Beacon'))
ff.addRule(rule({'radius':'Robot'},'Robot'))
ff.addRule(rule({'radius':'Alien'},'Alien'))

r = 0.0
print "Void", void
print "Beacon", beacon
print "Robot", robot
print "Alien", alien
while r < sensor_radius/10:
   
   k = ff.evaluate({'radius':r})
   #print k
   print "r:", r, max(k, key=k.get)
   r += 0.015
