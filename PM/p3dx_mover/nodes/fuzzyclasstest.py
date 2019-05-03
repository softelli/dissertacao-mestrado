from fuzzyclass import *

fObj = fuzzyObject()
#def rank(self, radius, input_name, element_names, beacon_radius_id, robot_radius_id, sensor_radius):

from ParametersServer import staticParameters
sp = staticParameters

obj = 0.0

while obj < sp.sensor_cone_radius:
  print obj, fObj.rank(obj, 'radius', ['Void', 'Beacon', 'Robot', 'Alien'],  sp.beacon_radius_id, sp.robot_radius_id, sp.sensor_cone_radius)
  obj += 0.01