from fuzzy import *

class fuzzyObject:
  def rank(self, radius, input_name, element_names, beacon_radius_id, robot_radius_id, sensor_radius):

    radius_diff = robot_radius_id - beacon_radius_id
	
    regions = []
    regions.append([0.0, beacon_radius_id - radius_diff, beacon_radius_id])
    regions.append([beacon_radius_id - radius_diff, beacon_radius_id, robot_radius_id])
    regions.append([beacon_radius_id, robot_radius_id, robot_radius_id + radius_diff])
    regions.append([robot_radius_id, robot_radius_id + radius_diff, sensor_radius])
	
    fzi = fuzzySignal(input_name)

    for i in range(len(element_names)):
      fzi.addRegion(element_names[i],trimf(regions[i]))

    f = fuzzy('Dupa')

    f.addInSignal(fzi)
    f.addOutSignal(fzi)
	
    for i in range(len(element_names)):
      f.addRule(rule({input_name:element_names[i]}, element_names[i]))

    k = f.evaluate({input_name:radius})
	
    return max(k, key=k.get)
 