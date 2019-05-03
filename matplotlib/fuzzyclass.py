from fuzzy import *

class fuzzyObj:
  def rank(self, radius, input_name, element_names, beacon_radius_id, robot_radius_id, sensor_radius):

    radius_diff = robot_radius_id - beacon_radius_id
	
    regions = []
    regions[0] = [0.0, beacon_radius_id - radius_diff, beacon_radius_id]
    regions[1] = [beacon_radius_id - radius_diff, beacon_radius_id, robot_radius_id]
    regions[2] = [beacon_radius_id, robot_radius_id, robot_radius_id + radius_diff]
    regions[3] = [robot_radius_id, robot_radius_id + radius_diff, sensor_radius]
	
    fzi = fuzzySignal(inputName)

    for i in range(len(element_names)):
      fzi.addRegion(element_names[i],trimf(regions[i]))

    f = fuzzy('Dupa')

    f.addInSignal(fzi)
    f.addOutSignal(fzi)
	
    for i in range(len(element_names)):
      f.addRule(rule({inputName:element_names[i]}, element_names[i]))

    k = f.evaluate({input_name:radius})
	
    return max(k, key=k.get)
  
class fuzzyAv: 
  def rankAvoidation(self, sensorRadius, obtainedRadiusToBeacon, desiredRadiusToBeacon, obtainedDistanceToRobot, minDistanceBetweenRobots, obtainedDistanceFromBeaconToRobot):
    sR = sensorRadius
    oRtB = obtainedRadiusToBeacon
    dRtB = desiredRadiusToBeacon
    oDtR = obtainedDistanceToRobot
    oDfBtR = obtainedDistanceFromBeaconToRobot
    mDbR = minDistanceBetweenRobots
    medDbR = mDbR + (sR - mDbR) / 2.0 
    minRtB = min(dRtB,oRtB)/2.0 - mDbR
    maxRtB = max(dRtB,oRtB)/2.0 + mDbR
    
    #INPUT 1
    distance = 'distance'
    fzi_d = fuzzySignal(distance)
    
    fzi_d.addRegion('low',    trimf([   0.0,   mDbR, medDbR]))
    fzi_d.addRegion('middle', trimf([  mDbR, medDbR, sR    ]))
    fzi_d.addRegion('high',   trimf([medDbR,     sR, 2*sR  ]))
    
    #INPUT 2
    localization = 'localization'
    fzi_l = fuzzySignal(localization)
    
    fzi_l.addRegion('left',   trimf([   0.0, minRtB, dRtB  ]))
    fzi_l.addRegion('center', trimf([minRtB,   dRtB, maxRtB]))
    fzi_l.addRegion('right',  trimf([  dRtB, maxRtB, 2 * sR]))
    
    #OUTPUT 1
    
    fzo_d = fuzzySignal('avoidation')
    
    fzo_d.addRegion('left',   trimf([   0.0,   mDbR, medDbR])) #trimf([-1.0, -0.5, 0.0]))
    fzo_d.addRegion('center', trimf([  mDbR, medDbR, sR    ])) #trimf([-0.5,  0.0, 0.5]))
    fzo_d.addRegion('right',  trimf([medDbR,     sR, 2*sR  ])) #trimf([ 0.0,  0.5, 1.0]))
    
    fav = fuzzy('av')
    
    fav.addInSignal(fzi_d)
    fav.addInSignal(fzi_l)
    fav.addOutSignal(fzo_d)
    
    fav.addRule(rule({'distance':'low',   'localization':  'left'}, 'right'))
    fav.addRule(rule({'distance':'low',   'localization':'center'},'center'))
    fav.addRule(rule({'distance':'low',   'localization': 'right'},  'left'))
    
    fav.addRule(rule({'distance':'middle','localization':  'left'}, 'right'))
    fav.addRule(rule({'distance':'middle','localization':'center'},'center'))
    fav.addRule(rule({'distance':'middle','localization': 'right'},  'left'))
    
    fav.addRule(rule({'distance':'high',  'localization':  'left'},'center'))
    fav.addRule(rule({'distance':'high',  'localization':'center'},'center'))
    fav.addRule(rule({'distance':'high',  'localization': 'right'},'center')) 
       
    #EVALUATION
    av = fav.evaluate({distance: oDtR, localization: oDfBtR})

    
    return max(av, key=av.get)
  
class fuzzyAc:    
  def rankAccelleration(self, sensorRadius, obtainedRadiusToBeacon, desiredRadiusToBeacon, obtainedDistanceToRobot, minDistanceBetweenRobots, obtainedDistanceFromBeaconToRobot):
    sR = sensorRadius
    oRtB = obtainedRadiusToBeacon
    dRtB = desiredRadiusToBeacon
    oDtR = obtainedDistanceToRobot
    oDfBtR = obtainedDistanceFromBeaconToRobot
    mDbR = minDistanceBetweenRobots
    medDbR = mDbR + (sR - mDbR) / 2.0 
    minRtB = min(dRtB,oRtB)/2.0 - mDbR
    maxRtB = max(dRtB,oRtB)/2.0 + mDbR
    
    #INPUT 1
    distance = 'distance'
    fzi_d = fuzzySignal(distance)
    
    fzi_d.addRegion('low',    trimf([   0.0,   mDbR, medDbR]))
    fzi_d.addRegion('middle', trimf([  mDbR, medDbR, sR    ]))
    fzi_d.addRegion('high',   trimf([medDbR,     sR, 2*sR  ]))
    
    #INPUT 2
    localization = 'localization'
    fzi_l = fuzzySignal(localization)
    
    fzi_l.addRegion('left',   trimf([   0.0, minRtB, dRtB  ]))
    fzi_l.addRegion('center', trimf([minRtB,   dRtB, maxRtB]))
    fzi_l.addRegion('right',  trimf([  dRtB, maxRtB, 2 * sR]))
    
      
    #OUTPUT 2
    
    fzo_a = fuzzySignal('accelleration')
    
    fzo_a.addRegion('negative',  trimf([   0.0, minRtB, dRtB  ])) #trimf([-1.0, -0.5, 0.0]))
    fzo_a.addRegion('neutral',   trimf([minRtB,   dRtB, maxRtB])) #trimf([-0.5,  0.0, 0.5]))
    fzo_a.addRegion('positive',  trimf([  dRtB, maxRtB, 2 * sR])) #trimf([ 0.0,  0.5, 1.0]))
    
    fac = fuzzy('ac')
    
    fac.addInSignal(fzi_d)
    fac.addInSignal(fzi_l)
    fac.addOutSignal(fzo_a)
    
    fac.addRule(rule({'distance':'low',   'localization':  'left'}, 'negative'))
    fac.addRule(rule({'distance':'low',   'localization':'center'}, 'negative'))
    fac.addRule(rule({'distance':'low',   'localization': 'right'}, 'negative'))
    
    fac.addRule(rule({'distance':'middle','localization':  'left'}, 'negative'))
    fac.addRule(rule({'distance':'middle','localization':'center'},  'neutral'))
    fac.addRule(rule({'distance':'middle','localization': 'right'}, 'negative'))
    
    fac.addRule(rule({'distance':'high',  'localization':  'left'},  'neutral'))
    fac.addRule(rule({'distance':'high',  'localization':'center'},'positive'))
    fac.addRule(rule({'distance':'high',  'localization': 'right'},  'neutral'))
    
       
    #EVALUATION
    ac = fac.evaluate({distance: oDtR, localization: oDfBtR})
    
    return max(ac, key=ac.get)