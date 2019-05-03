from numpy import *

class LinAng:
  def __init__(self):
    self.linear = 0.0
    self.angular = 0.0
    self.x = 0.0
    self.y = 0.0
  
  def clear(self):
    self.__init__()
    
  def setVelocities(self, linear_vel, angular_vel):
    self.linear = linear_vel
    self.angular = angular_vel
    
  #apenas por semantica
  def setPolarCoords(self, polarCoords):
    self.linear = polarCoords[0]
    self.angular = polarCoords[1]
    
  def setPolarAndCalcRetCoords(self, polarCoords):
    self.linear = polarCoords[0]
    self.angular = polarCoords[1]
    self.x = self.linear * cos(self.angular * pi/180)
    self.y = self.linear * sin(self.angular * pi/180)
  
  def setRetAndCalcPolarCoords(self, retCoords):
    self.x = retCoords[0]
    self.y = retCoords[1]
    self.linear = (self.x**2 + self.y**2)**0.5
    self.angular = arctan2(self.y, self.x) * 180 / pi
    
  def setRetCoords(self, coords):
    self.x = coords[0]
    self.y = coords[1]
    
  def prn(self):
    print "(x,y): ", self.x, self.y, "Linear:", self.linear, "Angular:", self.angular
    
  def getPolarCoords(self):
    return self.linear, self.angular
  
  #apenas por semantica
  def getVelocities(self):
    return self.linear, self.angular
  
  def getRetCoords(self):
      return (self.x, self.y)
    
  def setAllCoords(self, coords):
      self.linear = coords.radius
      self.angular = coords.angle
      self.x = coords.x
      self.y = coords.y
   