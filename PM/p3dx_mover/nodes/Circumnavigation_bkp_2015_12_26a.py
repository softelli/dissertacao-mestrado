from __future__ import division
from LocalObject import LocalObject
import math
import random
import numpy as np


#static parameters
from ParametersServer import staticParameters
sp = staticParameters

class circum:

	def __init__(self):

		#Raio obtido
		self.obtainedRadiusToBeacon = 0.0

		#Angulo Obtido em Relacao ao Beacon
		self.obtainedAngleToBeacon = 0.0

		#Angulo Obtido em Relacao ao Robo
		self.obtainedAngleToRobot = 0.0
		
		#Distancia obtida
		self.obtainedDistanceToRobot = 0.0		

		#Distancia obtida ao alien
		self.obtainedDistanceToAlien = 0.0
		
		#angulo obtido ao alien
		self.obtainedAngleToAlien = 0.0
		
		#novo controle
		self.angularCoefToBeacon = 0.0
		
		#novo controle
		self.angularCoefBeaconToRobot = 0.0
		
		#novo controle
		self.linearCoefToBeacon = 0.0
		
		#localizacao cartesiana
		self.beacon = 0.0
		self.robot = 0.0
		self.alien = 0.0


		#substituida por linear velocity, angular_velocity
		self.linearVelocity = sp.min_linear_velocity
		self.angularVelocity = sp.init_angular_velocity
		
		#velocidade tangencial/linear da roda direita e esquerda #m/s
		self.rightLinearVelocity = 0.0
		self.leftLinearVelocity = 0.0
		
		#rotacao da roda direita e esquerda #r.p.s
		self.rightWheelRotation = 0.0
		self.leftWheelRotation = 0.0
		
		#Beacon detectado?
		self.hasBeacon = False

		#Robo detectado?
		self.hasRobot = False

		#Alien detectado?
		self.hasAlien = False
	      
	def pAc(self, sensor_angle, obtained_angle, desired_angle):
		#calculo da diferenca proporcional angular
		#min_angle = desired_angle - (sensor_angle / 2 - desired_angle)
		min_angle = 2 * desired_angle - sensor_angle / 2
		max_angle = sensor_angle / 2
		a = 0.0
		
		if   obtained_angle < min_angle:
			a = -1.0
		elif obtained_angle > max_angle:
			a = 1.0
		else:
			a = 2 * (obtained_angle - min_angle) / (max_angle - min_angle) - 1
			
		return a
			
	def pRc(self, sensor_radius, obtained_radius, desired_radius):
		#calculo da diferenca proporcional radial
		r = 2 * (obtained_radius - desired_radius) / sensor_radius
		
		return r
	      
	def aCtB(self, sensor_angle, obtained_angle, desired_angle, sensor_radius, obtained_radius, desired_radius):
		p = 1.0
		if obtained_radius == 0.0:
			p = 0.0
		
		pac = self.pAc(sensor_angle, obtained_angle, desired_angle)
		prc = self.pRc(sensor_radius, obtained_radius, desired_radius)
		
		return (pac + prc) * p
	
	def acBtR(self, beacon, robot, desiredRadius, obtainedRadius):
		#distancia do beacon ao robot		
		odBtR = ((beacon.x - robot.x)**2 + (beacon.y - robot.y)**2)**0.5
		return 1 - (2 * odBtR / (desiredRadius + obtainedRadius))
	      
	def H(self, value, reference):
		#Heaviside function
		if value >= reference:
		    return 1
		else:
		    return 0
		
		
		      
	def printAll(self):
	        #print "[", num_id, "]:: Ci:", self.interferenceCoef, "Cc:", self.conversionCoef, "Co", self.orientationCoef, "Cp", self.proximityCoef, "Ca", self.forwardCoef
	        

	        print "---------------------------"
	        print("Velocidade Linear      (vL): %6.2f m/s" % (self.linearVelocity))
	        print("Velocidade Angular     (vA): %6.2f rad/s" % (self.angularVelocity))
	        print("Velocidade Tang Direita    : %6.2f m/s" % (self.rightLinearVelocity))
	        print("Rotacao Roda Direita       : %6.2f rad/s" % (self.rightWheelRotation))
	        print("Velocidade Tang Esquerda   : %6.2f m/s" % (self.leftLinearVelocity))
	        print("Rotacao Roda Esquerda      : %6.2f rad/s" % (self.leftWheelRotation))
	        
	        print "---------------------------"
		print("Beacon x,y                 : %6.2f, %6.2f " % (self.beacon))
	        print("Angulo ao Beacon       (aB): %6.2f" % (self.obtainedAngleToBeacon))
	        print("Raio do Sensor         (Sr):     %6.2f" % (sp.sensor_cone_radius))
	        print("Raio desejado          (dR):     %6.2f" % (sp.desired_radius))
	        print("Raio obtido            (oR): %6.2f" % (self.obtainedRadiusToBeacon))
	        print("Angular Coef to Beacon aCtB: %6.2f" % (self.angularCoefToBeacon))
	        print "---------------------------"
	        print("Robot x,y                  : %6.2f, %6.2f " % (self.robot))
	        print("Angulo ao Robot        (aR): %6.2f" % (self.obtainedAngleToRobot))
	        print("Min Distancia entre Rob(mD):     %6.2f" % (sp.min_distance_to_robot))
	        print("Distancia desejada     (dD):     %6.2f" % (sp.desired_distance_to_robot))
	        print("Distancia obtida       (oD): %6.2f" % (self.obtainedDistanceToRobot))
	        print("Ang. Coef Beacon to Robot  : %6.2f" % (self.angularCoefBeaconToRobot))
	        
	        
	        
	#atualiza a existencia de objetos detectados
	def updateDetectedObjects(self, detectedBeaconDist, detectedRobotDist, detectedAlienDist):
	        
		if detectedBeaconDist.linear > 0.0:
			self.hasBeacon = True
			self.obtainedRadiusToBeacon, self.obtainedAngleToBeacon = detectedBeaconDist.getPolarCoords()
			self.beacon = detectedBeaconDist.getRetCoords()
		else:
			self.hasBeacon = False
			self.obtainedRadiusToBeacon, self.obtainedAngleToBeacon = 0.0, 0.0
			self.beacon = 0.0, 0.0

		if detectedRobotDist.linear > 0.0:
			self.hasRobot = True
			self.obtainedDistanceToRobot, self.obtainedAngleToRobot = detectedRobotDist.getPolarCoords()
			self.robot = detectedRobotDist.getRetCoords()
		else:
			self.hasRobot = False
			self.obtainedDistanceToRobot, self.obtainedAngleToRobot = 0.0, 0.0
			self.robot = 0.0, 0.0

		if detectedAlienDist.linear > 0.0:
			self.hasAlien = True
			self.obtainedDistanceToAlien, self.obtainedAngleToAlien = detectedAlienDist.getPolarCoords()
			self.alien = detectedAlienDist.getRetCoords()
		else:
			self.hasAlien = False
			self.obtainedDistanceToAlien, self.obtainedAngleToAlien = 0.0, 0.0
			self.alien = 0.0, 0.0
	        
	#devolve as velocidades linear e angular
	def getVelocities(self, myVelocities, beaconCoord, robotCoord, alienCoord):
		#print "beacon coord", beaconCoord.getVelocities()
		#print "robot coord", robotCoord.getVelocities()
		#print "alien coord", alienCoord.getVelocities()
		
		#atualiza a existencia dos objetos
		self.updateDetectedObjects(beaconCoord, robotCoord, alienCoord)
				
		#novo controle angular
		#sensor_angle, obtained_angle, desired_angle, sensor_radius, obtained_radius, desired_radius#
		self.angularCoefToBeacon = self.aCtB(sp.sensor_cone_angle, self.obtainedAngleToBeacon, sp.desired_angle_to_beacon, sp.sensor_cone_radius, self.obtainedRadiusToBeacon, sp.desired_radius)
		
		#novo controle do beacon ao robot
		self.angularCoefBeaconToRobot = self.acBtR(beaconCoord, robotCoord, sp.desired_radius, self.obtainedRadiusToBeacon)
		
		#novo controle linear
		self.linearControl = 0.0
		
		#apenas teste para o resultados
		self.linearVelocity  = 0.5 # m/s
		
		#http://143.106.148.168:9080/Cursos/IA368W/parte1.pdf pag 33
		#self.angularVelocity = self.linearVelocity / sp.desired_radius # rad/s
		
		self.angularVelocity  = self.H(self.hasBeacon, True) * (self.linearVelocity / sp.desired_radius + self.angularCoefToBeacon)  # rad/s
		self.angularVelocity += self.H(self.hasBeacon, False) * sp.init_angular_velocity
		self.angularVelocity += self.H(self.hasBeacon and self.hasRobot, True) * self.angularCoefBeaconToRobot 
		
		myVelocities.angular = self.angularVelocity
		myVelocities.linear = self.linearVelocity
		
		#atualiza rotacoes e velocidades tangenciais #monitoramento
		self.rightLinearVelocity = self.angularVelocity + self.linearVelocity
		self.leftLinearVelocity  =  - 2 * self.linearVelocity - self.rightLinearVelocity
		self.rightWheelRotation  = self.rightLinearVelocity / (sp.wheel_diameter * np.pi)
		self.leftWheelRotation   = self.leftLinearVelocity / (sp.wheel_diameter * np.pi)
		
		self.printAll()
		
		return myVelocities


