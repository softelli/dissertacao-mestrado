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
		self.angularControl = 0.0
		
		#novo controle
		self.linearControl = 0.0


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
		r = (obtained_radius - desired_radius) / sensor_radius
		
		return r
	      
	def aCtrl(self, sensor_angle, obtained_angle, desired_angle, sensor_radius, obtained_radius, desired_radius):
		if obtained_radius == 0.0:
			return 0.0
		
		pac = self.pAc(sensor_angle, obtained_angle, desired_angle)
		prc = self.pRc(sensor_radius, obtained_radius, desired_radius)
		
		return pac + prc
			
		      
	def printCoef(self):
	        #print "[", num_id, "]:: Ci:", self.interferenceCoef, "Cc:", self.conversionCoef, "Co", self.orientationCoef, "Cp", self.proximityCoef, "Ca", self.forwardCoef
	        
	        print("Angular Control       aCtrl: %6.2f" % (self.angularControl))
	        print "---------------------------"
	        print("Velocidade Linear      (vL): %6.2f m/s" % (self.linearVelocity))
	        print("Velocidade Angular     (vA): %6.2f rad/s" % (self.angularVelocity))
	        print("Velocidade Tang Direita    : %6.2f m/s" % (self.rightLinearVelocity))
	        print("Rotacao Roda Direita       : %6.2f rad/s" % (self.rightWheelRotation))
	        print("Velocidade Tang Esquerda   : %6.2f m/s" % (self.leftLinearVelocity))
	        print("Rotacao Roda Esquerda      : %6.2f rad/s" % (self.leftWheelRotation))
	        
	        print "---------------------------"
	        print("Angulo ao Beacon       (aB): %6.2f" % (self.obtainedAngleToBeacon))
	        print("Raio do Sensor         (Sr):     %6.2f" % (sp.sensor_cone_radius))
	        print("Raio desejado          (dR):     %6.2f" % (sp.desired_radius))
	        print("Raio obtido            (oR): %6.2f" % (self.obtainedRadiusToBeacon))       
	        print "---------------------------"
	        print("Angulo ao Robot        (aR): %6.2f" % (self.obtainedAngleToRobot))
	        print("Min Distancia entre Rob(mD):     %6.2f" % (sp.min_distance_to_robot))
	        print("Distancia desejada     (dD):     %6.2f" % (sp.desired_distance_to_robot))
	        print("Distancia obtida       (oD): %6.2f" % (self.obtainedDistanceToRobot))
	       
	        
	        
	        
	#atualiza a existencia de objetos detectados
	def updateDetectedObjects(self, detectedBeaconDist, detectedRobotDist, detectedAlienDist):
	        #print "detected beacon"
	        #detectedBeaconDist.prn()
	        #print "detected robot"
	        #detectedRobotDist.prn()
	        #print "detected alien"
	        #detectedAlienDist.prn()
	        
		if detectedBeaconDist.linear > 0.0:
			self.hasBeacon = True
			self.obtainedRadiusToBeacon = detectedBeaconDist.linear
			self.obtainedAngleToBeacon = detectedBeaconDist.angular
		else:
			self.hasBeacon = False
			self.obtainedRadiusToBeacon = 0.0
			self.obtainedAngleToBeacon = 0.0

		if detectedRobotDist.linear > 0.0:
			self.hasRobot = True
			self.obtainedDistanceToRobot = detectedRobotDist.linear
			self.obtainedAngleToRobot = detectedRobotDist.angular	
		else:
			self.hasRobot = False
			self.obtainedDistanceToRobot = 0.0
			self.obtainedAngleToRobot = 0.0

		if detectedAlienDist.linear > 0.0:
			self.hasAlien = True
			self.obtainedDistanceToAlien = detectedAlienDist.linear
			self.obtainedAngleToAlien = detectedAlienDist.angular
		else:
			self.hasAlien = False
			self.obtainedDistanceToAlien = 0.0
			self.obtainedAngleToAlien = 0.0		
	        
	#devolve as velocidades linear e angular
	def process(self, myVelocities, beaconCoord, robotCoord, alienCoord):
		#print "beacon coord", beaconCoord.getVelocities()
		#print "robot coord", robotCoord.getVelocities()
		#print "alien coord", alienCoord.getVelocities()
		
		#atualiza a existencia dos objetos
		self.updateDetectedObjects(beaconCoord, robotCoord, alienCoord)
				
		#novo controle angular
		#sensor_angle, obtained_angle, desired_angle, sensor_radius, obtained_radius, desired_radius#
		self.angularControl = self.aCtrl(sp.sensor_cone_angle, self.obtainedAngleToBeacon, sp.desired_angle_to_beacon, sp.sensor_cone_radius, self.obtainedRadiusToBeacon, sp.desired_radius)
		
		#novo controle linear
		self.linearControl = 0.0
		
		#utilizando aF
		#self.angularVelocity = self.aV(self.angularVelocity, self.aF(self.obtainedAngleToBeacon, sp.desired_angle_to_beacon))
		
		#apenas teste para o resultados
		self.linearVelocity  = 0.5 # m/s
		
		#http://143.106.148.168:9080/Cursos/IA368W/parte1.pdf pag 33
		#self.angularVelocity = self.linearVelocity / sp.desired_radius # rad/s
		
		self.angularVelocity = self.linearVelocity / sp.desired_radius + self.angularControl  # rad/s
				
		myVelocities.angular = self.angularVelocity
		myVelocities.linear = self.linearVelocity
		
		#atualiza rotacoes e velocidades tangenciais #monitoramento
		self.rightLinearVelocity = self.angularVelocity + self.linearVelocity
		self.leftLinearVelocity  =  - 2 * self.linearVelocity - self.rightLinearVelocity
		self.rightWheelRotation  = self.rightLinearVelocity / (sp.wheel_diameter * np.pi)
		self.leftWheelRotation   = self.leftLinearVelocity / (sp.wheel_diameter * np.pi)
		
		self.printCoef()
		
		return myVelocities


