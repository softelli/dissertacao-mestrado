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
		
		#nova dimensao
		self.obtainedDistanceBeaconToRobot = 0.0
		
		#novo controle
		self.angularCoefToBeacon = 0.0
		
		#novo controle
		self.angularCoefToRobot = 0.0
		
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
	      
	def proportionalAngular(self):
		#sensor_angle, obtained_angle, desired_angle
		#calculo da diferenca proporcional angular
		#min_angle = desired_angle - (sensor_angle / 2 - desired_angle)
		min_angle = 2 * sp.desired_angle_to_beacon - sp.sensor_cone_angle / 2
		max_angle = sp.sensor_cone_angle / 2
	
		if   self.obtainedAngleToBeacon < min_angle:
			return -1.0
		elif self.obtainedAngleToBeacon > max_angle:
			return 1.0
		else:
			return 2 * (self.obtainedAngleToBeacon - min_angle) / (max_angle - min_angle) - 1
			
	def proportionalRadial(self):
		#sensor_radius, obtained_radius, desired_radius
		#calculo da diferenca proporcional radial
		return 2 * (self.obtainedRadiusToBeacon - sp.desired_radius) / sp.sensor_cone_radius
	      
	def angularControlToBeacon(self):
		if self.obtainedRadiusToBeacon == 0.0:
			return 0.0		      
		return self.proportionalAngular() + self.proportionalRadial()
	      
	def pIc(self):
		#calcula o valor do coeficiente em relacao ao angulo do robo
		angle_to_robot = (self.obtainedAngleToRobot**2)**0.5  #apenas valores positivos
		#print "pIc =", 1 - angle_to_robot / (sp.sensor_cone_angle / 2.0)
		return 1 - angle_to_robot / (sp.sensor_cone_angle / 2.0)
	
	def angularControlToRobot(self):
		#calcula coeficiente angular em relacao a posicao do robo
		#apenas se a distancia obtida for menor do que a desejada
		if(self.obtainedDistanceToRobot < sp.desired_distance_to_robot):
			return 1 - self.obtainedDistanceToRobot / sp.desired_distance_to_robot
		else:
			return 0.0
	
	def acBtR(self, beacon, robot):
		#calcula o coeficiente angular em relacao a distancia do beacon ao robo detectado
				
		#distancia do beacon ao robot
		odBtR = ((beacon.x - robot.x)**2 + (beacon.y - robot.y)**2)**0.5
		#atualiza atributo
		self.obtainedDistanceBeaconToRobot = self.H(self.hasRobot and self.hasBeacon, True) * odBtR
		
		od_min = sp.desired_radius - sp.min_distance_to_robot
		od_max = sp.desired_radius + sp.min_distance_to_robot
		if ((od_min < odBtR) and (odBtR < od_max)):
		        fr = odBtR - sp.desired_radius
			fr = (fr**2)**0.5 #apenas valores positivos
								
			return 1 - fr / (2 * sp.min_distance_to_robot)
		else:
			return 0.0
		
		
	
	def linearVelocityControl(self):
		#calcula
		if self.obtainedDistanceToRobot > 2 * sp.desired_distance_to_robot:
			return  1.0
		if self.obtainedDistanceToRobot < sp.min_distance_to_robot:
			return -1.0
		#senao, calcule y = (x - dD) / (2 * (dD - mD))
		return (self.obtainedDistanceToRobot - sp.desired_distance_to_robot) / (2 * (sp.desired_distance_to_robot - sp.min_distance_to_robot))
		  
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
	        print("Controle vel. Linear(Robot): %6.2f " % (self.linearControl))
	        
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
	        print("Distancia Beacon Robot     : %6.2f" % (self.obtainedDistanceBeaconToRobot))
	        print("Ang. Coef Beacon to Robot  : %6.2f" % (self.angularCoefBeaconToRobot))
	        print "hasBeacon", self.hasBeacon, "hasRobot", self.hasRobot
	        
	        
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
		self.angularCoefToBeacon = self.angularControlToBeacon()
		
		self.angularCoefToRobot = self.angularControlToRobot()
		
		#novo controle do beacon ao robot
		self.angularCoefBeaconToRobot = self.acBtR(beaconCoord, robotCoord)/2.0
		
		#novo controle linear
		self.linearControl = self.linearVelocityControl() * self.pIc()
		
		#### VELOCIDADE LINEAR ####
		#ativa se houver robot
		self.linearVelocity = sp.linear_velocity + self.H(self.hasRobot, True) * self.linearControl 
		
		
		#http://143.106.148.168:9080/Cursos/IA368W/parte1.pdf pag 33
		#self.angularVelocity = self.linearVelocity / sp.desired_radius # rad/s
		
		
		#### VELOCIDADE ANGULAR #####
		#se nao houver beacon e talvez houver robot
		self.angularVelocity = self.H(self.hasBeacon, False) * sp.init_angular_velocity + self.H(self.hasRobot, True) * self.pIc() 
		
		#ativa se houver beacon
		self.angularVelocity  = self.H(self.hasBeacon, True) * (self.linearVelocity / sp.desired_radius + self.angularCoefToBeacon)  # rad/s
		
		#ativa se houver beacon e robot
		#self.angularVelocity += self.H((self.hasBeacon and self.hasRobot), True) * self.angularCoefBeaconToRobot
		
		#ativa se houver robot
		#self.angularVelocity += self.H(self.hasRobot, True) * self.angularCoefToRobot/2.0 
		
		myVelocities.angular = self.angularVelocity
		myVelocities.linear = self.linearVelocity
		
		#atualiza rotacoes e velocidades tangenciais #monitoramento
		self.rightLinearVelocity = self.angularVelocity + self.linearVelocity
		self.leftLinearVelocity  =  - 2 * self.linearVelocity - self.rightLinearVelocity
		self.rightWheelRotation  = self.rightLinearVelocity / (sp.wheel_diameter * np.pi)
		self.leftWheelRotation   = self.leftLinearVelocity / (sp.wheel_diameter * np.pi)
		
		self.printAll()
		
		return myVelocities


