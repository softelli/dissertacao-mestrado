from __future__ import division
from LocalObject import LocalObject
from polar_coord import LinAng

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
		self.obtainedDistanceFromBeaconToRobot = 0.0
		
		#novo controle
		self.circularCoefToBeacon = 0.0
		
		#novo controle
		self.propDistCoefToRobot = 0.0
		
		#novo controle
		self.linearCoefBeaconToRobot = 0.0
		
		#novo controle
		self.propAngularCoefToRobot = 0.0 
		
		#novo controle
		self.linearCoefToBeacon = 0.0
		
		#self
		self.proximityCoefToRobot = 0.0
		
		#self
		self.actualTime = 0.0
		
		#self
		self.previousTime = 0.0
		
		#self
		self.approachRadius = 0.0
		
		
		#localizacao x,y , com base no frame centralizado em mim
		self.beacon = LinAng()
		self.robot  = LinAng()
		self.alien  = LinAng()


		#substituida por linear velocity, angular_velocity
		self.linearVelocity = sp.linear_velocity
		self.angularVelocity = sp.angular_velocity
		
		self.realLinearVelocity = 0.0
		self.realAngularVelocity = 0.0
		
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
		
		#Status
		self.status = 0
		
	def setTime(self, time):
		self.previousTime = self.actualTime
		self.actualTime = time
		
	def getApproachRadius(self):
		#retorna o raio de abordagem, para realizacao de um arco tangente
		radius = 0.0
		#self.beacon.prn()
		#lidando com a descontinuidade
		# quando y do beacon for o inverso do raio desejado
		if (self.beacon.y == sp.desired_radius):
			if (self.beacon.angular == 90):
				radius = sp.desired_radius
			else:
				radius = 0.0
		else:
			if (self.beacon.linear > sp.desired_radius) and (self.beacon.angular < 90):
			        #print self.actualTime, "beacon.linear <= sp.desired_radius and self.beacon.angular < 90"
				radius = -(self.beacon.x**2 + self.beacon.y**2 - sp.desired_radius**2) / (2 * (sp.desired_radius - self.beacon.y))
				#utilizando o modulo de y
				#radius = -(self.beacon.x**2 + self.beacon.y**2 - sp.desired_radius**2) / (2 * (sp.desired_radius - (self.beacon.y**2)**0.5))
			else:
				
				if(self.beacon.angular < sp.desired_angle_to_beacon):
					fator = (self.obtainedAngleToBeacon + sp.desired_angle_to_beacon)/(2 * sp.desired_angle_to_beacon)
					#print self.actualTime, "self.beacon.angular < sp.desired_angle_to_beacon"
					
					radius = self.linearVelocity / (sp.min_angular_velocity * fator)                                               
				else:
					radius = ((self.beacon.x + sp.desired_radius)**2 + self.beacon.y**2)/(2 * self.beacon.y)
					#print self.actualTime, "self.beacon.angular >= sp.desired_angle_to_beacon"
				#print self.actualTime, "beacon.linear > sp.desired_radius"
				#print "fi = ", 90 - self.beacon.angular
				#fi = (90 - self.beacon.angular) * np.pi/180
				#m = (self.beacon.linear + sp.desired_radius) * np.sin(fi)
				#n = (self.beacon.linear + sp.desired_radius) * np.cos(fi)
				#print "m: ", m, " n:", n
				#radius = (n**2 + m**2)/(2*n)
				
				
		print("%3.9f | %3.2f | %3.9f" % (self.beacon.linear, self.beacon.angular, radius))
		
		#se o raio for maior do que o permitido
		#if radius**2 > sp.max_approach_radius**2:
			#radius = 0.0
		  
		return radius
		
	      
	def getPropAngularCoefToBeacon(self):
		#sensor_angle, obtained_angle, desired_angle
		#calculo o coeficiente proporcional angular em relacao do angulo ao Beacon
		#envolve angulo do sensor, angulo desejado e angulo obtido
		#retorna 0.0 quando angulo_obtido = angulo_desejado
		#intervalo de retorno [-1.0, 1.0] *** plotar arquivo: "proportionalAngularToBeacon.py"
		
		min_angle = 2 * sp.desired_angle_to_beacon - sp.sensor_cone_angle / 2
		max_angle = sp.sensor_cone_angle / 2
	
		if   self.obtainedAngleToBeacon < min_angle:
			return -1.0
		elif self.obtainedAngleToBeacon > max_angle:
			return 1.0
		else:
			return 2 * (self.obtainedAngleToBeacon - min_angle) / (max_angle - min_angle) - 1
			
	def getPropRadialCoefToBeacon(self):
		#sensor_radius, obtained_radius, desired_radius
		#calcula o valor do coeficiente com base no raio ao Beacon
		#envolve raio do sensor, raio desejado e raio obtido
		#retorna 0.0 quando raio_obtido = raio_desejado
		#intervalo de retorno [-1.0, 1.0] *** plotar arquivo: "proportionalRadialToBeacon.py"
		
		return 2 * (self.obtainedRadiusToBeacon - sp.desired_radius) / sp.sensor_cone_radius
	      
	def getCircularCoefToBeacon(self):
		#calcula o coeficiente para circular o beacon
		#quando ha o Beacon, retorna a media dos valores obtidos por getProportionalAngularToBeacon e getProportionalRadialToBeacon
		if self.obtainedRadiusToBeacon == 0.0:
			return 0.0	
		return (self.getPropAngularCoefToBeacon() + self.getPropRadialCoefToBeacon())/2.0
	      
	def getPropAngularCoefToRobot(self):
		#calcula o valor do coeficiente em relacao ao angulo ao robo (evitar colisao)
		#envolve o angulo obtido e o angulo do sensor
		#retorna valor [0.0, 1.0] *** plotar arquivo "getPropAngularCoefToRobot.py"
		
		angle_to_robot = (self.obtainedAngleToRobot**2)**0.5  #apenas valores positivos
		return (1 - angle_to_robot / (sp.sensor_cone_angle / 2.0))**2
	
	def getPropDistCoefToRobot(self):
		#calcula coeficiente linear relacao a posicao do robo
		#apenas se a distancia obtida for menor do que a desejada
		#envolve a distancia obtida e a desejada em relacao ao robot
		#retorna valores no intervalo [0.0,1.0] *** plotar arquivo "getPropDistCoefToRobot"
		if(self.obtainedDistanceToRobot < sp.desired_distance_to_robot):
			return 1 - self.obtainedDistanceToRobot / sp.desired_distance_to_robot
		else:
			return 0.0
	
	def getLinearCoefBeaconToRobot(self, beacon, robot):
		#calcula o coeficiente linear em relacao a distancia do beacon ao robo detectado
		#plotar arquivo getLinearCoefBeaconToRobot.py
		#distancia do beacon ao robot
		obtainedDistanceFromBeaconToRobot = ((beacon.x - robot.x)**2 + (beacon.y - robot.y)**2)**0.5
		#atualiza atributo
		self.obtainedDistanceFromBeaconToRobot = self.H(self.hasRobot and self.hasBeacon, True) * obtainedDistanceFromBeaconToRobot
		
		minObtainedDistance = sp.desired_radius - sp.min_distance_to_robot
		maxObtainedDistance = sp.desired_radius + sp.min_distance_to_robot
		
		if ((minObtainedDistance < obtainedDistanceFromBeaconToRobot) and (obtainedDistanceFromBeaconToRobot < maxObtainedDistance )):
		        modularDistance = ((obtainedDistanceFromBeaconToRobot - sp.desired_radius)**2)**0.5
								
			return 1 - modularDistance / sp.min_distance_to_robot
		else:
			return 0.0
		
		
	
	def getProximityCoefToRobot(self):
		#calcula o coeficiente de proximidade em relacao ao robo mais proximo
		#envolve a distancia obtida, distancia desejada e minima distancia entre robos
		#retorna [-0.5,0.5] *** plotar arquivo "getProximityCoefToRobot.py"
		#quando a distancia desejada for igual obtida, retorna 0
		
		if self.obtainedDistanceToRobot > 2 * sp.desired_distance_to_robot:
			return  0.5
		if self.obtainedDistanceToRobot < sp.min_distance_to_robot:
			return -0.5
		#senao, calcule y = (x - dD) / (2 * (dD - mD))
		return (self.obtainedDistanceToRobot - sp.desired_distance_to_robot) / (2 * (sp.desired_distance_to_robot - sp.min_distance_to_robot))
		  
	def H(self, value, reference):
		#Heaviside function
		if value >= reference:
		    return 1
		else:
		    return 0
		
		
		      
	def printAll(self, num_id):
	        #print "[", num_id, "]:: Ci:", self.interferenceCoef, "Cc:", self.conversionCoef, "Co", self.orientationCoef, "Cp", self.proximityCoefToRobot, "Ca", self.forwardCoef
	        print "-------- P", num_id, " ---------"
	        print("Robot Real Velocities (vL,vA): %6.2f m/s, %6.2f rad/s " % (self.realLinearVelocity,self.realAngularVelocity))
	        print("p3dx_mover velocities (vL,vA): %6.2f m/s, %6.2f rad/s " % (self.linearVelocity,self.angularVelocity))
	        #print("Velocidade Tang Direita    : %6.2f m/s" % (self.rightLinearVelocity))
	        #print("Rotacao Roda Direita       : %6.2f rad/s" % (self.rightWheelRotation))
	        #print("Velocidade Tang Esquerda   : %6.2f m/s" % (self.leftLinearVelocity))
	        #print("Rotacao Roda Esquerda      : %6.2f rad/s" % (self.leftWheelRotation))
		#print("Beacon x,y                 : %6.2f, %6.2f " % (self.beacon))
		print(        "Beacon (Raio, Angulo): %6.2f, %6.2f " % (self.obtainedRadiusToBeacon, self.obtainedAngleToBeacon))
		#print("Raio do Sensor         (Sr):     %6.2f" % (sp.sensor_cone_radius))
		#print("Raio desejado          (dR):     %6.2f" % (sp.desired_radius))
		#print("Circular Coef   Beacon aCtB: %6.2f" % (self.circularCoefToBeacon))	
		#print("Robot x,y                  : %6.2f, %6.2f " % (self.robot))
		print("        Robot (Dist, Angulo) : %6.2f, %6.2f" % (self.obtainedDistanceToRobot, self.obtainedAngleToRobot))
		#print("Min Distancia entre Rob(mD):     %6.2f" % (sp.min_distance_to_robot))
		#print("Distancia desejada     (dD):     %6.2f" % (sp.desired_distance_to_robot))
		#print("proximityCoefToRobot       : %6.2f " % (self.proximityCoefToRobot))
		#print("propAngularCoefToRobot     : %6.2f" % (self.propAngularCoefToRobot)) 
		#print("propDistCoefToRobot        : %6.2f" % (self.propDistCoefToRobot))			
		#print("Distancia Beacon to Robot  : %6.2f" % (self.obtainedDistanceFromBeaconToRobot))
		#print("Ang. Coef Beacon to Robot  : %6.2f" % (self.linearCoefBeaconToRobot))
	        print sp.status_msg[self.status], self.status
	        print ""
	        print ""
	        
	        
	#atualiza a existencia de objetos detectados
	def updateDetectedObjects(self, detectedBeaconDist, detectedRobotDist, detectedAlienDist):
	        
		if detectedBeaconDist.linear > 0.0:
			self.hasBeacon = True
			self.obtainedRadiusToBeacon, self.obtainedAngleToBeacon = detectedBeaconDist.getPolarCoords()
			self.beacon.setRetCoords(detectedBeaconDist.getRetCoords())
			self.beacon.setPolarCoords(detectedBeaconDist.getPolarCoords())
		else:
			self.hasBeacon = False
			self.obtainedRadiusToBeacon, self.obtainedAngleToBeacon = 0.0, 0.0
			self.beacon.clear()

		if detectedRobotDist.linear > 0.0:
			self.hasRobot = True
			self.obtainedDistanceToRobot, self.obtainedAngleToRobot = detectedRobotDist.getPolarCoords()
			self.robot.setRetCoords(detectedRobotDist.getRetCoords())
			self.robot.setPolarCoords(detectedBeaconDist.getPolarCoords())
		else:
			self.hasRobot = False
			self.obtainedDistanceToRobot, self.obtainedAngleToRobot = 0.0, 0.0
			self.robot.clear()

		if detectedAlienDist.linear > 0.0:
			self.hasAlien = True
			self.obtainedDistanceToAlien, self.obtainedAngleToAlien = detectedAlienDist.getPolarCoords()
			self.alien.setRetCoords(detectedAlienDist.getRetCoords())
			self.alien.setPolarCoords(detectedBeaconDist.getPolarCoords())
		else:
			self.hasAlien = False
			self.obtainedDistanceToAlien, self.obtainedAngleToAlien = 0.0, 0.0
			self.alien.clear()
			
	def getDelimitedLinearVelocity(self, linearVelocity):
		if(linearVelocity < 0.0):
			return 0.0
		elif(linearVelocity > sp.max_linear_velocity):
			return sp.max_linear_velocity
		else:
			return linearVelocity
		      
	def getDelimitedAngularVelocity(self, angularVelocity):
		if(angularVelocity < sp.min_angular_velocity):
			return sp.min_angular_velocity
		elif(angularVelocity > sp.max_angular_velocity):
			return sp.max_angular_velocity
		else:
			return angularVelocity
		      
	def verifyMinDistances(self, numRobot, velocity):
		stop = False
		#prevents beacon collision
		if(0 < self.obtainedRadiusToBeacon) and (self.obtainedRadiusToBeacon < sp.min_distance_to_robot):
			if((self.obtainedAngleToBeacon**2)**0.5 < 45):
				stop = True
		elif(0 < self.obtainedDistanceToRobot) and (self.obtainedDistanceToRobot < sp.min_distance_to_robot):
			if((self.obtainedAngleToRobot**2)**0.5 < 45):
				stop = True
		if stop == True:
		    #print "XXXXXXX Stopping Linear Velocity in P", numRobot, "XXXXXXX"
		    velocity = 0.0
		return velocity
		
	#devolve as velocidades linear e angular
	def getVelocities(self, numRobot , myVelocities, beaconCoord, robotCoord, alienCoord, now):
		#print "beacon coord", beaconCoord.getVelocities()
		#print "robot coord", robotCoord.getVelocities()
		#print "alien coord", alienCoord.getVelocities()
		
		self.setTime(now.secs + now.nsecs / (10**9)) # segundos.milisegundos
		#print ("Tempo decorrido %4.9f" %(self.actualTime - self.previousTime))
		
		#atualiza a velocidade real
		self.realLinearVelocity, self.realAngularVelocity = myVelocities.getVelocities()
		
		#atualiza a existencia dos objetos
		self.updateDetectedObjects(beaconCoord, robotCoord, alienCoord)

		#age conforme os objetos detectados
		if(self.hasBeacon and self.hasRobot): #Escorting
			self.status = 3
			
			self.circularCoefToBeacon = self.getCircularCoefToBeacon()
			self.propAngularCoefToRobot = self.getPropAngularCoefToRobot()
			
			self.proximityCoefToRobot = self.getProximityCoefToRobot() # * self.getPropAngularCoefToRobot()
			
			self.linearCoefBeaconToRobot = self.getLinearCoefBeaconToRobot(beaconCoord, robotCoord)
			
			#self.linearVelocity = sp.linear_velocity + sp.linear_velocity * self.linearCoefBeaconToRobot * self.proximityCoefToRobot
			self.linearVelocity = sp.linear_velocity + sp.linear_velocity * self.propAngularCoefToRobot * self.proximityCoefToRobot
			
			#mantenha o raio desejado
			self.angularVelocity  = self.linearVelocity / sp.desired_radius + self.circularCoefToBeacon

			
		elif(self.hasBeacon and not self.hasRobot): #Circulating
			self.status = 2
			self.circularCoefToBeacon = self.getCircularCoefToBeacon()
			self.propRadialCoefToBeacon = self.getPropRadialCoefToBeacon()
			self.linearVelocity = sp.linear_velocity # / (1 + self.propRadialCoefToBeacon)# + (sp.max_linear_velocity - sp.min_linear_velocity) * self.circularCoefToBeacon
			self.approachRadius = self.getApproachRadius()
			if self.approachRadius == 0.0:
				self.angularVelocity = 0.0
			else:
				#if(self.obtainedAngleToBeacon > 0.8 * sp.desired_angle_to_beacon and self.obtainedAngleToBeacon < 1.2 * sp.desired_angle_to_beacon):
					#self.angularVelocity = self.linearVelocity / sp.desired_radius + self.circularCoefToBeacon
				#else:    
				self.angularVelocity = self.linearVelocity / self.approachRadius
			
			if self.angularVelocity > sp.max_angular_velocity:
				#print "*** clipping on MAX angular velocity ***"
				self.angularVelocity = sp.max_angular_velocity
				self.linearVelocity = self.approachRadius * self.angularVelocity
			 
			if self.angularVelocity < sp.min_angular_velocity:
				#print "*** clipping on MIN angular velocity ***"
				self.angularVelocity = sp.min_angular_velocity
				self.linearVelocity = self.approachRadius * self.angularVelocity
			#self.angularVelocity = self.linearVelocity / sp.desired_radius + (sp.max_angular_velocity - sp.min_angular_velocity) * self.circularCoefToBeacon
			
			
		elif(not self.hasBeacon and self.hasRobot): #Avoiding
			self.status = 1
			#coeficiente de proximidade
			#calculado em funcao de (distancia minima, desejada e obtida), no intervalo [-0.5,0.5]
			#multiplicado pela
			self.proximityCoefToRobot = self.getProximityCoefToRobot()
			self.propAngularCoefToRobot = self.getPropAngularCoefToRobot()
			self.propDistCoefToRobot = self.getPropDistCoefToRobot()
			
			self.linearVelocity = sp.linear_velocity + sp.linear_velocity * self.proximityCoefToRobot * self.propAngularCoefToRobot

			self.angularVelocity = self.linearVelocity / self.obtainedDistanceToRobot + sp.angular_velocity * self.proximityCoefToRobot * self.propDistCoefToRobot
			
		else: #status == "Seeking"
		        self.status = 0
		        self.linearVelocity = sp.linear_velocity
			self.angularVelocity = sp.angular_velocity + sp.angular_velocity * np.random.random() * 0.1
		
		### delimiter ###
		self.linearVelocity = self.getDelimitedLinearVelocity(self.linearVelocity)
		#self.angularVelocity = self.getDelimitedAngularVelocity(self.angularVelocity)
		
		### Parada de Seguranca ###
		self.linearVelocity = self.verifyMinDistances(numRobot, self.linearVelocity)
		
		###------------------------------###
		### Atualizacao das propriedades ###
				
		myVelocities.angular = self.angularVelocity
		myVelocities.linear  = self.linearVelocity
		
		#atualiza rotacoes e velocidades tangenciais #monitoramento
		self.rightLinearVelocity = myVelocities.angular + myVelocities.linear
		self.leftLinearVelocity  =  - 2 * myVelocities.linear - myVelocities.angular
		self.rightWheelRotation  = self.rightLinearVelocity / (sp.wheel_diameter * np.pi)
		self.leftWheelRotation   = self.leftLinearVelocity / (sp.wheel_diameter * np.pi)
		
		
		#self.printAll(numRobot)
		
		return myVelocities


