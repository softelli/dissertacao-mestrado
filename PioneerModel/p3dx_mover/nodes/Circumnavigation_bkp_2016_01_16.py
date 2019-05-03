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
		
		#localizacao cartesiana
		self.beacon = 0.0
		self.robot = 0.0
		self.alien = 0.0


		#substituida por linear velocity, angular_velocity
		self.linearVelocity = sp.min_linear_velocity
		self.angularVelocity = sp.angular_velocity
		
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
		#calcula o coeficiente angular em relacao a distancia do beacon ao robo detectado
				
		#distancia do beacon ao robot
		obtainedDistanceFromBeaconToRobot = ((beacon.x - robot.x)**2 + (beacon.y - robot.y)**2)**0.5
		#atualiza atributo
		self.obtainedDistanceFromBeaconToRobot = self.H(self.hasRobot and self.hasBeacon, True) * obtainedDistanceFromBeaconToRobot
		
		minObtainedDistance = sp.desired_radius - sp.min_distance_to_robot
		maxObtainedDistance = sp.desired_radius + sp.min_distance_to_robot
		
		if ((minObtainedDistance < obtainedDistanceFromBeaconToRobot) and (obtainedDistanceFromBeaconToRobot < maxObtainedDistance )):
		        modularDistance = ((obtainedDistanceFromBeaconToRobot - sp.desired_radius)**2)**0.5
								
			return 1 - modularDistance / (2 * sp.min_distance_to_robot)
		else:
			return 0.0
		
		
	
	def getProximityCoefToRobot(self):
		#calcula o coeficiente de proximidade em relacao ao robo mais proximo
		#envolve a distancia obtida, distancia desejada e minima distancia entre robos
		#retorna [-0.5,0.5] *** plotar arquivo "proximityControlToRobot.py"
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
		
		
		      
	def printAll(self):
	        #print "[", num_id, "]:: Ci:", self.interferenceCoef, "Cc:", self.conversionCoef, "Co", self.orientationCoef, "Cp", self.proximityCoefToRobot, "Ca", self.forwardCoef

	        print "---------------------------"
	        print "---------------------------"
	        print("Velocidade Linear      (vL): %6.2f m/s" % (self.linearVelocity))
	        print("Velocidade Angular     (vA): %6.2f rad/s" % (self.angularVelocity))
	        #print("Velocidade Tang Direita    : %6.2f m/s" % (self.rightLinearVelocity))
	        #print("Rotacao Roda Direita       : %6.2f rad/s" % (self.rightWheelRotation))
	        #print("Velocidade Tang Esquerda   : %6.2f m/s" % (self.leftLinearVelocity))
	        #print("Rotacao Roda Esquerda      : %6.2f rad/s" % (self.leftWheelRotation))
	        
	        if(self.hasBeacon == True):
			print "---------------------------"
			#print("Beacon x,y                 : %6.2f, %6.2f " % (self.beacon))
			print("Angulo ao Beacon       (aB): %6.2f" % (self.obtainedAngleToBeacon))
			#print("Raio do Sensor         (Sr):     %6.2f" % (sp.sensor_cone_radius))
			#print("Raio desejado          (dR):     %6.2f" % (sp.desired_radius))
			print("Raio obtido            (oR): %6.2f" % (self.obtainedRadiusToBeacon))

			print("Circular Coef   Beacon aCtB: %6.2f" % (self.circularCoefToBeacon))
			
		if(self.hasRobot == True):	
			print "---------------------------"
			#print("Robot x,y                  : %6.2f, %6.2f " % (self.robot))
			print("Angulo ao Robot        (aR): %6.2f" % (self.obtainedAngleToRobot))
			#print("Min Distancia entre Rob(mD):     %6.2f" % (sp.min_distance_to_robot))
			#print("Distancia desejada     (dD):     %6.2f" % (sp.desired_distance_to_robot))
			print("Distancia obtida       (oD): %6.2f" % (self.obtainedDistanceToRobot))
			print("Coef. de proximidade       : %6.2f " % (self.proximityCoefToRobot))
			print("propAngularCoefToRobot     : %6.2f" % (self.propAngularCoefToRobot))
		
		if(self.hasRobot == True and self.hasBeacon == True):
			print("Distancia Beacon to Robot  : %6.2f" % (self.obtainedDistanceFromBeaconToRobot))
			print("Ang. Coef Beacon to Robot  : %6.2f" % (self.linearCoefBeaconToRobot))
	        print sp.status_msg[self.status], self.status
	        
	        
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
	      
	#devolve as velocidades linear e angular
	def getVelocities(self, numRobot , myVelocities, beaconCoord, robotCoord, alienCoord):
		#print "beacon coord", beaconCoord.getVelocities()
		#print "robot coord", robotCoord.getVelocities()
		#print "alien coord", alienCoord.getVelocities()
		
		#atualiza a existencia dos objetos
		self.updateDetectedObjects(beaconCoord, robotCoord, alienCoord)

		#age conforme os objetos detectados
		if(self.hasBeacon and self.hasRobot): #Escorting
			self.status = 3
			
			### getCircularCoefToBeacon() = getPropRadialCoefToBeacon() + getPropAngularCoefToBeacon() ###
			
			self.circularCoefToBeacon = self.getCircularCoefToBeacon()
			self.proximityCoefToRobot = self.getProximityCoefToRobot() * self.getPropAngularCoefToRobot()
			
			#a velocidade linear depende do angulo em relacao ao beacon
			self.linearVelocity = sp.linear_velocity + self.circularCoefToBeacon + self.proximityCoefToRobot		
			
			
			#mantenha o raio desejado
			self.angularVelocity  = self.linearVelocity / sp.desired_radius + self.circularCoefToBeacon
			
			self.linearCoefBeaconToRobot = self.getLinearCoefBeaconToRobot(beaconCoord, robotCoord)
			self.angularVelocity += self.linearCoefBeaconToRobot * self.getPropAngularCoefToRobot()
			
		elif(self.hasBeacon and not self.hasRobot): #Circulating
			self.status = 2
			self.circularCoefToBeacon = self.getCircularCoefToBeacon()
			#self.linearVelocity = sp.linear_velocity + self.circularCoefToBeacon
			self.angularVelocity = self.linearVelocity / sp.desired_radius + self.circularCoefToBeacon
			
		elif(not self.hasBeacon and self.hasRobot): #Avoiding
			self.status = 1
			#coeficiente de proximidade
			#calculado em funcao de (distancia minima, desejada e obtida), no intervalo [-0.5,0.5]
			#multiplicado pela
			self.proximityCoefToRobot = self.getProximityCoefToRobot()
			self.propAngularCoefToRobot = self.getPropAngularCoefToRobot()
			self.propDistCoefToRobot = self.getPropDistCoefToRobot()
			
			self.linearVelocity = sp.linear_velocity * (self.proximityCoefToRobot + self.propAngularCoefToRobot)/2.0

			self.angularVelocity = sp.max_angular_velocity
			
		else: #status == "Seeking"
		        self.status = 0
		        self.linearVelocity = sp.linear_velocity
			self.angularVelocity = sp.angular_velocity + sp.angular_velocity * np.random.random() * 0.1
			
		#novo controle angular
		#sensor_angle, obtained_angle, desired_angle, sensor_radius, obtained_radius, desired_radius#
		#self.circularCoefToBeacon = self.getCircularCoefToBeacon()
		
		#self.propDistCoefToRobot = self.getPropDistCoefToRobot()
		
		#novo controle do beacon ao robot
		#self.linearCoefBeaconToRobot = self.getLinearCoefBeaconToRobot(beaconCoord, robotCoord)/2.0
		
		#novo controle linear
		#self.proximityCoefToRobot = self.getProximityCoefToRobot() * self.getPropAngularCoefToRobot()
			
		#ativa se houver robot
		#self.linearVelocity = sp.linear_velocity + self.H(self.hasRobot, True) * self.proximityCoefToRobot 
		
		
		#http://143.106.148.168:9080/Cursos/IA368W/parte1.pdf pag 33
		#self.angularVelocity = self.linearVelocity / sp.desired_radius # rad/s
		
		
		#### VELOCIDADE ANGULAR #####
		#se nao houver beacon e talvez houver robot
		#self.angularVelocity = self.H(self.hasBeacon, False) * sp.init_angular_velocity + self.H(self.hasRobot, True) * self.getPropAngularCoefToRobot() 
		
		#ativa se houver beacon
		#self.angularVelocity  = self.H(self.hasBeacon, True) * (self.linearVelocity / sp.desired_radius + self.circularCoefToBeacon)  # rad/s
		
		#ativa se houver beacon e robot
		#self.angularVelocity += self.H((self.hasBeacon and self.hasRobot), True) * self.linearCoefBeaconToRobot
		
		#ativa se houver robot
		#self.angularVelocity += self.H(self.hasRobot, True) * self.propDistCoefToRobot/2.0
		
		### delimiter ###
		self.linearVelocity = self.getDelimitedLinearVelocity(self.linearVelocity)
		self.angularVelocity = self.getDelimitedAngularVelocity(self.angularVelocity)
		
		###------------------------------###
		### Atualizacao das propriedades ###
				
		myVelocities.angular = self.angularVelocity
		myVelocities.linear = self.linearVelocity
		
		#atualiza rotacoes e velocidades tangenciais #monitoramento
		self.rightLinearVelocity = self.angularVelocity + self.linearVelocity
		self.leftLinearVelocity  =  - 2 * self.linearVelocity - self.rightLinearVelocity
		self.rightWheelRotation  = self.rightLinearVelocity / (sp.wheel_diameter * np.pi)
		self.leftWheelRotation   = self.leftLinearVelocity / (sp.wheel_diameter * np.pi)
		
		if(numRobot == 0):
			self.printAll()
		
		return myVelocities


