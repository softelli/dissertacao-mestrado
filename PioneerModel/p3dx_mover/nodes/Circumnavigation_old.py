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

		#forca
		self.force = 1.0

		#Forca dos coeficientes sobre o Robo
		self.coefficienteForce = 1.0

		#Coeficiente de orientacao
		self.orientationCoef = 1.0

		#Coeficiente de Interferencia...
		self.interferenceCoef = 1.0

		#Coeficiente de Proximdiade
		self.proximityCoef = 1.0

		#Coeficiente de aceleracao
		self.forwardCoef = 1.0

		#Coeficiente de Conversao
		self.conversionCoef = 0.0

		#Coeficiente de Velocidade
		self.linearCoef = 1.0

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

		#Angulo Mais proximo
		self.closerAngle = 0.0

		#Coeficiente de Relacao entre Raio Desejado e Obtido
		self.relativeDiffInRadius = 0.0
		
		#Coeficiente angular
		self.angularCoef = 0.1
		
		#novo controle
		self.angularControl = 0.0
		
		#novo controle
		self.linearControl = 0.0

		#Dimensoes do cone do sensor
		self.sensorRadius = sp.sensor_cone_radius
		self.sensorAngle = sp.sensor_cone_angle

		#substituida por linear velocity, angular_velocity
		self.maxLinearVelocity = sp.max_linear_velocity
		self.linearVelocity = sp.min_linear_velocity
		self.angularVelocity = sp.init_angular_velocity
		
		#velocidade tangencial/linear da roda direita m/s
		self.rightLinearVelocity = 0.0
		
		#velocidade tangencial/linear da roda esquerda m/s
		self.leftLinearVelocity = 0.0
		
		#rotacao da roda direita r.p.s
		self.rightWheelRotation = 0.0
		
		#rotacao da roda esquerda r.p.s
		self.leftWheelRotation = 0.0
		
		#Beacon detectado?
		self.hasBeacon = False

		#Robo detectado?
		self.hasRobot = False

		#Alien detectado?
		self.hasAlien = False


	#diferenca relativa entre o raio obtido e o desejado
	def rDiR(self, dRtB, oRtB):
	        #dRtB = desiredRadiusToBeacon
	        #oRtB = obtainedRadiusToBeacon
		#return relativeDiffInRadius
		return (dRtB - oRtB) / dRtB	      
		

	#coeficiente de velocidade linear
	def lC(self, pC, fC, iC):
		#pC = proximityCoef
		#fC = forwardCoef
		#iC = interferenceCoef
		#return linearCoef
		return pC + 2.0 * fC * iC
		
	#coeficiente de conversao #eliminado o halfSensorAngle e o mirrorAngleToBeacon
	def cC(self, oAtB, dAtB):
		#oAtB = obtainedAngleToBeacon
		#dAtB = desiredAngleToBeacon
		#return conversionCoef
		return 1.0 - (oAtB / dAtB)
	      
		#codigo simplificado
		#se nao funcionar, voltar ao codigo do NETLOGO
		#AnguloObtido > AnguloDesejado, retorna 0 > valor >= -0.5
		#AnguloObtido = AnguloDesejado, retorna 0
		#AnguloObtido < AnguloDesejado, retorna 0 < valor <= 2.5
				
	#coeficiente angular (para incremento na velocidade angular) #2015.11.07 alterado
	def aC(self, cF, rDiR, cC, iC):
		#cF: coefficienteForce
		#rDiR: relativeDiffInRadius
		#cC: updateConversionCoef
		#iC: interferenceCoef
		#return angularCoef
		
		#divide by zero prevent
		
		return cF * rDiR - cC/iC
	
	#forca angular --- acrescentado para teste em 2015.11.07
	#def aF(self, oAtB, dAtB):
		#if not self.hasBeacon:
			#return 0.0
		#else:
			#return (oAtB - dAtB)/(dAtB * 2.5)
		
	#coeficiente de orientacao - modificcado pois angulo medio agora eh ZERO
	def oC(self,oAtR, Sa):
		#oAtR: obtainedAngleToRobot
		#Sa: sensorAngle
		#return orientationCoef
		#return 1 - self.plus(oAtR/(Sa/2.0))
		return 1 - 2 * ((oAtR**2)**0.5)/Sa
		
		#oAtR =   0.0: retorna 1
		#oAtR =  Sa/2: retorna 0
		#oAtR = -Sa/2: retorna 0
	      
	#Velocidade Angular -- essa abordagem sera utilizada aqui????
	def aV(self,aV,aC):
		#aV: angularVelocity (actual)
		#aC: angularCoef
		#return angularVelocity (new)
		## aV = aC ## utilizar quando aC == aF
		aV += aC
		aV = self.limit(sp.min_angular_velocity, aV, sp.max_angular_velocity)
		return aV
		
	#Coeficiente de Interferencia
	def iC(self,oRtB, dRtB):
		#oRtB: obtainedRadiusToBeacon
		#dRtB: desiredRadiusToBeacon
		#return interferenceCoef (always positive)
		#return self.plus((oRtB - dRtB)/ dRtB)
		return ((oRtB - dRtB)**2)**0.5 / dRtB
	      
		#oRtB = dR: 0.0
		#oRtB =  0: 1.0aC
		#oRtB > dR: iC > 0.0 
		#oRtB < dR: iC > 0.0 
		
	#Coeficiente de Proximidade
	def pC(self,oDtR, mDbR, sR):
		#oDtR: obtainedDistanceToRobot
		#mDbR: minDistanceBetweenRobots
		#sR: sensorRadius
		#return proximityCoef
		return 1 - ((oDtR - mDbR)/(sR - mDbR))**2
		
		#oDtR = mDbR: 1.0
		#oDtR = 0.0: ~= 1.0
		#oDtR = sR:   0.0
		

	#Coeficiente de Avanco - OK 2015.10.19
	def fC(self, oDtR, dDtR, sR, mDbR):
		#oDtR: obtainedDistanceToRobot
		#dDtR: desiredDistanceToRobot
		#sR: sensorRadius
		#mDbR: minDistanceBetweenRobots
		#return forwardCoef
		
		return (oDtR - dDtR) / (sR - dDtR + mDbR)
			  
		#(o - d)/(r - d + m), retornando
		#oDtR > dDtR: 0.0 < fC <= 1.0
		#oDtR = dDtR: 0.0
		#oDtR = mDbR: -1.0
		#oDtR < mDbR: -1.0
		
		#ca --> -1, quando o --> min
		#ca < -1, quando o < min
				
		
	#retorna sempre positivo
	#def plus(self,value):
		#if value >= 0.0:
		#	return value
		#else:
		#	return value * -1.0
		      
	def limit(self,min_value, value, max_value):
		#min_value:
		#max_value:
		#value:s
		#return value constrained by: min_value <= value <= max_value
		
		if value > max_value:
			return max_value
		if value < min_value:
			return min_value
		return value
	      
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
	        print("Coef. de interferencia (iC): %6.2f" % (self.interferenceCoef))
	        print("Coef. de conversao     (cC): %6.2f" % (self.conversionCoef))
	        print("Coef. de orientacao    (oC): %6.2f" % (self.orientationCoef))
	        print("Coef. de proximidade   (pC): %6.2f" % (self.proximityCoef))
	        print("Coef. de avanco        (fC): %6.2f" % (self.forwardCoef))
	        print("Coef. de vel linear    (lC): %6.2f" % (self.linearCoef))
	        print("Coef. de vel angular   (aC): %6.2f" % (self.angularCoef))
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
	        print("Diferenca entre raios  (rD): %6.2f" % (self.relativeDiffInRadius))        
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
		
		#atualizar a diferenca relativa entre os raios
		self.relativeDiffInRadius = self.rDiR(sp.desired_radius, self.obtainedRadiusToBeacon)
		
		#atualizar o Coeficiente de Interferencia #nao havendo Beacon, retorna 1.0
		self.interferenceCoef = self.iC(self.obtainedRadiusToBeacon, sp.desired_radius)
		
		#atualizar o Coeficiente de Conversao #nao havendo Beacon, retorna 1.0		
		self.conversionCoef = self.cC(self.obtainedAngleToBeacon, sp.desired_angle_to_beacon)
		
		#atualizar coeficiente de orientacao
		self.orientationCoef = self.oC(self.obtainedAngleToRobot, sp.sensor_cone_angle)
		
		#atualizar coeficiente de proximidade
		self.proximityCoef = self.pC(self.obtainedDistanceToRobot, sp.min_distance_to_robot, sp.sensor_cone_radius)
		
		#atualizar coeficiente de avanco
		self.forwardCoef = self.fC(self.obtainedDistanceToRobot, sp.desired_distance_to_robot, sp.sensor_cone_radius, sp.min_distance_to_robot)
		
		#atualiza o coeficiente de velocidade linear
		self.linearCoef = self.lC(self.proximityCoef, self.forwardCoef, self.interferenceCoef)
		
		#atualiza o coeficiente de velocidade angular
		self.angularCoef = self.aC(self.coefficienteForce, self.relativeDiffInRadius, self.conversionCoef, self.interferenceCoef)
		
		#atualiza a velocidade angular -- alterado em 2015.11.07
		#self.angularVelocity = self.aV(self.angularVelocity, self.angularCoef)
		
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


