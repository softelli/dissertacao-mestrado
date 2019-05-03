from __future__ import division
from LocalObject import LocalObject
from polar_coord import LinAng
from pid import PID

import math
import random
import numpy as np
import log

import colorprint
cp = colorprint.colorprint()

#static parameters
from ParametersServer import staticParameters
sp = staticParameters

class circum:

	def __init__(self):

		#Raio obtido
		self.obtainedRadiusToTarget = 0.0

		#Angulo Obtido em Relacao ao Target
		self.obtainedAngleToTarget = 0.0

		#Angulo Obtido em Relacao ao Robo
		self.obtainedAngleToRobot = 0.0
		
		#Distancia obtida
		self.obtainedDistanceToRobot = 0.0		

		#Distancia obtida ao alien
		self.obtainedDistanceToAlien = 0.0
		
		#angulo obtido ao alien
		self.obtainedAngleToAlien = 0.0
		
		#nova dimensao
		self.obtainedDistanceFromTargetToRobot = 0.0
		
		#novo controle
		self.circularCoefToTarget = 0.0
		
		#novo controle
		self.propDistCoefToRobot = 0.0
		
		#novo controle
		self.linearCoefTargetToRobot = 0.0
		
		#novo controle
		self.propAngularCoefToRobot = 0.0 
		
		#novo controle
		self.linearCoefToTarget = 0.0
		
		#self
		self.proximityCoefToRobot = 0.0
		
		#self
		self.actualTime = 0.0
		
		#self
		self.previousTime = 0.0
		
		#self
		self.approachRadius = 0.0
		
		#self
		#self.vRobot = LinAng()		
		
		#localizacao x,y , com base no frame centralizado em mim
		self.target = LinAng()
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
		
		#Target detectado?
		self.hasTarget = False

		#Robo detectado?
		self.hasRobot = False

		#Alien detectado?
		self.hasAlien = False
		
		#Status
		self.status = 0
		
		#pids
		self.pidLinear = PID(3.0, 4.0, 1.2)
		self.pidLinear.setPoint(sp.desired_radius)
		
		self.pidAngular = PID(3.0, 4.0, 1.2)
		self.pidAngular.setPoint(sp.desired_angle_to_target)
		
		#mutable circumnavigation appRadiusCoef
		self.desired_radius = sp.desired_radius
		
	def setTime(self, time):
		self.previousTime = self.actualTime
		self.actualTime = time
		
	def getPropInterFromRobot(self, target_to_robot_distance): #pifR
	        #gera um trapezio
	        #utilizada para medir a crenca na circunavegacao do robo
	        #com base na distancia x entre o alvo e o robo
	        #retorna 1.0 quando x estiver entre minima distancia e raio desejado
	        #retorna proporcional quando x < minima distancia 
	        #retorna proporcional quando x > raio desejado
	        #retorna 0,0 quando a distancia for 0 ou maior que raio do sensor
	        x = target_to_robot_distance
	        a = 0.0
	        b = sp.min_distance_to_robot
	        c = sp.desired_radius
	        d = sp.sensor_cone_radius
		return max(min((x-a)/(b-a), 1.0, (d-x)/(d-c)), 0.0)
	      
	def getPropDistToRobot(self): #pdtR
	        #gera uma valor de erro proporcional a distancia desejada ao robo
	        #utilizado para incrementar as velocidades linear e angular
	        #se obtida = desejada, pde = 1.0
	        #se obtida > desejada, pde = 1.xxxx
	        #se obtida < desejada, pde = 0.9xxx
	        if(self.obtainedDistanceToRobot == sp.sensor_cone_radius):
		     return 1.0
		return (sp.sensor_cone_radius + self.obtainedDistanceToRobot)/(sp.sensor_cone_radius + sp.desired_distance_to_robot)
	      
	def getDistanceFromTargetToRobot(self): #dfTtR
		#retorna a distancia do alvo ao robo, quando houver os dois
		if((not self.hasRobot) or (not self.hasTarget)):
		       return 0.0
		else:
		       return ((self.target.x - self.robot.x)**2 + (self.target.y - self.robot.y)**2)**0.5
	      
	def getPropAngularCoefToTarget(self): #pactT
		#sensor_angle, obtained_angle, desired_angle
		#calcula o coeficiente proporcional angular em relacao do angulo ao Target
		#envolve angulo do sensor, angulo desejado e angulo obtido
		#retorna 0.0 quando angulo_obtido = angulo_desejado
		#intervalo de retorno [-1.0, 1.0] *** plotar arquivo: "proportionalAngularToTarget.py"
		
		#valor fixo
		if   self.target.angular <= sp.min_ctrl_angle:
			#print "self.target.angular <= sp.min_ctrl_angle" 
			return -1.0
		
		#apenas semantica
		elif self.target.angular >= sp.max_ctrl_angle:
			#print "self.target.angular >= sp.max_ctrl_angle" 
			return 1.0
		      
		#entao, min_ctrl_angle < self.target.angular < max_ctrl_angle
		else:
			#print "min_ctrl_angle < self.target.angular < max_ctrl_angle" 
			return 2 * (self.target.angular - sp.min_ctrl_angle) / (sp.max_ctrl_angle - sp.min_ctrl_angle) - 1
		        #2 * (a - min)/(max - min)
			
	def getPropRadialCoefToTarget(self): #prctT
		#sensor_radius, obtained_radius, desired_radius
		#calcula o valor do coeficiente com base no raio ao Target
		#envolve raio do sensor, raio desejado e raio obtido
		#retorna 0.0 quando raio_obtido = raio_desejado
		#intervalo de retorno [-1.0, 1.0] *** plotar arquivo: "proportionalRadialToTarget.py"
		
		#2017 ago 22
		#if self.target.linear >= sp.desired_radius:
		#    #y.flat[high] = (2 * (x - dR)/(sR - dR) + 1) / 2
		#    #print "self.target.linear >= sp.desired_radius"
		#    return (2 * (self.target.linear - sp.desired_radius) / (sp.sensor_cone_radius - sp.desired_radius) + 1) / 2
		#else:
		#    #print "self.target.linear < sp.desired_radius"
		#    return (self.target.linear - sp.desired_radius) / (sp.desired_radius - sp.min_distance_to_robot)
		#    #y.flat[low]  = (x - dR)/(dR - mD)
		
		#nova
		#retorna 1.0 quando raio obtido = desejado
		#retorna uma curva de tangente hiperbolica
		#assintota negativa para 0
		#assintota positiva para 2
		#https://www.wolframalpha.com/input/?i=plot(1+%2B+(1+-+e%5E(-(x-3%2F2)*k))+%2F+(1+%2B+e%5E(-(x-3%2F2)*k))),+0+%3C+x+%3C+3,++k%3D6
		d = self.obtainedRadiusToTarget - sp.desired_radius
		cp.cyan("d: " + str(d))
		k = sp.sensor_cone_radius / sp.min_distance_to_robot
		cp.cyan("k: " + str(k))
		m = math.e**(-d*k)
		cp.cyan("m: " + str(m)) 
		prctT = 1 - (1-m)/(1+m)
	        cp.cyan("prctT: " + str(prctT))
	        return prctT
	      
	#def getCircularCoefToTarget(self):
		#calcula o coeficiente para circular o target
		#quando ha o Target, retorna a media dos valores obtidos por getProportionalAngularToTarget e getProportionalRadialToTarget
		#if self.target.angular == 0.0:
			#return 0.0	
		#return (self.getPropAngularCoefToTarget() + self.getPropRadialCoefToTarget())/2.0
	      
	#def getPropAngularCoefToRobot(self):
		#desativado pelo desuso 2017 08 08
		#calcula o valor do coeficiente em relacao ao angulo ao robo (evitar colisao)
		#envolve o angulo obtido e o angulo do sensor
		#retorna valor [0.0, 1.0] *** plotar arquivo "getPropAngularCoefToRobot.py"
		
		#abs_angle_dif = abs(self.obtainedAngleToRobot)  #apenas valores positivos
		#return (1 - angle_to_robot / (sp.sensor_cone_angle / 2.0))**2
	
	def getPropAngularCoefToRobot(self): #pactR
		#utilizado para desviar do robo circunavegando
		#intervalo de retorno [-1.0, 1.0]
		#return 2 * (self.robot.angular - sp.min_ctrl_angle) / (sp.max_ctrl_angle - sp.min_ctrl_angle) - 1
		#wolfram alpha: f(x)=(1 - |x|/(3 * pi/4))^4, x = -3pi/4 to 3pi/4
		#<https://www.wolframalpha.com/input/?i=f(x)%3D(1+-+%7Cx%7C%2F(3+*+pi%2F4))%5E4,+x+%3D+-3pi%2F4+to+3pi%2F4>

		pactR = (1.0 - (abs(self.robot.angular) / (sp.sensor_cone_angle / 2.0)))**6.0
		return pactR
		 
	      
	def getInterfAngularProjToRobot(self): #iaptR
		#calcula o angulo projetado ao robo
		#utiliza o R descrito pelo robo
		#retorna 1.0, para angulo projetado = angulo detectado (interferencia maxima)
		#retorna valor proporcional < 1.0
		
		iaptR = 1.0
		if(self.angularVelocity == 0.0):
		    return 1.0
		r = self.linearVelocity / self.angularVelocity
		if(abs(r) > sp.desired_distance_to_robot/2.0):
		    fcos = (sp.desired_distance_to_robot / 2.0) / r
		    cp.cyan("cos: " + str(fcos)) 
		    f = math.acos(fcos)
		    cp.cyan("firstAngle(rad): " + str(fcos))
		    g = f * 180 / math.pi
		    cp.cyan("firstAngle(grades): " + str(g))
		    p = 90.0 - g
		    cp.cyan("projectedAngle (90 - g):" + str(p)) 
		    iaptR = abs(self.obtainedAngleToRobot - p)/sp.sensor_cone_angle 
		    cp.cyan("iaptR" + str(iaptR))
		return iaptR
		
	
#	def getPropDistCoefToRobot(self): #pdctR
#		#calcula coeficiente linear relacao a posicao do robo
#		#apenas se a distancia obtida for menor do que a desejada
#		#envolve a distancia obtida e a desejada em relacao ao robot
#		#retorna valores no intervalo [0.0,1.0] *** plotar arquivo "getPropDistCoefToRobot"
#		if(self.obtainedDistanceToRobot < sp.desired_distance_to_robot):
#			return 1 - self.obtainedDistanceToRobot / sp.desired_distance_to_robot
#		else:
#			return 0.0
		      
	def getPropDistCoefToRobot(self): #pdctR
		#calcula coeficiente linear relacao a posicao do robo
		#apenas se a distancia obtida for menor do que a desejada
		#envolve a distancia obtida e a desejada em relacao ao robot
		#retorna valores no intervalo [0.0,1.0] *** plotar arquivo "getPropDistCoefToRobot2"
		if(self.obtainedDistanceToRobot < sp.desired_distance_to_robot):
			return 1 - self.obtainedDistanceToRobot / sp.desired_distance_to_robot
		else:
			return 0.0
	
#	def getLinearCoefTargetToRobot(self, target, robot):
#		#calcula o coeficiente linear em relacao a distancia do target ao robo detectado
#		#plotar arquivo getLinearCoefTargetToRobot.py
#		#distancia do target ao robot
#		obtainedDistanceFromTargetToRobot = ((target.x - robot.x)**2 + (target.y - robot.y)**2)**0.5
#		#atualiza atributo
#		self.obtainedDistanceFromTargetToRobot = self.H(self.hasRobot and self.hasTarget, True) * obtainedDistanceFromTargetToRobot
#		
#		minObtainedDistance = sp.desired_radius - sp.min_distance_to_robot
#		maxObtainedDistance = sp.desired_radius + sp.min_distance_to_robot
#		
#		if ((minObtainedDistance < obtainedDistanceFromTargetToRobot) and (obtainedDistanceFromTargetToRobot < maxObtainedDistance )):
#		        modularDistance = ((obtainedDistanceFromTargetToRobot - sp.desired_radius)**2)**0.5
								
#			return 1 - modularDistance / sp.min_distance_to_robot
#		else:
#			return 0.0
		
		
	def getProportionalAngleToTarget(self): #patT
	        num = sp.desired_angle_to_target - self.obtainedAngleToTarget
	        den = sp.sensor_cone_angle/2
	        #wolfram alpha f(x)=(pi/2 - x)/( 3pi/4), x=-3pi/4 to 3pi/4 
		return num/den
	      
	def getProximityCoefToRobot(self): #pctR
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
		    return 1.0
		else:
		    return 0.0
		
		
		      
	def printAll(self, num_id):
	        #print "[", num_id, "]:: Ci:", self.interferenceCoef, "Cc:", self.conversionCoef, "Co", self.orientationCoef, "Cp", self.proximityCoefToRobot, "Ca", self.forwardCoef
	        print "-------- P", num_id, " ---------"
	        #print("Robot Real Velocities (vL,vA): %6.2f m/s, %6.2f rad/s " % (self.realLinearVelocity,self.realAngularVelocity))
	        #print("p3dx_mover velocities (vL,vA): %6.2f m/s, %6.2f rad/s " % (self.linearVelocity,self.angularVelocity))
	        #print("Velocidade Tang Direita    : %6.2f m/s" % (self.rightLinearVelocity))
	        #print("Rotacao Roda Direita       : %6.2f rad/s" % (self.rightWheelRotation))
	        #print("Velocidade Tang Esquerda   : %6.2f m/s" % (self.leftLinearVelocity))
	        #print("Rotacao Roda Esquerda      : %6.2f rad/s" % (self.leftWheelRotation))
		#print("Target x,y                 : %6.2f, %6.2f " % (self.target))
		#print(        "Target (Raio, Angulo): %6.2f, %6.2f " % (self.obtainedRadiusToTarget, self.obtainedAngleToTarget))
		#print("Raio do Sensor         (Sr):     %6.2f" % (sp.sensor_cone_radius))
		#print("Raio desejado          (dR):     %6.2f" % (sp.desired_radius))
		#print("Circular Coef   Target aCtB: %6.2f" % (self.circularCoefToTarget))	
		#print("Robot x,y                  : %6.2f, %6.2f " % (self.robot))
		#print("        Robot (Dist, Angulo) : %6.2f, %6.2f" % (self.obtainedDistanceToRobot, self.obtainedAngleToRobot))
		#print("Min Distancia entre Rob(mD):     %6.2f" % (sp.min_distance_to_robot))
		#print("Distancia desejada     (dD):     %6.2f" % (sp.desired_distance_to_robot))
		#print("proximityCoefToRobot       : %6.2f " % (self.proximityCoefToRobot))
		#print("propAngularCoefToRobot     : %6.2f" % (self.propAngularCoefToRobot)) 
		#print("propDistCoefToRobot        : %6.2f" % (self.propDistCoefToRobot))			
		#print("Distancia Target to Robot  : %6.2f" % (self.obtainedDistanceFromTargetToRobot))
		#print("Ang. Coef Target to Robot  : %6.2f" % (self.linearCoefTargetToRobot))
	        #print sp.status_msg[self.status], self.status
	        #print ""
	        print ""
	        
	        
	#atualiza a existencia de objetos detectados
	def updateDetectedObjects(self, detectedTargetDist, detectedRobotDist, detectedAlienDist):
	        
		if detectedTargetDist.linear > 0.0:
			self.hasTarget = True
			self.obtainedRadiusToTarget, self.obtainedAngleToTarget = detectedTargetDist.getPolarCoords()
			self.target.setRetCoords(detectedTargetDist.getRetCoords())
			self.target.setPolarCoords(detectedTargetDist.getPolarCoords())
		else:
			self.hasTarget = False
			#self.obtainedRadiusToTarget, self.obtainedAngleToTarget = 0.0, 0.0
			self.obtainedRadiusToTarget, self.obtainedAngleToTarget = sp.sensor_cone_radius, sp.sensor_cone_angle
			self.target.clear()

		if detectedRobotDist.linear > 0.0:
			self.hasRobot = True
			self.obtainedDistanceToRobot, self.obtainedAngleToRobot = detectedRobotDist.getPolarCoords()
			self.robot.setRetCoords(detectedRobotDist.getRetCoords())
			self.robot.setPolarCoords(detectedRobotDist.getPolarCoords())
		else:
			self.hasRobot = False
			#self.obtainedDistanceToRobot, self.obtainedAngleToRobot = 0.0, 0.0
			self.obtainedDistanceToRobot, self.obtainedAngleToRobot = sp.sensor_cone_radius, sp.sensor_cone_angle
			self.robot.clear()

		if detectedAlienDist.linear > 0.0:
			self.hasAlien = True
			self.obtainedDistanceToAlien, self.obtainedAngleToAlien = detectedAlienDist.getPolarCoords()
			self.alien.setRetCoords(detectedAlienDist.getRetCoords())
			self.alien.setPolarCoords(detectedAlienDist.getPolarCoords())
		else:
			self.hasAlien = False
			#self.obtainedDistanceToAlien, self.obtainedAngleToAlien = 0.0, 0.0
			self.obtainedDistanceToAlien, self.obtainedAngleToAlien = sp.sensor_cone_radius, sp.sensor_cone_angle
			self.alien.clear()
			

	      
	
	      
	#devolve as velocidades linear e angular
	def getVelocities(self, numRobot , myVelocities, targetCoord, robotCoord, alienCoord, now):
		#print "target coord", targetCoord.getVelocities()
		#print "robot coord", robotCoord.getVelocities()
		#print "alien coord", alienCoord.getVelocities()
		
		self.setTime(now.secs + now.nsecs / (10**9)) # segundos.milisegundos
		#print ("Tempo decorrido %4.9f" %(self.actualTime - self.previousTime))
		
		#atualiza a velocidade real
		self.realLinearVelocity, self.realAngularVelocity = myVelocities.getVelocities()
		
		#atualiza a existencia dos objetos
		self.updateDetectedObjects(targetCoord, robotCoord, alienCoord)

		#age conforme os objetos detectados
		
		#iaptR = self.getInterfAngularProjToRobot()
		
		#prctT = self.getPropRadialCoefToTarget()
		
		pactT = self.getPropAngularCoefToTarget() #[-1.0...1.0]
		cp.target("pactT : " + str(pactT))
		
		patT = self.getProportionalAngleToTarget() #[negativo, para angulo > 90.. positivo para angulo < 90
		cp.target("patT : " + str(patT))
		
		pacTR = self.getPropAngularCoefToRobot() #[-1.0...1.0]
		cp.robot("pacTR : " + str(pacTR))
		
		dfTtR = self.getDistanceFromTargetToRobot()
		cp.robot("dfTtR : " + str(dfTtR))
		
		pdtR = self.getPropDistToRobot() #[---, 1.0, +++]
		cp.robot("pdtR : " + str(pdtR))
		
		pifR = self.getPropInterFromRobot(dfTtR) #trapezio [1.0 quando Dmin <= x <= Dd], [0.0 quando x > rS]
		cp.robot("pifR : " + str(pifR))		
		
		pdctR = self.getPropDistCoefToRobot()
		cp.robot("pdctR : " + str(pdctR))
		
		#heaviside wandering eh 0.0 se um dos dois forem detectados
		hW = (0.0 if (self.obtainedRadiusToTarget < sp.sensor_cone_radius or self.obtainedDistanceToRobot < sp.sensor_cone_radius) else 1.0)
		cp.wander("hW: " + str(hW) + " (heaviside Wander)")
		
		vW = sp.linear_velocity
		cp.wander("vW: " + str(vW) + " (linear velocity to Wander)")
		
		rW = sp.linear_velocity / (sp.angular_velocity * (1 + (np.random.random() / 10.0)))
		cp.wander("rW: " + str(rW) + " (radius to Wander)")		
				
		hT = (1.0 if self.obtainedRadiusToTarget < sp.sensor_cone_radius else 0.0)
		cp.target("hT: " + str(hT) + " (heaviside Target)")
		
		hR = (1.0 if self.obtainedDistanceToRobot < sp.sensor_cone_radius else 0.0)
		cp.robot("hR: " + str(hR) + " (heaviside Robot)")
		
		#vT = sp.min_linear_velocity + sp.delta_linear_velocity * patT
		vT = sp.max_linear_velocity
		if(pactT == -1):
		    vT = vT * patT
		    
		cp.target("vT: " + str(vT) + " (linear velocity to Target)")
		#if pacT == -1 ... self.linearVelocity = sp.min_linear_velocity + sp.delta_linear_velocity * patT
		#             else self.linearVelocity = sp.min_linear_velocity + sp.delta_linear_velocity * bic * pde
		
		#vR = sp.min_linear_velocity + sp.delta_linear_velocity * pifR**2 * pdtR
		vR = sp.max_linear_velocity * (pdctR * pdtR)**pifR
		cp.robot("vR: " + str(vR) + " (linear velocity to Robot)")
		
		#irtT = (0.0 if pactT == -1 else self.obtainedRadiusToTarget)
		#cp.target("irtt: " + str(irtT) + " (inverse radius to Target)")
		
		#drtT = (sp.desired_radius if pactT == -1 else sp.desired_radius * (-1))
		#cp.target("drtT: " + str(drtT) + " (desired radius to Target)")
		
		rT = sp.wheel_separation / 2.0
		
		if(pactT == -1):
		    rT = rT + self.obtainedRadiusToTarget - sp.desired_radius  
		else:
		    rT = rT + sp.desired_radius
		
		#rT = drtT + sp.wheel_separation / 2.0 + irtT
		cp.target("rT: " + str(rT) + " (radius to Target)")
		#if pact == -1 ... rT = -sp.desired_radius + sp.wheel_separation / 2.0 + self.obtainedRadiusToTarget 
		#if pact != -1 ... rT =  sp.desired_radius + sp.wheel_separation / 2.0
		
		rR = self.robot.linear * (1 - pacTR**2)
		cp.robot("rR: " + str(rR) + " (radius to Robot)")		
		
		#linear = hw * wander_linear + ht * target_linear + hr * robot_linear		
		#print "linear: ", linear
		
		#radius = hw * wander_radius + ht * target_radius + hr * robot_radius
		#print "radius: ", radius
		
		#angular = 0.0 se raio == 0.0
		
		wW = 0.0 if rW == 0.0 else hW * vW/rW
		cp.wander("wW : " + str(wW) + " (angular velocity to Wander)")
		
		wT = 0.0 
		if(rT != 0.0): 
		  if(pactT == -1):
		    wT = hT * vT/rT * (-1)
		  else:
		    wT = hT * vT/rT * (1 + pactT)
		
		wT = self.getHardLimited(-1.0, wT, 1.0)
		cp.target("wT : " + str(wT) + " (angular velocity to Target)")
		#self.angularVelocity = (self.linearVelocity / self.desired_radius) * (1 + pactT)
		
		#wR = 0.0 if rR == 0.0 else hR * vR/rR
		#getHardLimited(self, min_value, value, max_value)
		wR = (0.0 if rR == 0.0 else hR * vR/rR)
		wR = self.getHardLimited(-1.0, wR, 1.0)
		
		cp.robot("wR : " + str(wR) + " (angular velocity to Robot)")
		
		#angular = wander_ang + target_ang + robot_ang
		
		#print "angular: ", angular
				
		self.linearVelocity = (vW*hW + vT*hT + vR*hR) / (hW + hT + hR)
		self.angularVelocity = (wW + wT + wR) / (hW + hT + hR)
		print "v : ", self.linearVelocity
		print "w : ", self.angularVelocity
		print "r(t-1) : ", self.obtainedRadiusToTarget
		print "r(t)   : ", self.linearVelocity / (self.angularVelocity + 0.00001)
		print "obtainedDistanceToRobot: ", self.obtainedDistanceToRobot
			
			
		if(self.hasTarget): #Circulating and not self.hasRobot
			self.status = 0
		
			#if(pact == -1.0):
			  #print "pact == -1"
			  #self.desired_radius = self.obtainedRadiusToTarget - sp.desired_radius + sp.wheel_separation/2
			  #self.linearVelocity = sp.max_linear_velocity * pa
			  #self.linearVelocity = sp.min_linear_velocity + sp.delta_linear_velocity * pa
			  #elf.angularVelocity = -(self.linearVelocity / self.desired_radius) #equivale a multiplicar pelo pact, que eh -1
			  
			#else:
			  #print "pact != -1"
			  #self.desired_radius = sp.desired_radius + sp.wheel_separation / 2.0
			  #self.linearVelocity  = sp.max_linear_velocity
			  #self.linearVelocity = sp.min_linear_velocity  + sp.delta_linear_velocity * bic * pde
			  #self.angularVelocity = (self.linearVelocity / self.desired_radius) * (1 + pact)
			
			#if(self.hasRobot):
			  #pactr = self.getPropAngularCoefToRobot()
			  #self.desired_radius = ((self.desired_radius + self.robot.linear)/2.0) * (1 - pactr)
			  #self.angularVelocity = (self.linearVelocity / self.desired_radius) * (1 + pactr)
			  
			#print "iap:", iap
			#print "pact:", pact
			#print "rconv:", rconv
			#print "propAng", propAng
			#print "linearVelocity", self.linearVelocity
			#print "angularVelocity", self.angularVelocity			
			#print "pidLinear : ", self.pidLinear.update(self.obtainedRadiusToTarget)
			#print "pidAngular: ", self.pidAngular.update(self.obtainedAngleToTarget) 
			#print "vel angular: ", self.angularVelocity
			
		#elif(self.hasTarget and self.hasRobot): #Escorting
			#self.status = 3 
			#self.linearVelocity = 1.0				
		elif(not self.hasTarget and self.hasRobot): #Avoiding
			self.status = 4
			#abordagem que diminui a velocidade linear para desviar
			#iap = self.getInterfAngularProjected()
			#self.linearVelocity = self.linearVelocity * iap
			#print "robot.angular", self.robot.angular
			#abordagem que circunavega o outro robo em rotacao inversa
			#pactr = self.getPropAngularCoefToRobot()
			#print "pactr: ", pactr			
			#self.desired_radius = ((self.desired_radius + self.robot.linear)/2.0) * (1 - pactr)
			#if(self.robot.angular > 0.0):
			    #self.desired_radius += -1
			     
			#print "desired_radius: ", self.desired_radius
			#self.linearVelocity  = sp.max_linear_velocity

			#self.angularVelocity = (self.linearVelocity / self.desired_radius) * (1 + pactr)
			#print "angularVelocity: ", self.angularVelocity
						
		else: #status == "Seeking"
		        self.status = 0
		        #self.linearVelocity = sp.linear_velocity
		        #decimal_random_rate = np.random.random() / 10.0
			#self.angularVelocity = sp.angular_velocity * (1 + decimal_random_rate)
			
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

	#def getDelimitedLinearVelocity(self, linearVelocity):
		#if(linearVelocity < 0.0):
			#return 0.0
		#elif(linearVelocity > sp.max_linear_velocity):
			#return sp.max_linear_velocity
		#else:
			#return linearVelocity
		      
	def getHardLimited(self, min_value, value, max_value):
		hlim = 0.0
		if(value < min_value):
			hlim = min_value
		elif(value > max_value):
			hlim = max_value
		else:
			hlim = value
		return hlim
		      
	#def verifyMinDistances(self, numRobot, velocity):
		#stop = False
		#prevents target collision
		#if(0 < self.obtainedRadiusToTarget) and (self.obtainedRadiusToTarget < sp.min_distance_to_robot):
			#if((self.obtainedAngleToTarget**2)**0.5 < 45):
				#stop = True
		#elif(0 < self.obtainedDistanceToRobot) and (self.obtainedDistanceToRobot < sp.min_distance_to_robot):
			#if((self.obtainedAngleToRobot**2)**0.5 < 45):
				#stop = True
		#if stop == True:
		    #print "XXXXXXX Stopping Linear Velocity in P", numRobot, "XXXXXXX"
		    #velocity = 0.0
		#return velocity
	      
	#def getTransitionCoefs(self):
		#utiliza o raio obter a transicao linear 
		
		#circRadiusCoef = 1.0
		#appRadiusCoef = 0.0
		#if self.target.linear >= sp.desired_radius:
		#    appRadiusCoef = (self.target.linear - sp.desired_radius)/(sp.sensor_cone_radius - sp.desired_radius)
		#    appRadiusCoef = (appRadiusCoef**2)**0.5
		#    circRadiusCoef = 1 - appRadiusCoef
		
		#coeficiente do raio de aproximacao obtido de uma sigmoidal centrada no raio desejado
		#appRadiusCoef = self.getBotzmann(self.target.linear, sp.desired_radius, sp.boltzmann_time_constant)
		#circRadiusCoef = 1 - appRadiusCoef
		
		#return circRadiusCoef,appRadiusCoef
	      
	#def getBotzmann(self, linear_x, middle_x, const_time):
		#calcula a funcao botzman com ponto central e tempo constante sobre x
		#return 1/(1 + np.exp(-(linear_x - middle_x)/const_time))
