#!/usr/bin/env python
# --------------------------------------------------#
# class LocalObject --- Descreve e identifica Objetos locais detectados pelo Laser
# - define os atributos e metodos adicionais para controle do robo
# - define 3 pontos para identificacao do circulo
# - calcula o circulo por CRAMER e SARRUS 
# - identifica os objetos com base na lista passada na inicilizacao e no raio
# #----------------------------------------------------#


import math
from Point import Point
from fuzzyclass import *
from ParametersServer import staticParameters

class LocalObject:
	def __init__(self, object_types):
		self.p1 = Point()
		self.p2 = Point()
		self.p3 = Point()
		self.center = Point()
		self.typ = ""
		self.radius = 0.0
		self.object_types = object_types
	  
	def clear(self):
		self.p1 = Point()
		self.p2 = Point()
		self.p3 = Point()
		self.center = Point()
		self.typ = ""
		self.radius = 0.0

	def getDist(self, pA, pB):
		return math.sqrt(math.pow(pB.x - pA.x,2) + math.pow(pB.y - pA.y,2))
		
	def getSarrus(self, matriz, cols):
		#Entradas
		  #matriz: contem as linhas e colunas do sistema a ser utilizado no calculo
		  #cols: contem a identificacao das colunas para o calculo
		#Saida
		  #determinante calculado por SARRUS
		
		D = cols[0]
		E = cols[1]
		F = cols[2]
		
		m = matriz #alias
		
		#SARRUS
		
		det =       m[0][D] * m[1][E] * m[2][F]
		det = det + m[1][D] * m[2][E] * m[0][F]
		det = det + m[2][D] * m[0][E] * m[1][F]
		det = det - m[0][F] * m[1][E] * m[2][D]
		det = det - m[1][F] * m[2][E] * m[0][D]
		det = det - m[2][F] * m[0][E] * m[1][D]
		
		return det

	def identify(self, target_radius, robot_radius):	
	#def identify(self, target_radius, robot_radius, radius_tol):
		#Entradas
		  #target_radius: raio do target
		  #robot_radius: raio do robo
		  
		  #radius_tol: tolerancia percentual de variacao dos raios
		  #object_types: lista de objetos possiveis de identificacao
		  
		#Saida
		  #faz alteracao nos atributos dessa classe 
		  
		#Nova identificacao por conjunto fuzzy: 2015/12/10
		
		fObj = fuzzyObject()
		sp = staticParameters

			
		# montar a matriz 4x3
		
		eq1 = [self.p1.x, self.p1.y, 1, math.pow(self.p1.x,2) + math.pow(self.p1.y,2)]
		eq2 = [self.p2.x, self.p2.y, 1, math.pow(self.p2.x,2) + math.pow(self.p2.y,2)]
		eq3 = [self.p3.x, self.p3.y, 1, math.pow(self.p3.x,2) + math.pow(self.p3.y,2)]
		
		m = [eq1, eq2, eq3] #matriz

		# calcular determinantes (por SARRUS)
		det = self.getSarrus(m, [0,1,2]) # (valores x, y, 1)
		
		# (resolucao por CRAMER) http://mathforum.org/library/drmath/view/55239.html
		cx = self.getSarrus(m, [3,1,2]) / (2 * det)
		cy = self.getSarrus(m, [0,3,2]) / (2 * det)
		
		self.center.setCoordRet(cx,cy)
					
		#F = getSarrus(m, [0,1,3]) / (2 * det) # nao foi necessario
		
		self.radius = math.sqrt(math.pow(self.p1.x - self.center.x,2) + math.pow(self.p1.y - self.center.y,2))

		typ = sp.id_types[0]
		if sp.target_radius_id * 0.85 <= self.radius <= sp.target_radius_id * 1.15:
			typ = sp.id_types[1]
		elif sp.robot_radius_id * 0.85 <= self.radius <= sp.robot_radius_id * 1.15:
			typ = sp.id_types[2]

		self.typ = typ
		#typ2 = fObj.rank(self.radius, 'radius', sp.id_types,  sp.target_radius_id, sp.robot_radius_id, sp.sensor_cone_radius)
		
		#if typ2 == sp.id_types[2] and typ == sp.id_types[1]:
		#	self.typ = typ
		#else:
		#	self.typ = typ2
		   
		
	def isRobot(self):
		if(self.typ == "robot"):
			#print "It's robot!"
			return True
		else:
			return False
	     
	def isTarget(self):
		if(self.typ == "target"):
			#print "It's target!"
			return True
		else:
			return False
	     
	def isAlien(self):
		if(self.typ == "alien"):
			#print "It's alien"
			return True
		else:
			return False
		      
	def prt(self):
		#impressao para debug
		self.center.prt(0)
		self.p1.prt(1)
		self.p2.prt(2)
		self.p3.prt(3)

		print "typ = ", self.typ, ("radius = %4.3f" % (self.radius))
