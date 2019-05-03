#!/usr/bin/env python
# --------------------------------------------------#
# AUTOR: softelli@gmail.com
# class Point
# - define os atributos e metodos adicionais para cada ponto
# - define 3 pontos para identificacao do circulo
# - calcula o circulo por CRAMER e SARRUS 
# #----------------------------------------------------#

import math

class Point:
	def __init__(self):
		#inicializa as variaveis 
		self.exists = False
		self.x = 0.0
		self.y = 0.0
		self.radius = 0.0
		self.angle = 0.0
	  
	def setCoordPol(self, radius, angle):
		#recebe coordenadas polares e calcula as retangulares
		self.radius = radius
		self.angle = angle
		self.x = math.cos(math.radians(angle)) * radius
		self.y = math.sin(math.radians(angle)) * radius
		self.exists = True
	
	def setCoordRet(self, x, y):
		#recebe as coordenadas retangulares e calcula as polares
		self.x = x
		self.y = y
		#prevent float by zero division
		if(self.x == 0):
			self.x = 0.0000000001
		self.angle = math.degrees(math.atan2(self.y,self.x))
		self.radius = math.sqrt(math.pow(self.x, 2) + math.pow(self.y, 2))
		self.exists = True
		
	def getCoordPol(self):
	  return (self.radius, self.angle)
	
	def getCoordRet(self):
	  return self.x, self.y
	
	def getAllCoords(self):
	  return self
	
	def prt(self, num):
		#imprime os valores - apenas para debug
		print("[P %d] (%5.3f,%5.3f) || %5.3f < %5.2f" % (num, self.x, self.y, self.radius, self.angle)) 
		#print("[P%d]: %d (%5.3f, %5.3f) | (%5.3f < %5.2f) % (num, self.exists, self.x, self.y, self.radius, self.angle))