#!/usr/bin/env python

# --------------------------------------------------#
# - obter a leitura do laser
# - analisar e identificar os objetos no alcance
# - - > obter os 3 pontos mais significativos (inicio, mais profundo e fim)
# - - > 
# - - > discriminar os objetos encontrados (r = robot, b = beacon, o = obstaculo)
# - calcular qual a proxima direcao e coordenada
# - enviar comando para as rodas do robot
#----------------------------------------------------#

import roslib; roslib.load_manifest('p3dx_mover')
import rospy
import math
import copy
import sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#control parameters

MAX_READS = 270
RADIO_BEACON = 0.10 # 0.9 ... 0.11
RADIO_ROBOT = 0.15 # 0.
RADIO_TOL = 0.30
ID_TYPES = ["unknown", "beacon", "robot", "wall"]

class Point:
	def __init__(this):
		#inicializa as variaveis do ponto
		this.exists = False
		this.x = 0.0
		this.y = 0.0
		this.radius = 0.0
		this.angle = 0.0
	  
	def setCoordPol(this, radius, angle):
		#recebe coordenadas polares e calcula as retangulares
		this.radius = radius
		this.angle = angle
		this.x = math.cos(math.radians(angle)) * radius
		this.y = math.sin(math.radians(angle)) * radius
		this.exists = True
	
	def setCoordRet(this, x, y):
		#recebe as coordenadas retangulares e calcula as polares
		this.x = x
		this.y = y
		#prevent float by zero division
		if(this.x == 0):
			this.x = 0.0000000001
		this.angle = math.degrees(math.atan(this.y / this.x))
		this.radius = math.sqrt(math.pow(this.x, 2) + math.pow(this.y, 2))
		this.exists = True
	
	def prt(this, num):
		#imprime os valores
		#print "[P %d] (%5.3f,%5.3f) || %5.3f < %5.3f", num, this.x, this.y, this.radius, this.angle 
		print "[P", num, "]: ", this.exists, " (", this.x, ",", this.y,") | ", this.radius, " < ", this.angle 

class TargetObject:
	def __init__(this):
		this.p1 = Point()
		this.p2 = Point()
		this.p3 = Point()
		this.center = Point()
		this.dist = {'p1to2': 0.0, 'p1to3': 0.0, 'p2to3': 0.0}
		this.typ = "released"
		this.radius = 0.0
	  
	def clear(this):
		this.__init__()


	def getDist(this, pA, pB):
		return math.sqrt(math.pow(pB.x - pA.x,2) + math.pow(pB.y - pA.y,2))
		
	def calcPointsDist(this):
		this.dist['p1to2'] = this.getDist(this.p1, this.p2)
		this.dist['p2to3'] = this.getDist(this.p2, this.p3)
		this.dist['p1to3'] = this.getDist(this.p1, this.p3)
		
	def getSarrus(this, matriz, cols):
		
		D = cols[0]
		E = cols[1]
		F = cols[2]
		
		m = matriz
		
		#print "D", D, "E", E, "F", F 
		
		#SARRUS
		
		det =       m[0][D] * m[1][E] * m[2][F]
		det = det + m[1][D] * m[2][E] * m[0][F]
		det = det + m[2][D] * m[0][E] * m[1][F]
		det = det - m[0][F] * m[1][E] * m[2][D]
		det = det - m[1][F] * m[2][E] * m[0][D]
		det = det - m[2][F] * m[0][E] * m[1][D]
		
		return det
	
	def identify(this):
		#x^2 + y^2 + D x + E y + F = 0, 
		# portanto xD + yE + F = -(x^2 + y^2)
		
		# montar a matriz 4x3
		
		eq1 = [this.p1.x, this.p1.y, 1, math.pow(this.p1.x,2) + math.pow(this.p1.y,2)]
		eq2 = [this.p2.x, this.p2.y, 1, math.pow(this.p2.x,2) + math.pow(this.p2.y,2)]
		eq3 = [this.p3.x, this.p3.y, 1, math.pow(this.p3.x,2) + math.pow(this.p3.y,2)]
		
		m = [eq1, eq2, eq3] #matriz

		# calcular determinantes (por SARRUS)
		det = this.getSarrus(m, [0,1,2]) # (valores x, y, 1)
		
		# (resolucao por CRAMER) http://mathforum.org/library/drmath/view/55239.html
		cx = this.getSarrus(m, [3,1,2]) / (2 * det)
		cy = this.getSarrus(m, [0,3,2]) / (2 * det)
		
		this.center.setCoordRet(cx,cy)
		
		#F = getSarrus(m, [0,1,3]) / (2 * det) # nao foi necessario
		
		this.radius = math.sqrt(math.pow(this.p1.x - this.center.x,2) + math.pow(this.p1.y - this.center.y,2))
				
		#identificando
		max_robot_radius = RADIO_ROBOT * (1 + RADIO_TOL)
		
		min_robot_radius = RADIO_ROBOT * (1 - RADIO_TOL)
		
		max_beacon_radius = RADIO_BEACON * (1 + RADIO_TOL)
		
		min_beacon_radius = RADIO_BEACON * (1 - RADIO_TOL)
				
		this.typ = ID_TYPES[0] #unknown, por default
		
		if(this.radius > max_robot_radius or this.center.radius <= this.p2.radius):
			#print "this.radius > max_robot_radius or this.center.radius <= this.p2.radius"
			#o raio eh maior do que o esperado ou menor do que o p2 (convexo)
			this.typ = ID_TYPES[3] #wall
		else:
			#o arco eh concavo e o raio esta dentro do limit para ser um robot ou beacon
			#print "this.radius <= max_robot_radius and this.center.radius > this.p2.radius"
			if(this.radius >= min_robot_radius):
				#se for maior que o raio minimo do robo
				#print "this.radius >= min_robot_radius"
				this.typ = ID_TYPES[2] #robot
			else:
				#eh menor que o minimo do robo
				#print "this.radius < min_robot_radius"
				if(this.radius < max_beacon_radius and this.radius >= min_beacon_radius):
					#print "this.radius < max_beacon_radius and this.radius >= min_beacon_radius"
					#raio esta dentro dos limits para ser um beacon
					this.typ = ID_TYPES[1]
		
		
	
	def prt(this):
		this.center.prt(0)
		this.p1.prt(1)
		this.p2.prt(2)
		this.p3.prt(3)

		print "typ = ", this.typ
		print "radius = ", this.radius
		
def getListaObj(data):
	obj = TargetObject()
	limit = data.range_max * (1 - 2 * RADIO_ROBOT)
	
	listaObjetos = []
	p = 0
	while(p < MAX_READS):
		#index = p - MAX_READS/2 #substituido para melhorar a precisao
		index = math.degrees(data.angle_min + p * data.angle_increment)
		if(data.ranges[p] < limit):
			#print "index:", index, "data.range", data.ranges[p]
			
			if(obj.p1.exists == False): #primeiro ainda nao encontrado
				obj.p1.setCoordPol(data.ranges[p], index)
				#print obj.p1.prt(1)
				p = p + 1
				continue
			if(obj.p2.exists == False): #o segundo ainda nao foi encontrado
				obj.p2.setCoordPol(data.ranges[p], index)
				p = p + 1
				#obj.p2.prt(2)
				continue
			else:
				if(obj.p2.radius > data.ranges[p]): #o segundo ja foi encontrado, mas este eh menor
					obj.p2.setCoordPol(data.ranges[p], index)
					p = p + 1
					#obj.p2.prt(2)
					continue
			if(obj.p3.exists == False): # o terceiro ponto ainda nao foi encontrado
				if((p + 1 < MAX_READS) and (data.ranges[p + 1] < limit)): 
					#se nao for o ultimo do indice
					# e o proximo valor for valido
					p = p + 1
					continue
				else: # eh o ultimo indice ou o proximo eh invalido
					obj.p3.setCoordPol(data.ranges[p], index)
					#obj.p3.prt(3)
					
					#calcula a circunferencia e identifica o objeto
					obj.identify()
					
					#insere uma copia profunda e desvinculada desse objeto
					listaObjetos.append(copy.deepcopy(obj)) 
					
					#reinicia o objeto para reuso
					obj.clear()
		
		p = p + 1
	
	return listaObjetos[:] #retorna uma copia da lista
      
def isRobot(objeto):
	if(objeto.typ == "robot"):
		print "It's robot!"
		return True
	else:
		return False
	     
def isBeacon(objeto):
	if(objeto.typ == "beacon"):
		#print "It's beacon!"
		return True
	else:
		return False
	     
def isWall(objeto):
	if(objeto.typ == "wall"):
		print "It's wall!"
		return True
	else:
		return False

def callback(data):
	global robot, beacon
	b = 0
	r = 0
	tmp = 0
	lista = getListaObj(data)
	
	#identifica o robot mais proximo
	for obj in lista:
		if(isRobot(obj)):
			if(obj.center.radius <= robot.center.radius):
				robot = obj
				r += 1
		
		if(isBeacon(obj)):
			beacon = obj
			b += 1
		 
	if(b < 1): #nao ha beacon
		beacon.clear()
		#print "No beacon."
	else:
		tmp = 1
		#print "Beacon at :", beacon.center.angle, " degrees"
		
	if(r < 1): #nao ha robot
		robot.clear()
		#print "No robot."
	else:
		print "More next robot at :", robot.center.angle, " degrees"
	
	
def getTwist(data):
	global twist
	twist = data

def getStatus():
	if(beacon.center.exists): #se existe o beacon
		if(robot.center.exists): #se existe o robot
			return 3 #circunavegacao e evitacao de obstaculos
		else:
			return 2 #apenas circunavegacao
			
	else:
		if(robot.center.exists): #se existe o robot
			return 1
	return 0 #apenas vagando

def printStatusChange(status, prev):
	if status <> prev:
		if status == 0:
			print "Status: wandering"
		elif status == 1:
			print "Status: avoiding collision"
		elif status == 2:
			print "Status: circunavegating"
		elif status == 3: 
			print "Status: circunavegating and avoiding collision"
	return status

twist = Twist()

LINEAR_VEL = 1.0
MIN_LINEAR_VEL = 0.5
ANGULAR_VEL = 0.5
MIN_ANGULAR_VEL = 0.05
MAX_ANGULAR_VEL = 0.3
ANG_INCR = 0.001
ANG_TOL = 5/100.0
LIN_INCR = 0.001

robot = TargetObject()
beacon = TargetObject()

def main():
	#ordem certa?
	ns = rospy.get_namespace()
	p1_pub = rospy.Publisher(ns + 'cmd_vel', Twist, queue_size = 3)
	rospy.init_node('p3dx_mover')
	#p1_twist = rospy.Subscriber('pionner2/cmd_vel", Twist, getTwist)
	p1_laser = rospy.Subscriber( ns + "p3dx/laser/scan", LaserScan, callback)
	#print "Running for robot ", sys.argv[1], " -  Please, wait ..."
	prev,status = -1,0
	#if 'p1' in ns:
	#	return
	  
	while(1):

		status = getStatus()
		#print "status:" , status
		prev = printStatusChange(prev,status)
		twist.linear.x = MIN_LINEAR_VEL
		#if status == 0:
			#twist.angular.z = MIN_ANGULAR_VEL
			#twist.linear.x = LINEAR_VEL
			
		#elif status == 1:
		#	print "Status: avoiding collision"
		if status == 2:
			#print "Status: circunavegating"
			if(beacon.center.angle < 90):
				twist.angular.z = -ANGULAR_VEL
			elif(beacon.center.angle > 90):
				twist.angular.z = ANGULAR_VEL
			else:
				twist.angular.z = 0.0
			#
			#if(beacon.center.radius > 1):
			#	twist.linear.z = -LINEAR_VEL
			#elif(beacon.center.radius < 1):
			#	twist.linear.z = LINEAR_VEL
			#else:
			#	twist.linear.z = 0.0
		#elif status == 3: 
		#	print "Status: circunavegating and avoiding collision"
				
		p1_pub.publish(twist)
	exit()
	
if __name__ == "__main__":
   main()