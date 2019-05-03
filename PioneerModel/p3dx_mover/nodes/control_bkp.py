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
from LocalObject import LocalObject

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#from Robot import Robot


#control parameters

MAX_READS = 270
RADIO_BEACON = 0.10 # 0.9 ... 0.11
RADIO_ROBOT = 0.15 # 0.
RADIO_TOL = 0.30
ID_TYPES = ["unknown", "beacon", "robot", "wall"]

		
def getListaObj(data):
	obj = LocalObject(ID_TYPES)
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
					obj.identify(RADIO_ROBOT, RADIO_BEACON, RADIO_TOL)
					
					#insere uma copia profunda e desvinculada desse objeto
					listaObjetos.append(copy.deepcopy(obj)) 
					
					#reinicia o objeto para reuso
					obj.clear()
		
		p = p + 1
	
	return listaObjetos[:] #retorna uma copia da lista

def callback(data):
	global robot, beacon
	b = 0
	r = 0
	tmp = 0
	lista = getListaObj(data)
	
	#identifica o robot mais proximo
	for obj in lista:
		if(obj.isRobot()):
			if(obj.center.radius <= robot.center.radius):
				robot = obj
				r += 1
		
		if(obj.isBeacon()):
			beacon = obj
			b += 1
		 
	if(b < 1): #nao ha beacon
		beacon.clear()
		#print "No beacon."
	else:
		tmp = 1
		print "Beacon at :", beacon.center.angle, " degrees"
		
	if(r < 1): #nao ha robot
		robot.clear()
		#print "No robot."
	else:
		print "More next robot at :", robot.center.angle, " degrees"
	
	
def getTwist(data):
	global twist
	twist = data

def getStatus():
	global beacon, robot

	if(beacon.center.exists): #se existe o beacon
		if(robot.center.exists): #se existe o robot
			return 3 #circunavegacao e evitacao de obstaculos
		else:
			return 2 #apenas circunavegacao
		
	else:
		if(robot.center.exists): #se existe o robot
			return 1 #evitacao de obstaculos
		      
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
MIN_LINEAR_VEL = 0.1
ANGULAR_VEL = 0.5
MIN_ANGULAR_VEL = 0.05
MAX_ANGULAR_VEL = 0.3
ANG_INCR = 0.001
ANG_TOL = 5/100.0
LIN_INCR = 0.001

robot = LocalObject(ID_TYPES)
beacon = LocalObject(ID_TYPES)
wall = LocalObject(ID_TYPES)

def main(argv):
	#ordem certa?
	p1_pub = rospy.Publisher(sys.argv[1] + 'cmd_vel', Twist, queue_size = 1)
	rospy.init_node('p3dx_mover')
	#p1_twist = rospy.Subscriber(sys.argv[1] + "cmd_vel", Twist, getTwist)
	p1_laser = rospy.Subscriber(sys.argv[1] + "p3dx/laser/scan", LaserScan, callback)
	print "Running for robot ", sys.argv[1], " -  Please, wait ..."
	prev,status = -1,0
	
	while(1):

		status = getStatus()
		#print "status:" , status
		prevs= printStatusChange(prev,status)
		twist.linear.x = MIN_LINEAR_VEL
		if status == 0:
			twist.angular.z = MIN_ANGULAR_VEL
			twist.linear.x = LINEAR_VEL
			
		elif status == 1:
			print "Status: avoiding collision"
						
		if status == 2:
			print "Status: circunavegating"
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
	twist.angular.z = 0.0
	twist.linear.x = 0.0
	p1_pub.publish(twist)
	
	exit()
	
if __name__ == "__main__":
   main(sys.argv[1:])