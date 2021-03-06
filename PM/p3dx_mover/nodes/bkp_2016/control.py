#!/usr/bin/env python

# --------------------------------------------------#
# 
#----------------------------------------------------#

import roslib; roslib.load_manifest('p3dx_mover')
import rospy
import math
import copy
import sys
from LocalObject import LocalObject
from polar_coord import LinAng

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from Circumnavigation import circum
circ = circum()

#static parameters
from ParametersServer import staticParameters
sp = staticParameters

#control parameters

MAX_READS = sp.max_laser_reads
RADIO_BEACON = sp.beacon_radius_id
RADIO_ROBOT = sp.robot_radius_id
RADIO_TOL = sp.radius_tolerance
ID_TYPES = sp.id_types
LINEAR_VEL = sp.linear_velocity
MIN_LINEAR_VEL = sp.min_linear_velocity
ANGULAR_VEL = sp.angular_velocity
MIN_ANGULAR_VEL = sp.min_angular_velocity
MAX_ANGULAR_VEL = sp.max_angular_velocity

#global objects
robot = LocalObject(ID_TYPES)
beacon = LocalObject(ID_TYPES)
alien = LocalObject(ID_TYPES)
twist = Twist()
 


		
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
					obj.identify(RADIO_BEACON, RADIO_ROBOT, RADIO_TOL)
					
					#insere uma copia profunda e desvinculada desse objeto
					listaObjetos.append(copy.deepcopy(obj)) 
					
					#reinicia o objeto para reuso
					obj.clear()
		
		p = p + 1
	
	return listaObjetos[:] #retorna uma copia da lista

def callback(data):
	global robot, beacon, alien
	a = 0
	b = 0
	r = 0
	tmp = 0
	lista = getListaObj(data)
	#print len(lista), ":: ------ Objetos encontrados na lista -------"
	
	#identificando o robot mais proximo
	for obj in lista:
		#para debug, imprime os objetos
		#obj.prt()
	        
		if(obj.isRobot()):
			#print "obj.center.radius", obj.center.radius
			#print "robot.center.radius", robot.center.radius
			#se nenhum robo foi detectado ainda
			if r == 0:
				#esse serah o atual
				robot = obj
				r = 1
				#print "* first obj robot"
			
			#senao, jah existe robot detectado
			else:
			        #se estiver mais proximo que o atual
				if(obj.center.radius < robot.center.radius):
					#esse eh o atual
					robot = obj
					r += 1
					#print "+1 obj robot"
					#robot.prt()
					
		
		elif(obj.isBeacon()):
			beacon = obj
			b += 1
			#print "+1 obj beacon"
			#beacon.prt()
			
			
		#elif(obj.isAlien()):
			#if(obj.center.radius <= alien.center.radius):
			#	alien = obj
			#	print "+1 obj alien"
			#	alien.prt()
			#	a += 1
		#else:
			#print "unknow object"
		 
	if(b < 1): #nao ha beacon
		beacon.clear()
		#print "No beacon."
	#else:
		#tmp = 1
		#print "Beacon at :", beacon.center.angle, " degrees"
		
	if(r < 1): #nao ha robot
		robot.clear()
		#print "No robot."
	#else:
		#print "Closer robot at :", robot.center.angle, " degrees"
		
	if(a < 1): #nao ha alien
		alien.clear()
		#print "No alien"
		
	#print("b[%d] r[%d] a[%d]" % (b, r, a))
	
def getTwist(data):
	global twist
	twist = data
	#print twist

def getStatus():
	global beacon, robot

	if(beacon.center.exists): #se existe o beacon ...
		if(robot.center.exists): #se existe o robot
			return 3 #circunavegacao e evitacao de obstaculos
		else:
			return 2 #apenas circunavegacao
		
	else:
		if(robot.center.exists): #se existe o robot
			return 1 #evitacao de obstaculos
		      
	return 0 #apenas vagando

def statusChange(status, prev):
	if status <> prev:
		print sp.status_msg[status]
	return status



def main(argv):
  
	global robot, beacon, alien, twist
	#ordem certa?
	p1_pub = rospy.Publisher(sys.argv[1] + 'cmd_vel', Twist, queue_size = 1)
	rospy.init_node('p3dx_mover')
	p1_twist = rospy.Subscriber(sys.argv[1] + "cmd_vel", Twist, getTwist)
	p1_laser = rospy.Subscriber(sys.argv[1] + "p3dx/laser/scan", LaserScan, callback)
	print "Running for robot ", sys.argv[1], " -  Please, wait ..."
	prev,status = 0,0
	rate = rospy.Rate(3) #original 10hz
	
	
	
	#instancias de representacoes locais simplificadas de coordenadas
	rob = LinAng()
	bea = LinAng()
	ali = LinAng()
	vel = LinAng()
	debugTest = ""
	
	while not rospy.is_shutdown():
	  		
		#prev = statusChange(prev,status)
		#status = getStatus()
		#twist.linear.x = MIN_LINEAR_VEL
		
		print ""
		print "=" * 40
		print ""
		      
		#se houver beacon no cone do sensor, pega valores diferentes de 0.0
		if beacon.isBeacon():
		      bea.linear, bea.angular = beacon.center.getCoordPol()
		      debugTest = "| Beacon |"
		      #bea.prn()
		else:
		      bea.linear = 0.0
		      bea.angular = 0.0
		      debugTest = "|        |"
		      
		#se houver o robo no cone do sensor, pega valores diferentes de 0.0
		if robot.isRobot():
		      rob.linear, rob.angular = robot.center.getCoordPol()
		      debugTest = debugTest + "| Robot |"
		      #rob.prn()
		else:
		      rob.linear = 0.0
		      rob.angular = 0.0
		      debugTest = debugTest + "|       |"
		      
		#se houver alien no cone do sensor, pega valores diferentes de 0.0
		if alien.isAlien():
		      ali.linear, ali.angular = alien.center.getCoordPol()
		      debugTest = debugTest + "| Alien |"
		      #ali.prn()
		else:
		      ali.linear = 0.0;
		      ali.angular = 0.0;
		      debugTest = debugTest + "|       |"
		
		#obtem a velocidade atual
		vel.linear = twist.linear.x
		vel.angular = twist.angular.z
		#vel.prn()
		print debugTest
		
		
		#processa a circunavegacao
		twist.linear.x, twist.angular.z = circ.process(vel, bea, rob, ali).getVelocities()
		#twist.linear.x = 0.25
		
		#envie ao robo os valores das velocidades
		p1_pub.publish(twist)
		
		rate.sleep();

	print "ending control.py..."	
	twist.angular.z = 0.0
	twist.linear.x = 0.0
	p1_pub.publish(twist)
	
	exit()
	
if __name__ == "__main__":
   main(sys.argv[1:])