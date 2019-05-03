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
from staticParameters import staticParameter
sp = staticParameter()

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

#objects
robot = LocalObject(ID_TYPES)
beacon = LocalObject(ID_TYPES)
alien = LocalObject(ID_TYPES)

 


		
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
	#print twist

def getStatus():
	global beacon, robot

	if(beacon.center.exists): #se existe o beaco ...
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

twist = Twist()

def main(argv):
  
	global robot, beacon, alien, twist
	#ordem certa?
	p1_pub = rospy.Publisher(sys.argv[1] + 'cmd_vel', Twist, queue_size = 3)
	rospy.init_node('p3dx_mover')
	p1_twist = rospy.Subscriber(sys.argv[1] + "cmd_vel", Twist, getTwist)
	p1_laser = rospy.Subscriber(sys.argv[1] + "p3dx/laser/scan", LaserScan, callback)
	print "Running for robot ", sys.argv[1], " -  Please, wait ..."
	prev,status = 0,0
	r = rospy.Rate(3) #original 10hz
	
	
	
	#simplified linear and angular data to objects
	rob = LinAng()
	bea = LinAng()
	ali = LinAng()
	vel = LinAng()
	
	while not rospy.is_shutdown():
	  		
		#prev = statusChange(prev,status)
		#status = getStatus()
		#twist.linear.x = MIN_LINEAR_VEL
		
		#se houver o robo no cone do sensor, pega valores diferentes de 0.0
		rob.linear, rob.angular = robot.center.getCoordPol()
		#rob.prn()
		
		#se houver beacon no cone do sensor, pega valores diferentes de 0.0
		bea.linear, bea.angular = beacon.center.getCoordPol()
		bea.prn()
		
		#se houver alien no cone do sensor, pega valores diferentes de 0.0
		ali.linear, ali.angular = alien.center.getCoordPol()
		#ali.prn()
		
		#obtem a velocidade atual
		vel.linear = twist.linear.x
		vel.angular = twist.angular.z
		vel.prn()		
		
		
		#processa a circunavegacao
		twist.linear.x, twist.angular.z = circ.process(vel, bea, rob, ali).getVelocities()
		twist.linear.x = 0.25
		
		#envie ao robo os valores das velocidades
		p1_pub.publish(twist)
		
		r.sleep();

	print "ending control.py..."	
	twist.angular.z = 0.0
	twist.linear.x = 0.0
	p1_pub.publish(twist)
	
	exit()
	
if __name__ == "__main__":
   main(sys.argv[1:])