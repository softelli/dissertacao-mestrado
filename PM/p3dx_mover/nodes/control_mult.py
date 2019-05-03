#!/usr/bin/env python

# --------------------------------------------------#
# 
#----------------------------------------------------#

import roslib; roslib.load_manifest('p3dx_mover')
import rospy
import math
from copy import deepcopy
import sys
from LocalObject import LocalObject
from polar_coord import LinAng
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from numpy import *

from Circumnavigation import circum


#static parameters
from ParametersServer import staticParameters
sp = staticParameters

#control parameters

MAX_READS = sp.max_laser_reads
#RADIO_BEACON = sp.target_radius_id
RADIO_BEACON = sp.target_radius_id
RADIO_ROBOT = sp.robot_radius_id
#RADIO_TOL = sp.radius_tolerance
ID_TYPES = sp.id_types
LINEAR_VEL = sp.linear_velocity
MIN_LINEAR_VEL = sp.min_linear_velocity
ANGULAR_VEL = sp.angular_velocity
MIN_ANGULAR_VEL = sp.min_angular_velocity
MAX_ANGULAR_VEL = sp.max_angular_velocity

#global objects  --- 2015.10.27 --- switched to main objects
robot = LocalObject(ID_TYPES)
target = LocalObject(ID_TYPES)
alien = LocalObject(ID_TYPES)
twist = Twist()
 


		
def getListaObj(data):
	obj = LocalObject(ID_TYPES)
	limit = data.range_max * (1 - 1.5 * RADIO_ROBOT)
	
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
					obj.identify(RADIO_BEACON, RADIO_ROBOT) #, RADIO_TOL)
					
					#insere uma copia profunda e desvinculada desse objeto
					listaObjetos.append(deepcopy(obj)) 
					
					#reinicia o objeto para reuso
					obj.clear()
		
		p = p + 1
	
	return listaObjetos[:] #retorna uma copia da lista

def getLaser(data,nRob):
	#print "getLaser for nRob=", nRob
	global robots, targets, aliens
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
				robots[nRob] = obj
				r = 1
				#print "* first obj robot"
			
			#senao, jah existe robot detectado
			else:
			        #se estiver mais proximo que o atual
				if(obj.center.radius < robots[nRob].center.radius):
					#esse eh o atual
					robots[nRob] = obj
					r += 1
					#print "+1 obj robot"
					#robot.prt()
					
		
		elif(obj.isTarget()):
			targets[nRob] = obj
			b += 1
			#print "+1 obj target"
			#target.prt()
			
			
		elif(obj.isAlien()):
			if(obj.center.radius <= aliens[nRob].center.radius):
				aliens[nRob] = obj
			#	print "+1 obj alien"
			#	alien.prt()
			#	a += 1
		#else:
			#print "unknow object"
		 
	if(b < 1): #nao ha target
		targets[nRob].clear()
		#print "No target."
	#else:
		#tmp = 1
		#print "Target at :", target.center.angle, " degrees"
		
	if(r < 1): #nao ha robot
		robots[nRob].clear()
		#print "No robot."
	#else:
		#print "Closer robot at :", robot.center.angle, " degrees"
		
	if(a < 1): #nao ha alien
		aliens[nRob].clear()
		#print "No alien"
		
	#print("b[%d] r[%d] a[%d]" % (b, r, a))
	
def getRealTwist(odom,nRob):
	global realTwists
	
	realTwists[nRob] = odom.twist.twist
	#print "twist for nRob = ", nRob, realTwists[nRob]  

def getStatus():
	global target, robot

	if(target.center.exists): #se existe o target ...
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

velocity_publishers = []
odom_twist_subscribers = []
laser_subscribers = []
	
robots = []
targets = []
aliens = []
ctrlTwists = [] #valores-objetivo que sao passados ao robo
realTwists = [] #valores obtidos do robo
	
robots_coords = []
targets_coords = []
aliens_coords = []
	
velocities = []
debug_msgs = []
ctrlCircum = []

def main(argv):
  
	#global robot, target, alien, twist

	quantRobots = int(sys.argv[1])
	
	rospy.init_node('p3dx_mover')
	
	#define a frequencia de atualizacao
	rate = rospy.Rate(50) #original 10hz
	#referente ao Target
	p0_radius_to_target = rospy.Publisher("p0/radius_to_target", Float32, queue_size = 1)
	sp_desired_radius_to_target = rospy.Publisher("sp/desired_radius_to_target", Float32, queue_size = 1)
	p0_angle_to_target = rospy.Publisher("p0/angle_to_target", Float32, queue_size = 1)
	sp_desired_angle_to_target = rospy.Publisher("sp/desired_angle_to_target", Float32, queue_size = 1)
	p0_approach_radius_to_target = rospy.Publisher("p0/approach_radius_to_target", Float32, queue_size = 1)
	p0_performed_radius = rospy.Publisher("p0/performed_radius", Float32, queue_size = 1)
	
	#referente ao Robo (estaticos)
	sp_minimal_distance = rospy.Publisher("sp/minimal_distance", Float32, queue_size = 1)
	sp_desired_distance = rospy.Publisher("sp/desired_distance", Float32, queue_size = 1)
	#dinamicos
	p0_robot_distance = rospy.Publisher("p0/robot_distance", Float32, queue_size = 1)
	p0_robot_angle = rospy.Publisher("p0/robot_angle", Float32, queue_size = 1)
	
	#objetos detectados
	p0_hasTarget = rospy.Publisher("p0/hasTarget", Int8, queue_size = 1)
	p0_hasRobot = rospy.Publisher("p0/hasRobot", Int8, queue_size = 1)
	p0_hasAlien = rospy.Publisher("p0/hasAlien", Int8, queue_size = 1)

	
	#objects creation
	for numRobot in range(0, quantRobots):
	        strNumRobot = 'p' + str(numRobot)
	    
		velocity_publishers.append(rospy.Publisher(strNumRobot + "/cmd_vel", Twist, queue_size = 1))
		#twist_subscribers.append(rospy.Subscriber(strNumRobot + "/cmd_vel", Twist, getTwist))
		odom_twist_subscribers.append(rospy.Subscriber(strNumRobot + "/odom", Odometry, getRealTwist, callback_args=numRobot))
		laser_subscribers.append(rospy.Subscriber(strNumRobot + "/p3dx/laser/scan", LaserScan, getLaser, callback_args=numRobot))
		print("Running for p%2d. Please, wait ..." % (numRobot))
					
		robots.append(LocalObject(ID_TYPES))
		targets.append(LocalObject(ID_TYPES))
		aliens.append(LocalObject(ID_TYPES))
		
		ctrlCircum.append(circum())
		ctrlTwists.append(Twist())
		realTwists.append(Twist())
		
		#instancias de representacoes locais simplificadas de coordenadas/velocidades
		robots_coords.append(LinAng())
		targets_coords.append(LinAng())
		aliens_coords.append(LinAng())
		
		velocities.append(LinAng())
		
		debug_msgs.append("")
	
	#publicando os dados estaticos - removidos de dentro do while
	sp_desired_radius_to_target.publish(sp.desired_radius)
	sp_desired_angle_to_target.publish(sp.desired_angle_to_target)
	sp_minimal_distance.publish(sp.min_distance_to_robot)
	sp_desired_distance.publish(sp.desired_distance_to_robot)
	
	while not rospy.is_shutdown():
	  		
		#prev = statusChange(prev,status)
		#status = getStatus()
		#twist.linear.x = MIN_LINEAR_VEL
		
		#print "---- While ---- "
		#print "----------------"
		
		for numRobot in range(0, quantRobots):
			
			#envia comando a cada robo antes do controle rate.sleep()
			#numRobotrint "numRobot", numRobot
			#print "-" * 40
			#print ""
			
			#se houver target no cone do sensor, pega valores diferentes de 0.0
			if targets[numRobot].isTarget():
				#targets_coords[numRobot].linear, targets_coords[numRobot].angular = targets[numRobot].center.getCoordPol()
				targets_coords[numRobot].setAllCoords(targets[numRobot].center.getAllCoords())
				debug_msgs[numRobot] = "| Target |"
				#bea.prn()
			else:
				targets_coords[numRobot].clear()
				debug_msgs[numRobot] = "|        |"
		      
			#se houver o robo no cone do sensor, pega valores diferentes de 0.0
			if robots[numRobot].isRobot():
				#robots_coords[numRobot].linear, robots_coords[numRobot].angular = robots[numRobot].center.getCoordPol()
				robots_coords[numRobot].setAllCoords(robots[numRobot].center.getAllCoords())
				debug_msgs[numRobot] = debug_msgs[numRobot] + "| Robot |"
				#rob.prn()
			else:
				robots_coords[numRobot].clear()
				debug_msgs[numRobot] = debug_msgs[numRobot] + "|       |"
		      
			#se houver alien no cone do sensor, pega valores diferentes de 0.0
			if aliens[numRobot].isAlien():
				#aliens_coords[numRobot].setPolarCoords(aliens[numRobot].center.getCoordPol())
				aliens_coords[numRobot].setAllCoords(aliens[numRobot].center.getAllCoords())
				debug_msgs[numRobot] = debug_msgs[numRobot] + "| Alien |"
				#ali.prn()
			else:
				aliens_coords[numRobot].clear()
				debug_msgs[numRobot] = debug_msgs[numRobot] + "|       |"
		
			#obtem a velocidade atual
			velocities[numRobot].setVelocities(realTwists[numRobot].linear.x, realTwists[numRobot].angular.z)
			
			#vel.prn()
			
			print debug_msgs[numRobot]
		
			
			#obtem o horario atual
			now = rospy.get_rostime()
			
				
			#processa a circunavegacao
			ctrlTwists[numRobot].linear.x, ctrlTwists[numRobot].angular.z = ctrlCircum[numRobot].getVelocities(numRobot, velocities[numRobot], targets_coords[numRobot], robots_coords[numRobot], aliens_coords[numRobot], now).getVelocities()
			#twist.linear.x = 0.25
		
			
			
			#envie as informacoes de P0 para plotagem
			if(numRobot == 0):
				p0_radius_to_target.publish(ctrlCircum[numRobot].obtainedRadiusToTarget)
				p0_angle_to_target.publish(ctrlCircum[numRobot].obtainedAngleToTarget)
				p0_robot_angle.publish(ctrlCircum[numRobot].obtainedAngleToRobot)
				p0_robot_distance.publish(ctrlCircum[numRobot].obtainedDistanceToRobot)
				p0_approach_radius_to_target.publish(ctrlCircum[numRobot].approachRadius)
				p0_hasRobot.publish(ctrlCircum[numRobot].hasRobot)
				p0_hasTarget.publish(ctrlCircum[numRobot].hasTarget)
				p0_hasAlien.publish(ctrlCircum[numRobot].hasAlien)
				pRadius = NaN
				if velocities[numRobot].angular != 0.0:
					pRadius = velocities[numRobot].linear/ velocities[numRobot].angular
					#print "Real Velocity : vL =", velocities[numRobot].linear, " vA =",  velocities[numRobot].angular, "pRadius =", pRadius
				p0_performed_radius.publish(pRadius)
				  
			#envie ao robo os valores das velocidades
			velocity_publishers[numRobot].publish(ctrlTwists[numRobot])
			

		
		
		#for numRobot in range(0, quantRobots):
			#print "P", numRobot, "vl: ", twists[numRobot].linear.x, ", va: ", twists[numRobot].angular.z
		
		#estabelece uma pausa para manter a frequencia definida
		rate.sleep()

	print "ending control.py..."	
	#twist.angular.z = 0.0
	#twist.linear.x = 0.0
	#p1_pub.publish(twist)
	
	exit()
	
if __name__ == "__main__":
   main(sys.argv[1:])