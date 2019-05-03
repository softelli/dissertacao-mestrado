#!/usr/bin/env python

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
import numpy as np

#from Circumnavigation import circum


#static parameters
from ParametersServer import staticParameters as sp

MAX_READS = sp.max_laser_reads
RADIO_TARGET = sp.target_radius_id
RADIO_ROBOT = sp.robot_radius_id
#RADIO_TOL = sp.radius_tolerance
ID_TYPES = sp.id_types
LINEAR_VEL = sp.linear_velocity
MIN_LINEAR_VEL = sp.min_linear_velocity
ANGULAR_VEL = sp.angular_velocity
MIN_ANGULAR_VEL = sp.min_angular_velocity
MAX_ANGULAR_VEL = sp.max_angular_velocity


def create_pub(nome, atributo, tipo = Float32, tam_fila = 1):
	return rospy.Publisher("%s/%s" % (nome, atributo), tipo, queue_size=tam_fila)

def create_sub(nome, atributo, origem, callback):
	return rospy.Subscriber("%s/%s" % (nome, atributo), origem, callback)

def boundv(minv, value, maxv):
	return min(max(minv, value), maxv)


DES_RAD_TO_TARGET = create_pub("sp","desired_radius_to_target")
DES_ANG_TO_TARGET = create_pub("sp","desired_angle_to_target")
MIN_DISTANCE = create_pub("sp","minimal_distance")
DES_DISTANCE = create_pub("sp","desired_distance")


def lista_objetos(data):
	obj = LocalObject(ID_TYPES)
	limit = data.range_max * (1 - 1.5 * RADIO_ROBOT)
	retorno = dict(robots = [], targets=[], aliens=[])
	
	for p in xrange(MAX_READS):
		indice = math.degrees(data.angle_min + p * data.angle_increment)
		 
		if data.ranges[p] < limit:
			if not obj.p1.exists:
				obj.p1.setCoordPol(data.ranges[p], indice)
				continue
			if not obj.p2.exists:
				obj.p2.setCoordPol(data.ranges[p], indice)
				continue
			else:
				if obj.p2.radius > data.ranges[p]:
					obj.p2.setCoordPol(data.ranges[p], indice)
					continue
			if not obj.p3.exists:
				if ((p + 1) < MAX_READS) and (data.ranges[p + 1] < limit): 
					#se nao for o ultimo do indice
					# e o proximo valor for valido
					continue
				else: # eh o ultimo indice ou o proximo eh invalido
					obj.p3.setCoordPol(data.ranges[p], indice)
					#obj.p3.prt(3)
					
					#calcula a circunferencia e identifica o objeto
					obj.identify(sp.target_radius_id, sp.robot_radius_id) #, RADIO_TOL)
					
					#insere uma copia profunda e desvinculada desse objeto
					#listaObjetos.append(deepcopy(obj))

					if obj.isRobot():
						retorno["robots"].append(obj)
					elif obj.isTarget():
						retorno["targets"].append(obj)
					elif obj.isAlien():
						retorno["aliens"].append(obj)
					#listaObjetos.append(obj)
					#reinicia o objeto para reuso
					obj = LocalObject(ID_TYPES)
				
	
	return retorno

#def boltzmann(x, mu, sigma):
#	return 1/(1 + exp(-(x - mu)/sigma))

def prop_ang_coef(angulo):
	if angulo <= sp.min_ctrl_angle:
		return -1.0
	elif angulo >= sp.max_ctrl_angle:
		return 1.0
	else:
		return 2 * (angulo - sp.min_ctrl_angle) / (sp.max_ctrl_angle - sp.min_ctrl_angle) - 1

class Robo(object):

	def _c_pub(self, atributo, tipo = Float32):
		return create_pub(self.nome, atributo, tipo)

	def _c_sub(self, atributo, origem, callback):
		return create_sub(self.nome, atributo, origem, callback)

	def __init__(self, num, taime):
		self.num = num
		self.app_radius = 0.0
		self.performed_radius = 0.0
		self.radius = self._c_pub("radius_to_target")
		self.angle_to_target = self._c_pub("angle_to_target")
		self.approach = self._c_pub("approach_radius_to_target")
		self.perf_radius = self._c_pub("performed_radius")
		self.distance = self._c_pub("robot_distance")
		self.angle = self._c_pub("robot_angle")
		self.status_p = self._c_pub("current_status")

		self.has_target_p = self._c_pub("hasTarget", Int8)
		self.has_robot_p = self._c_pub("hasRobot", Int8)
		self.has_alien_p = self._c_pub("hasAlien", Int8)

		#self.vizinhos = []

		self.vel_publ = self._c_pub("cmd_vel", Twist)
		self.odo_twist_sub = self._c_sub("odom", Odometry, self.set_real_twist)
		self.laser_sub = self._c_sub("p3dx/laser/scan", LaserScan, self.set_laser)

		self.control_twist = Twist()
		self.laser_data = dict(robots=[],targets=[],aliens=[])

		self.status = 0
		self.time = taime.secs + (taime.nsecs / 1E+9)

		self.linear_speed = sp.linear_velocity
		self.angular_speed = sp.angular_velocity

	@property
	def nome(self):
		return "p%d" % self.num

	@property
	def has_target(self):
		return len(self.laser_data["targets"]) > 0

	@property
	def has_robot(self):
		return len(self.laser_data["robots"]) > 0

	@property
	def has_alien(self):
		return len(self.laser_data["aliens"]) > 0

	def update(self):
		self.radius.publish(self.target.center.radius)
		self.angle_to_target.publish(self.target.center.angle)
		self.approach.publish(self.app_radius)
		self.perf_radius.publish(self.performed_radius)
		#self.distance.publish(self.robot.) = self._c_pub("robot_distance")
		#self.angle = self._c_pub("robot_angle")
		self.status_p.publish(self.status)

		self.has_target_p.publish(int(self.has_target))
		self.has_robot_p.publish(int(self.has_robot))
		self.has_alien_p.publish(int(self.has_alien))

		self.vel_publ.publish(self.control_twist)

	def set_real_twist(self, odom):
		self._twist = odom.twist.twist

	def set_laser(self, data):
		self.laser_data = lista_objetos(data)

	@property
	def linear_speed(self):
		return self.control_twist.linear.x

	@linear_speed.setter
	def linear_speed(self, value):
		self.control_twist.linear.x = value

	@property
	def angular_speed(self):
		return self.control_twist.angular.z

	@angular_speed.setter
	def angular_speed(self, value):
		self.control_twist.angular.z = value


	def linear_add(self, value):
		self.control_twist.linear.x += value

	def angular_add(self, value):
		self.control_twist.angular.z += value

	@property
	def robot(self):
		if self.has_robot:
			lrob = sorted(self.laser_data['robots'], key=lambda o : o.center.radius)
			return lrob[0] 
		else:
			return LocalObject(ID_TYPES)

	@property
	def target(self):
		if self.has_target:
			return self.laser_data['targets'][0]
		else:
			return LocalObject(ID_TYPES)

	def adjust_lin_velocity(self, v):
		objects = self.laser_data['robots'] + self.laser_data['targets']
		for o in objects:
			la = LinAng()
			la.setAllCoords(o.center.getAllCoords())
			if 0 < la.linear < sp.min_distance_to_robot and abs(la.angular) < 45:
				print ">>>>> STOP <<<<<"
				return 0.1
		return v


	def process(self, now):
		t = now.secs + (now.nsecs / 1E+9)
		delta_time, self.time = t - self.time, t

		#self.setTime(now.secs + now.nsecs / (10**9)) # segundos.milisegundos
		
		if self.has_target and self.has_robot: #Escorting
			self.status = 3
			robot_dist_norm = abs(self.robot.center.radius - sp.desired_distance_to_robot) / sp.sensor_cone_radius

			# primeiro verifica se jah estah circunavegando...
			if abs(self.target.center.radius - sp.desired_radius) < 0.5:				
				self.linear_speed = self.linear_speed * robot_dist_norm
			#pacr = -1.0

			#if self.target.angular <= sp.min_ctrl_angle: 
			#	pacr = -1.0
			#elif self.target.angular >= sp.max_ctrl_angle:
			#	pacr =  1.0
			#else: 
			#	pacr = 2 * (self.target.angular - sp.min_ctrl_angle) / (sp.max_ctrl_angle - sp.min_ctrl_angle) - 1
	
			#self.propAngularCoefToRobot = pacr
			#pacr = prop_ang_coef(self.laser_data["robots"][0])

			#self.linear_speed = 2 * sp.linear_velocity
			# self.proximityCoefToRobot = self.getProximityCoefToRobot() # * self.getPropAngularCoefToRobot()
			# self.linearVelocity = sp.linear_velocity + sp.linear_velocity * self.propAngularCoefToRobot * self.proximityCoefToRobot
			

		elif self.has_target and not self.has_robot: #Circulating
			self.status = 2
			tcoords = LinAng()
			tcoords.setAllCoords(self.target.center.getAllCoords())
			radius_coef = 1/(1 + exp(-(tcoords.linear - sp.desired_radius)/sp.boltzmann_time_constant))
	
			#circRadiusCoef, appRadiusCoef = self.getTransitionCoefs()
			kpA = prop_ang_coef(tcoords.angular)
			error_radius = sp.desired_radius - tcoords.linear
			#error_radius = sp.desired_radius - self.target.linear
			self.app_radius = -(tcoords.x**2 + tcoords.y**2 - sp.desired_radius**2) / (2 * (sp.desired_radius - tcoords.y)) 
			#appRadius = -(self.target.x**2 + self.target.y**2 - sp.desired_radius**2) / (2 * (sp.desired_radius - self.target.y))
			
			circ_radius = sp.desired_radius + error_radius
			#circRadius = sp.desired_radius + error_radius
			self.performed_radius = (1 - radius_coef) * circ_radius + radius_coef * self.app_radius
			# ou radius = circ_radius + radius * (app_radius - circ_radius)
			ang_vel = self.linear_speed / self.performed_radius
			hl_ang_vel = boundv(sp.min_angular_velocity, ang_vel, sp.max_angular_velocity)
			hl_ang_vel += kpA * sp.delta_angular
			self.linear_speed = sp.linear_velocity
			self.angular_speed = boundv(sp.min_angular_velocity, hl_ang_vel, sp.max_angular_velocity)
			
		elif not self.has_target and self.has_robot: #Avoiding
			self.status = 1
			speed = (sp.linear_velocity + self.linear_speed) / 2.0
			if speed > 0:
				time_to_collide = abs(self.robot.center.radius - sp.desired_distance_to_robot) / speed
				if (time_to_collide < 5) and abs((self.angular_speed * time_to_collide) % 180) < 15:
					self.linear_speed = self.linear_speed * (0.8 - 0.1 * (5 - time_to_collide))

		else: #status == "Seeking"
			self.status = 0
			self.linear_speed = sp.linear_velocity
			decimal_random_rate = np.random.random() / 10.0
			self.angular_speed = sp.angular_velocity  * (1 + decimal_random_rate)
		self.linear_speed = boundv(0.0, self.linear_speed, sp.max_linear_velocity)
		self.linear_speed = self.adjust_lin_velocity(self.linear_speed)
		print "[" + str(t) + "] Robo=p" + str(self.num) + "     status=" + str(self.status)
		self.update()

	def __str__(self):
		return "%s %f %f %d" % (self.nome, self._twist.angular.z, self._twist.linear.x. len(self.laser_data))


class Controle(object):
	def __init__(self, numRobots):
		rospy.init_node('p3dx_mover')
		self.numRobots = numRobots
		self.robos = []
		for i in range(0, numRobots):
			self.robos.append(Robo(i, rospy.get_rostime()))

	def run(self):

		rate = rospy.Rate(50)

		DES_RAD_TO_TARGET.publish(sp.desired_radius)
		DES_ANG_TO_TARGET.publish(sp.desired_angle_to_target)
		MIN_DISTANCE.publish(sp.min_distance_to_robot)
		DES_DISTANCE.publish(sp.desired_distance_to_robot)

		while not rospy.is_shutdown():
			for robo in self.robos:
				robo.process(rospy.get_rostime())

			rate.sleep()


if __name__  == "__main__":
	import sys
	Controle(int(sys.argv[1])).run()	

