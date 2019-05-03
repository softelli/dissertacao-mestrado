#!/usr/bin/env python
import getch
import roslib; roslib.load_manifest('p3dx_mover')
import rospy
import numpy as np


from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(data):
	p = 0
	rospy.loginfo("%f, %f", data.range_min, data.range_max)
	while(p < 360):
		rospy.loginfo("[%d] = %f, %f", p, data.ranges[p], data.intensities[p])
		p = p + 1
	#rospy.loginfo(rospy.get_namespace() + ": I heard %s")

if __name__ == '__main__':
	rospy.init_node('p3dx_target_mover')
	cont = 0
	linear = 0.15
	angular = 0.5
	limit = 20000
	girando = False
	
	while(1):
		p1_pub = rospy.Publisher('t0/cmd_vel', Twist, queue_size = 10)
		#p1_lis = rospy.Subscriber("p1/p3dx/laser/scan", LaserScan, callback)
		#p2 = rospy.Publisher('p2/cmd_vel', Twist, queue_size = 10)
		
		rs = np.random.random_sample()
		cont += 1
		
		if cont >= limit:
		    
		    cont = 0
		    
		    if girando == True:
			  limit = 300000 * rs
			  angular = 0.0
			  linear  = 0.2 * rs
			  girando = False
		    else:
			  limit = 100000 * rs
			  angular = 2.0 * rs
			  linear = 0.0
			  girando = True
		    print "changing limit to ", limit
			  
		twist = Twist()
		twist.linear.x = linear
		twist.angular.z = angular
		rospy.Rate(100)

		p1_pub.publish(twist)
		#p2.publish(twist)

	
	exit()
	
