#!/usr/bin/env python
import getch
import roslib; roslib.load_manifest('p3dx_mover')
import rospy
from random import randint


from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(data):
	p = 0
	rospy.loginfo("%f, %f", data.range_min, data.range_max)
	while(p < 270):
		rospy.loginfo("[%d] = %f, %f", p, data.ranges[p], data.intensities[p])
		p = p + 1
	#rospy.loginfo(rospy.get_namespace() + ": I heard %s")

if __name__ == '__main__':
	while(1):
		p1_pub = rospy.Publisher('p1/cmd_vel', Twist, queue_size = 10)
		p2_pub = rospy.Publisher('p2/cmd_vel', Twist, queue_size = 10)
		#p1_lis = rospy.Subscriber("p1/p3dx/laser/scan", LaserScan, callback)
		#p2 = rospy.Publisher('p2/cmd_vel', Twist, queue_size = 10)

		rospy.init_node('p3dx_mover')

		twist = Twist()
		
		
		twist.linear.x = randint(1,9)/10.0
		p1_pub.publish(twist)
		p2_pub.publish(twist)
		rospy.sleep(5)
		
		twist.linear.x = 0.0
		p1_pub.publish(twist)
		p2_pub.publish(twist)		
		
		    
		
		twist.angular.z = 1.57/2
		p1_pub.publish(twist)
		p2_pub.publish(twist)
		rospy.sleep(2)

		twist.angular.z = 0.0
		p1_pub.publish(twist)
		p2_pub.publish(twist)
	
	twist = Twist()
	p1.publish(twist)
	p2.publish(twist)
	exit()
	
