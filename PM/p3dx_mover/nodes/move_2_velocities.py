#!/usr/bin/env python
import getch
import roslib; roslib.load_manifest('p3dx_mover')
import rospy
import sys


from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(data):
	p = 0
	rospy.loginfo("%f, %f", data.range_min, data.range_max)
	while(p < 360):
		rospy.loginfo("[%d] = %f, %f", p, data.ranges[p], data.intensities[p])
		p = p + 1
	#rospy.loginfo(rospy.get_namespace() + ": I heard %s")

def main(argv):
	rospy.loginfo("Twist.linear.x =  [%f], Twist.angular.z = [%f]", sys.argv[1], sys.argv[2])
	rospy.init_node('p3dx_mover')
	while(1):
		p1_pub = rospy.Publisher('p1/cmd_vel', Twist, queue_size = 3)
		#p1_lis = rospy.Subscriber("p1/p3dx/laser/scan", LaserScan, callback)
		#p2 = rospy.Publisher('p2/cmd_vel', Twist, queue_size = 10)

		twist = Twist()
		twist.linear.x = 0
		twist.angular.z = 0

		p1_pub.publish(twist)
		#p2.publish(twist)

	#p1_pub = rospy.Publisher('p1/cmd_vel', Twist, queue_size = 10)
	#p1_lis = rospy.Subscriber("p1/p3dx/laser/scan", LaserScan, funCallback)
	#p2 = rospy.Publisher('p2/cmd_vel', Twist, queue_size = 10)
	#rospy.init_node('p3dx_mover')
	#twist = Twist()
	#p1.publish(twist)
	#p2.publish(twist)
	exit()

if __name__ == '__main__':
	main(sys.argv[1:])
	
