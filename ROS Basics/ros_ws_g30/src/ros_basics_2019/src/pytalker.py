#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Point,Pose

def talker():
	pos=rospy.Publisher('goal_pos',Point,queue_size=5) 
	rospy.init_node('talker',anonymous=True)
	rate = rospy.Rate(5)
	goal_poses = [[5,5]]

	#while not rospy.is_shutdown():
	for goal_pos in goal_poses:
		point = Point()
		point.x = goal_pos[0]
		point.y = goal_pos[1]
		point.z = 0
		pos.publish(point)
		rate.sleep()

	rospy.spin() 

if __name__ == '__main__':
	talker()