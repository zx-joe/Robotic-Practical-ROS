#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Point,Pose
from math import atan2, pow, sqrt, pi

class velocityControl:
 
    def __init__(self):
	self.goalPoslist = []
	point = Point()
	# used for testing
        point.x = 0
        point.y = 0
 	self.goalPoslist.append(point)
	
        rospy.init_node('velocity_controller_G30', anonymous=True)
        self.goal_pos_sub = rospy.Subscriber('/goal_pos', Point, self.stackPos)
        self.current_pose_sub = rospy.Subscriber('/odom', Odometry, self.newPosition)
        self.velocity_pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 10) 
	#to initialize
        self.pos = Point()
        self.pos.x = 0
        self.pos.y = 0 
        self.pos.z = 0.05 # in accordance with the launch file
        self.yaw = 0
	self.vx = 0
	self.angVel = 0
        self.rate = rospy.Rate(20)

    def stackPos(self, newPos):
	#to accumulate the positions published by the topic of /goal_pos
        self.goalPoslist.append(newPos)

    def newPosition(self, odom):

        # geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        # odom.pose.pose.orientation = odom_quat;
        self.pos.x = odom.pose.pose.position.x
        self.pos.y = odom.pose.pose.position.y
        self.pos.z = odom.pose.pose.position.z
        orientation = odom.pose.pose.orientation
	# got yaw angle from quaternion 
        (_, _, self.yaw) = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
	# store current linear velocity 
	self.vx = odom.twist.twist.linear.x 
	# store current angular velcoity
	self.angVel = odom.twist.twist.angular.z;


    def distance(self, final):
        dist = pow((final.x - self.pos.x), 2) + pow((final.y - self.pos.y), 2)
        dist = sqrt(dist)
        return dist

    #PD control
    def velPID(self, final, kp = 0.82, kd = 0.8):
	velocity = kd * self.vx + kp * self.distance(final)
	#saturate
	if(velocity > 0.2):
	    velocity = 0.2
	elif(velocity < -0.2):
	    velocity = -0.2
        return velocity


    def orientation_angle(self, goal):
	# orientation angle of current position and goal position 
        angle = atan2((goal.y - self.pos.y),(goal.x - self.pos.x))
        #angle %= 2*pi
	while abs(angle) > pi:
	    if(angle > pi):
	        angle -= 2 * pi
	    elif(angle < -pi):
	        angle += 2 * pi

	return angle #range[-pi, pi]

    #PD controller
    def anglePID(self, goal, kd = 0.1, kp = 1):

	angle_difference = self.orientation_angle(goal) - self.yaw
	angle_difference = self.clip(angle_difference)
	angular_vel = kd * self.angVel + kp * angle_difference
	print(angle_difference)
        return angular_vel

    def clip(self,angle):
	while abs(angle) > pi:
	    if(angle > pi):
	        angle -= 2 * pi
	    elif(angle < -pi):
	        angle += 2 * pi
	return angle

   
    def move(self):

        distance_tol = 0.05 
        angular_tol = 0.20 #0.08
        # define it with Twist type
        vel_control = Twist()

        while(len(self.goalPoslist)>0):

            while self.distance(self.goalPoslist[0]) > distance_tol:

                vel_control.angular.z = self.yaw

		while abs(self.clip(self.yaw - self.orientation_angle(self.goalPoslist[0]))) > angular_tol:
		    
                    vel_control.angular.z =self.anglePID(self.goalPoslist[0])
                    vel_control.linear.x = 0 # here we seperate the task into 2 parts, so we did not assign linear velocity 
                    vel_control.linear.y = 0
                    vel_control.linear.z = 0

                    print(self.pos)
                    self.velocity_pub.publish(vel_control)
                    self.rate.sleep()

                for i in range(20):

                    vel_control.angular.z = 0
                    vel_control.linear.x = self.velPID(self.goalPoslist[0])
                    vel_control.linear.y = 0
                    vel_control.linear.z = 0

                    self.velocity_pub.publish(vel_control)
                    self.rate.sleep()

		print(self.pos)
                # to adjust velocity
                vel_control.linear.x = self.velPID(self.goalPoslist[0])
                vel_control.linear.y = 0
                vel_control.linear.z = 0
                self.velocity_pub.publish(vel_control)
                self.rate.sleep()
            #when approached, delete it
	    self.goalPoslist.pop(0)

        # stop the vehcle when it approachs the goal position
        vel_control.linear.x = 0
        vel_control.angular.z = 0
        self.velocity_pub.publish(vel_control)
	self.rate.sleep()
	#rospy.spin()

    def loop2approach(self):
        while(True):
	    self.move()

if __name__ == '__main__':

    thymio = velocityControl()
    thymio.loop2approach() 

