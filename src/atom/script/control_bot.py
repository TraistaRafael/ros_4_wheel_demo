#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def stop(vel_msg):
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0

def move():
	pub = rospy.Publisher('/atom/cmd_vel', Twist, queue_size=10)
	rospy.init_node('bot_controller', anonymous=True)
	rate = rospy.Rate(10)
	linear_acc = 0.5
	angular_acc = 0.1

	vel_msg = Twist()
	stop(vel_msg)

	print("Use w,s,a,d to move the bot and press q for exit")

	while not rospy.is_shutdown():
		keyPressed = input()
		if keyPressed=='q':
			stop(vel_msg)
			pub.publish(vel_msg)
			break
		elif keyPressed=='w':
			vel_msg.linear.x = vel_msg.linear.x + linear_acc

		elif keyPressed=='a':
			vel_msg.angular.z = vel_msg.angular.z + angular_acc

		elif keyPressed=='s':
			vel_msg.linear.x = vel_msg.linear.x - linear_acc

		elif keyPressed=='d':
			vel_msg.angular.z = vel_msg.angular.z - angular_acc

		elif keyPressed=='x':
			stop(vel_msg)
			
		print ("send twist : ", str(vel_msg))

		vel_msg.angular.z + 0.01
		pub.publish(vel_msg)


if __name__ == '__main__':
	try:
		move()
	except rospy.ROSInterruptException:
		pass