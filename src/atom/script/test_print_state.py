#!/usr/bin/env python

# Subscribe to model state and print current location

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

prev_x = 0

def model_states_callback(msg):
    global prev_x
    
    try:
        index = msg.name.index("atom")
        robot_pose = msg.pose[index]
        if abs(robot_pose.position.x - prev_x) > 0.1 :
            prev_x = robot_pose.position.x
            rospy.loginfo("Robot Pose: {:.2f} {:.2f} {:.2f}".format(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z))
    except ValueError:
        rospy.logwarn("Robot not found in model_states")

if __name__ == '__main__':
    rospy.init_node('robot_pose_reader')

    # Set the rate of the node to 10 Hz
    rate = rospy.Rate(10)

    # Subscriber for model states
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

    rospy.spin()