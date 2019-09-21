#!/usr/bin/env python
import rospy
import tf
import tf.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import math
import random

class Goals:
  def __init__(self):
    rospy.init_node("odom_to_base_link")
    self.pub = rospy.Publisher("~/CVT/waypoint_list", PoseArray, queue_size=1)
    while True:
		msg = PoseArray()
		goal1 = Pose()
		goal1.position.x = 0
		goal1.position.y = 0
		goal2 = Pose()
		goal2.position.x = 3
		goal2.position.y = 0
		goal3 = Pose()
		goal3.position.x = 3
		goal3.position.y = 3
		goal4 = Pose()
		goal4.position.x = 0
		goal4.position.y = 3
		msg.poses.append(goal1)
		msg.poses.append(goal2)
		msg.poses.append(goal3)
		msg.poses.append(goal4)
		self.pub.publish(msg)

if __name__ == "__main__":
	Goals()