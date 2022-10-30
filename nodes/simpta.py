#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

from tf.transformations import quaternion_from_euler, euler_from_quaternion

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

class ActionClientNode:
    def __init__(self):
        self.count = 0
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        rospy.loginfo("Starting: message publisher node")
        self.pub_init = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.header.frame_id = "map"
        self.init_pose.header.stamp = rospy.Time.now()
        self.init_pose.pose.pose.position.x = 0.0
        self.init_pose.pose.pose.position.y = 0.0
        self.init_pose.pose.pose.orientation.w = 1.0
        self.init_theta = 0.0
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()

    def update_init_pose(self, x, y, theta):
        self.init_pose.header.stamp = rospy.Time.now()
        self.init_pose.pose.pose.position.x += x
        self.init_pose.pose.pose.position.y += y
        self.init_theta += theta
        q = quaternion_from_euler(0.0, 0.0, self.init_theta)
        rospy.loginfo(self.init_pose.pose.pose)
        self.init_pose.pose.pose.orientation.x = q[0]
        self.init_pose.pose.pose.orientation.y = q[1]
        self.init_pose.pose.pose.orientation.z = q[2]
        self.init_pose.pose.pose.orientation.w = q[3]


    def main(self):
        #rospy.spin()
            self.update_init_pose(0.1, 0.0, 0.0)
            self.update_init_pose(0.1, 0.0, 0.0)
            self.update_init_pose(0.1, 0.0, 0.0)
            self.update_init_pose(0.1, 0.0, 0.0)
            self.update_init_pose(0.1, 0.0, 0.0)

if __name__ == '__main__':
    try :
        rospy.init_node('action_client_node')
        node = ActionClientNode()
        node.main()
    except rospy.ROSInterruptException:
        pass