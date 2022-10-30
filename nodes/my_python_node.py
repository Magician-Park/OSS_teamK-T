#!/usr/bin/env python

import sys
import rospy
import actionlib
from geometry_msgs.msg import Twist, P

from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rospy.init_node('my_python_node')
pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pub_init = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
print(client.wait_for_server())

def update_init_pose(self, x, y, theta):
    init_pose = PoseWithCovarianceStamped()
    self.init_pose.header.stamp = rospy.Time.now()
    self.init_pose.pose.pose.position.x = x
    self.init_pose.pose.pose.position.y = y
    self.init_pose.pose.pose.orientation.w = 1.0
    q = quaternion_from_euler(0.0, 0.0, theta)
    self.init_pose.pose.pose.orientation.x = q[0]
    self.init_pose.pose.pose.orientation.y = q[1]
    self.init_pose.pose.pose.orientation.z = q[2]
    self.init_pose.pose.pose.orientation.w = q[3]
    pub_init.publish(init_pose)

def send_goal(x, y, theta):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame.id = "map"
    goal.target_pose.header.stemp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    quat = quaternion_from_euler(0.0, 0.0, theta)
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[0]
    goal.target_pose.pose.orientation.z = quat[0]
    goal.target_pose.pose.orientation.w = quat[0]
    wait = client.wait_for_result()
    if not wait:
        print('Error')
    else:
        print(client.get_result())

update_init_pose(-2.0, -0.3, 0.0)
send_goal(3.0, 3.0, 0.0)
#Added another waypoint
send_goal(-1.0, 1.0, 0.0)
send_goal(-2.0, -0.3, 0.0)

def update_timer(timer_event):
    print('update time')

rospy.Timer(rospy.Duration(0.5), update_timer)

rospy.spin()