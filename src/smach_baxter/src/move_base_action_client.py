#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient

class MoveRidgeback(object):
    def __init__(self):
        self.node_name = "Move_Base action client"
        rospy.loginfo("%s: Waiting for move_base action server...", self.node_name)
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base_ac action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to move_base_ac action server", self.node_name)
        self.state = 0
        self.check_states()


    def check_states(self):

        while not rospy.is_shutdown() and self.state != 2:

            rospy.loginfo("%s: Going to pick up location", self.node_name)
            goal_pick = MoveBaseGoal()
            goal_pick.target_pose.header.frame_id = "map"
            goal_pick.target_pose.header.stamp = rospy.Time.now()
            goal_pick.target_pose.pose.position.x = 0.1
            goal_pick.target_pose.pose.position.y = 0
            goal_pick.target_pose.pose.position.z = 0.000
            goal_pick.target_pose.pose.orientation.x = 0.0
            goal_pick.target_pose.pose.orientation.y = 0.0
            goal_pick.target_pose.pose.orientation.z = -0.085
            goal_pick.target_pose.pose.orientation.w = 0.996

            self.move_base_ac.send_goal(goal_pick)
            wait = self.move_base_ac.wait_for_result(rospy.Duration(100.0))
            if not wait:
                rospy.logerr("Robot stuck or not able to reach pick up pose!")
                self.state = 1
            else:
                rospy.loginfo("%s: Pick up pose reached.", self.node_name)
                self.state = 2

if __name__ == "__main__":

    rospy.init_node('ridgeback move base')
    try:
        MoveRidgeback()
    except rospy.ROSInterruptException:
        pass
