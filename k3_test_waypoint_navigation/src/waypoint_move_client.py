#!/usr/bin/env python

import rospy
import actionlib
import math

from geometry_msgs.msg import PoseStamped
from k3_test_waypoint_navigation.msg import WaypointNavigationAction, WaypointNavigationGoal


class WaypointNavigationClient:
    def __init__(self):
        self.waypoints = []
        self.final_goal = PoseStamped()

        rospy.Subscriber('set_waypoint', PoseStamped, self.handle_set_waypoint)
        rospy.Subscriber('set_final_goal', PoseStamped, self.handle_set_final_goal)

        self.ac_waypoint = actionlib.SimpleActionClient('waypoint_move', WaypointNavigationAction)
        rospy.loginfo("[%s] wait for move_base"%rospy.get_name())
        self.ac_waypoint.wait_for_server()

        rospy.loginfo("[%s] initialized..."%rospy.get_name())
        rospy.spin()

    def handle_set_waypoint(self, msg):
        self.waypoints.append(msg)

    def handle_set_final_goal(self, msg):
        self.final_goal = msg

        goal = WaypointNavigationGoal()
        goal.waypoints = self.waypoints
        goal.final_goal = self.final_goal

        self.ac_waypoint.send_goal(goal, done_cb=self.handle_ac_done, active_cb=None, feedback_cb=self.handle_ac_feedback)
        rospy.loginfo("[%s] Send the goal with (%d) waypoints..."%(rospy.get_name(), len(self.waypoints)))

        self.waypoints = []
        self.final_goal = PoseStamped()

    def handle_ac_done(self, state, result):
        rospy.loginfo("[%s] Waypoint navigation done."%rospy.get_name())

    def handle_ac_feedback(self, feedback):
        rospy.loginfo("[%s] Waypoint reached, try next goal..."%rospy.get_name())


if __name__ == "__main__":
    rospy.init_node('waypoint_navigation_client', anonymous=False)
    try:
        m = WaypointNavigationClient()
    except rospy.ROSInterruptException: pass