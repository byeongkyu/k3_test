#!/usr/bin/env python

import rospy
import actionlib
import math

import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from k3_test_waypoint_navigation.msg import WaypointNavigationAction, WaypointNavigationResult, WaypointNavigationFeedback


class WaypointNavigationNode:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.current_odom = Odometry()
        rospy.Subscriber('odom', Odometry, self.handle_odometry_info)

        self.pub_move_base_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.loginfo("[%s] wait for move_base"%rospy.get_name())
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        self._as = actionlib.SimpleActionServer('waypoint_move', WaypointNavigationAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        rospy.loginfo("[%s] initialized..."%rospy.get_name())
        rospy.spin()

    def handle_odometry_info(self, msg):
        self.current_odom = msg

    def execute_cb(self, goal):
        succes = False
        feedback = WaypointNavigationFeedback()
        result = WaypointNavigationResult()


        for wp in goal.waypoints:
            wp.header.stamp = rospy.Time.now()

            feedback.current_goal = wp
            self._as.publish_feedback(feedback)
            self.pub_move_base_goal.publish(wp)

            is_reached_waypoint = False
            while not rospy.is_shutdown() and not is_reached_waypoint:
                try:
                    transform = self.tfBuffer.lookup_transform("base_footprint", "map", rospy.Time(0))
                    pose_transformed = tf2_geometry_msgs.do_transform_pose(wp, transform)

                    dx = pose_transformed.pose.position.x
                    dy = pose_transformed.pose.position.y

                    dist = math.sqrt((dx * dx) + (dy * dy))
                    if dist < 2.0 * self.current_odom.twist.twist.linear.x:
                        is_reached_waypoint = True

                    rospy.sleep(0.1)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue

            rospy.loginfo('reached...')

        move_goal = MoveBaseGoal()
        move_goal.target_pose = goal.final_goal
        move_goal.target_pose.header.stamp = rospy.Time.now()

        self.move_base_client.send_goal(move_goal)
        self.move_base_client.wait_for_result()

        if succes:
            result.result = True
            self._as.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node('waypoint_navigation_node', anonymous=False)
    try:
        m = WaypointNavigationNode()
    except rospy.ROSInterruptException: pass
