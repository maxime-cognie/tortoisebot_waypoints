#! /usr/bin/env python

from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal
from std_srvs.srv import Empty, EmptyRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
import actionlib
import rospy
import rosunit
import unittest
import rostest
import time
import math
PKG = 'tortoisebot_waypoints'
NAME = 'tortoisebot_waypoints_test'

class TestTortoisebotWaypoints(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_node')
        # parameters for the test
        self.goal = Point()
        self.goal.x = rospy.get_param('goal_x', 0.0)
        self.goal.y = rospy.get_param('goal_y', 0.0)
        self.goal.z = rospy.get_param('goal_yaw', 0.0)
        self.goal_precision_xy_param = rospy.get_param('precision_xy', 0.1)
        self.goal_precision_yaw_param = rospy.get_param('precision_yaw', 0.2)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_odom = Odometry()

        # self.service_call_reset()

        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        self.client.wait_for_server()
        self.success = False

    def odom_callback(self, msg):

        self.current_odom = msg

    # return current yaw angle
    def quaternion_to_euler(self, quat):

        orientation_list = [quat.x, quat.y, quat.z, quat.w]
        (_, _, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    # send goal to the server
    def action_server_call(self):

        g = WaypointActionGoal()
        g.position = self.goal
        self.client.send_goal(g)

    # reset position of the robot
    def service_call_reset(self):

        rospy.wait_for_service('/gazebo/reset_world')
        s = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        resp = s.call(EmptyRequest())

    def test_goal_pos_reached(self):

        self.action_server_call()
        # wait until the goal is reached
        self.success = self.client.wait_for_result(rospy.Duration(30.0))

        # verify that the goal is reached
        self.assertTrue(self.success)

        final_pos = self.current_odom.pose.pose.position

        dist_to_goal = math.sqrt(pow(self.goal.y - final_pos.y, 2) +
            pow(self.goal.x - final_pos.x, 2))
        # test that the final position of the robot is within the range of the goal
        self.assertTrue(dist_to_goal < self.goal_precision_xy_param,
            "ERROR. Action server call error the robot didn't reach the goal")

    def test_goal_yaw_reached(self):

        final_yaw = self.quaternion_to_euler(self.current_odom.pose.pose.orientation)

        # test that the final orientation of the robot is the same as the goal
        error_yaw = self.goal.z - final_yaw
        self.assertTrue((math.fabs(error_yaw) < self.goal_precision_yaw_param),
            "ERROR. final YAW != goal YAW")

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestTortoisebotWaypoints)