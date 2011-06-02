#!/usr/bin/env python

import roslib; roslib.load_manifest("precision_steering_tests")
import rospy

import numpy
import math

import actionlib

from gazebo.srv import SetModelState
from gazebo.msg import ModelState
from geometry_msgs.msg import Pose
from precision_navigation_msgs.msg import ExecutePathAction
from precision_navigation_msgs.msg import ExecutePathGoal
from precision_navigation_msgs.msg import PathSegment
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

from tf import transformations as tf_math

ERROR_MAX = 0.04

class FindErrorContour:
    def __init__(self, outputfile):
        self.reset_pub = rospy.Publisher("reset_pose", Pose)
        self.gazebo_model_state_setter_service = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
        self.toggle_steering_commands = rospy.ServiceProxy("toggle_steering_commands", Empty)
        self.test_path = FindErrorContour.make_test_path()
        self.sample_done = True
        self.min_x = 0.0
        self.initial_y = 0.0
        self.action_client = \
                actionlib.SimpleActionClient("execute_path", ExecutePathAction)
        self.output_file = outputfile
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.handle_odom)
        while not rospy.is_shutdown() and not self.action_client.wait_for_server(rospy.Duration(5.0)):
                rospy.logwarn("Unable to connect to execute_path server. Retrying")
        rospy.loginfo("Execute path server found. Ready to continue with finding the error contours")

    def reset_harlie_state(self,x,y,theta):
        self.toggle_steering_commands()
        self.action_client.cancel_goal()
        rospy.sleep(2.5)
        reset_state = ModelState()
        reset_state.model_name = "harlie"
        reset_state.pose.position.x = x
        reset_state.pose.position.y = y
        reset_state.pose.orientation = \
            Quaternion(*(tf_math.quaternion_about_axis(theta, (0,0,1))))
        self.gazebo_model_state_setter_service(reset_state)
        i = 0
        while not rospy.is_shutdown():
            self.reset_pub.publish(reset_state.pose)
            rospy.sleep(0.1)
            i+=1
            if i > 10:
                break
        rospy.sleep(5.0)
        self.toggle_steering_commands()
        self.action_client.send_goal(self.test_path)
        self.output_file.write("%f, %f, %f," % (x, y, theta))
        self.min_x = 0.0
        self.initial_y = y
        self.sample_done = False

    def handle_odom(self, msg):
        if self.sample_done:
            return
        lateral_distance = msg.pose.pose.position.y
        tangential_distance = msg.pose.pose.position.x
        if tangential_distance < self.min_x:
            self.min_x = tangential_distance
        #If we have changed signs and the distance is "big enough", stop
        if (lateral_distance * self.initial_y) < 0.0:
            if abs(lateral_distance) > ERROR_MAX:
                self.output_file.write("%f\n" % (tangential_distance - self.min_x))
                self.sample_done = True
        #If we got to the end of the path without overshooting, stop and record
        #how far we got
        if (5.0 - tangential_distance) <= ERROR_MAX:
            self.output_file.write("%f\n" % (tangential_distance - self.min_x))
            self.sample_done = True

    @staticmethod
    def make_test_path():
        p1 = ExecutePathGoal()
        p = PathSegment()
        p.header.frame_id = "odom"
        p.seg_type = PathSegment.LINE
        p.init_tan_angle.w = 1.0
        p.seg_length = 5.0
        p.max_speeds.linear.x = 0.5
        p.max_speeds.angular.z = 1.0
        p.accel_limit = 0.1
        p.decel_limit = 0.1
        p1.segments.append(p)
        return p1

if __name__ == "__main__":
    rospy.init_node("find_overshoot_distance_contour")
    #offsets = [3,-3]
    #angles = [0]
    offsets = numpy.linspace(-3, 3, 30) #Steps of 2.5cm are 240
    angles = numpy.linspace(-math.pi,math.pi, 36) #5 degree increments is 72
    with open("find_overshoot_contours.csv", "w") as f:
        fec = FindErrorContour(f)
        for offset in offsets:
            for angle in angles:
                fec.reset_harlie_state(0,offset, angle)
                while not fec.sample_done:
                    rospy.sleep(0.01)
        for angle in angles: #make sure to get 0 path offset
            fec.reset_harlie_state(0,0.0,angle)
            while not fec.sample_done:
                rospy.sleep(0.01)
