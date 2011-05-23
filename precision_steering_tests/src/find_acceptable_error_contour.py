#!/usr/bin/env python

import roslib; roslib.load_manifest("precision_steering_tests")

import rospy
from gazebo.srv import SetModelState
from gazebo.msg import ModelState
from geometry_msgs.msg import Pose

reset_pub = None

def reset_harlie_state(gazebo_model_state_setter_service):
    reset_state = ModelState()
    reset_state.model_name = "harlie"
    reset_state.pose.orientation.w = 1.0
    i = 0
    while not rospy.is_shutdown():
        reset_pub.publish(reset_state.pose)
        rospy.sleep(0.1)
        i+=1
        if i > 10:
            break
    gazebo_model_state_setter_service(reset_state)

if __name__ == "__main__":
    model_state = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
    reset_pub = rospy.Publisher("reset_pose", Pose)
    rospy.init_node("find_acceptable_error_contour")
    reset_harlie_state(model_state)
