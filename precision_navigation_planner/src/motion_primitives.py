#!/usr/bin/env python

import roslib; roslib.load_manifest("precision_navigation_planner")

import math
import copy
from tf import transformations as tf_math
from geometry_msgs.msg import Quaternion

from precision_navigation_msgs.msg import PathSegment

class MotionPrimitive:
    def __init__(self, pathSegment):
        self.action = pathSegment
        self.g_score = None
        self.h_score = None
        self.f_score = None
        self.cost_multiplier = 1.0

def calculate_heuristic(a, b):
    """
    Calculate the heuristic value from primitive a to primitive b

    Currently, this is just the euclidean distance from the start of a to the
    start of b.
    """
    return math.sqrt((b.action.ref_point.x - a.action.ref_point.x)**2 + \
            (b.action.ref_point.y - a.action.ref_point.y)**2)

def calculate_path_cost(a,b):
    """
    Calculate the cost to go from a to b
    """
    cost = 1 * b.cost_multiplier
    return cost

def get_successors(a):
    """
    Get the motion primitives that could come after this one
    """
    successors = []
    seg = a.action
    init_quat = [seg.init_tan_angle.x, seg.init_tan_angle.y, \
            seg.init_tan_angle.z, seg.init_tan_angle.w]
    init_tan_theta = tf_math.euler_from_quaternion(init_quat)[2]
    end_point = copy.deepcopy(seg.ref_point)
    end_theta = init_tan_theta
    if seg.seg_type == PathSegment.SPIN_IN_PLACE:
        end_theta = init_tan_theta + seg.curvature * seg.seg_length
    elif seg.seg_type == PathSegment.LINE:
        end_point.x = seg.ref_point.x + seg.seg_length * math.cos(init_tan_theta)
        end_point.y = seg.ref_point.y + seg.seg_length * math.sin(init_tan_theta)
    else:
        print "Get successors not support for seg_type %s" % (seg.seg_type)
        return []
    #successors.extend(get_line_successors(end_point, end_theta))
    successors.extend(get_spin_in_place_successors(end_point, end_theta))
    return successors

def get_spin_in_place_successors(end_point, end_theta):
    """
    Given the point & heading where the previous action ended, generate the
    possible spin in places that could happen at that point
    """
    num_angles = 16 # How many increments to divide the 0 to pi range into.
    # Angles will be mirrored to the 0 to -pi range
    successors = []
    angle_increment = math.pi / num_angles
    end_quat = Quaternion(*(tf_math.quaternion_from_euler(0, 0, end_theta,\
        'sxyz')))
    for i in range(1,num_angles):
        for curv in [-1.0, 1.0]:
            seg = PathSegment()
            seg.seg_type = PathSegment.SPIN_IN_PLACE
            seg.ref_point = end_point
            seg.init_tan_angle = end_quat
            seg.seg_length = i * angle_increment
            seg.curvature = curv
            successors.append(seg)
    seg = PathSegment()
    seg.seg_type = PathSegment.SPIN_IN_PLACE
    seg.ref_point = end_point
    seg.init_tan_angle = end_quat
    seg.seg_length = math.pi
    seg.curvature = curv
    successors.append(seg)
    return successors


if __name__ == "__main__":
    print "This is a support library and not meant to be run directly"
