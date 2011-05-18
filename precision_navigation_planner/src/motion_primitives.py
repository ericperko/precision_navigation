#!/usr/bin/env python

import roslib; roslib.load_manifest("precision_navigation_planner")

import math
import copy
import operator

from tf import transformations as tf_math
from geometry_msgs.msg import Quaternion

from precision_navigation_msgs.msg import PathSegment

STATE_ATTRS_FOR_HASHING = ['seg_type',
        'seg_number',
        'seg_length',
        'ref_point.x',
        'ref_point.y',
        'ref_point.z',
        'init_tan_angle.x',
        'init_tan_angle.y',
        'init_tan_angle.z',
        'init_tan_angle.w',
        'curvature']

get_hash_value_tuple = operator.attrgetter(*STATE_ATTRS_FOR_HASHING)

class MotionPrimitive:
    def __init__(self, pathSegment, cost_multiplier = 1.0):
        self.action = pathSegment
        self.cost_multiplier = cost_multiplier

    def __eq__(self, other):
        return self.action == other.action

    def __hash__(self):
        return hash(get_hash_value_tuple(self.action))

    def __repr__(self):
        return repr(self.action)

    def __str__(self):
        return repr(self.action)

def calculate_heuristic(a, b):
    """
    Calculate the heuristic value from primitive a to primitive b

    Currently, this is just the euclidean distance from the end of a to the
    start of b.
    """
    end_a = get_endpoint(a)
    if end_a:
        return math.sqrt((b.action.ref_point.x - end_a.x)**2 + \
            (b.action.ref_point.y - end_a.y)**2)
    else:
        return 1e100000

def calculate_path_cost(a,b):
    """
    Calculate the cost to go from a to b
    """
    cost = 1 * b.cost_multiplier
    return cost

def get_endpoint(a):
    """
    Get the endpoint of motion primitive a and return it
    """
    seg = a.action
    end_point = copy.deepcopy(seg.ref_point)
    if seg.seg_type == PathSegment.SPIN_IN_PLACE:
        return end_point
    elif seg.seg_type == PathSegment.LINE:
        init_quat = [seg.init_tan_angle.x, seg.init_tan_angle.y, \
            seg.init_tan_angle.z, seg.init_tan_angle.w]
        init_tan_theta = tf_math.euler_from_quaternion(init_quat)[2]
        end_point.x = seg.ref_point.x + seg.seg_length * math.cos(init_tan_theta)
        end_point.y = seg.ref_point.y + seg.seg_length * math.sin(init_tan_theta)
        return end_point
    else:
        return None


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
    successors.extend(get_line_successors(end_point, end_theta))
    successors.extend(get_spin_in_place_successors(end_point, end_theta))
    return successors

def get_line_successors(end_point, end_theta):
    """
    Given the point & heading where the previous action ended, generate the
    possible line actions that could start at that point
    """
    max_distance = 5.0 #Meters that we will create lines up till
    num_lines = 50 #Number of lines to create up to the max distance
    distance_increment = max_distance / num_lines
    successors = []
    end_quat = Quaternion(*(tf_math.quaternion_from_euler(0, 0, end_theta,\
        'sxyz')))
    for i in range(1, num_lines+1):
        seg = PathSegment()
        seg.seg_type = PathSegment.LINE
        seg.ref_point = end_point
        seg.init_tan_angle = end_quat
        seg.seg_length = i * distance_increment
        successors.append(MotionPrimitive(seg))
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
    for i in range(0,num_angles):
        for curv in [-1.0, 1.0]:
            seg = PathSegment()
            seg.seg_type = PathSegment.SPIN_IN_PLACE
            seg.ref_point = end_point
            seg.init_tan_angle = end_quat
            seg.seg_length = i * angle_increment
            seg.curvature = curv
            successors.append(MotionPrimitive(seg))
    #Get a 180 deg spin
    seg = PathSegment()
    seg.seg_type = PathSegment.SPIN_IN_PLACE
    seg.ref_point = end_point
    seg.init_tan_angle = end_quat
    seg.seg_length = math.pi
    seg.curvature = curv
    successors.append(MotionPrimitive(seg))
    return successors


if __name__ == "__main__":
    print "This is a support library and not meant to be run directly"
