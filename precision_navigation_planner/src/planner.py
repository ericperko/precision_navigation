#!/usr/bin/env python

import roslib
roslib.load_manifest("precision_navigation_planner")

import heapq
from tf import transformations as tf_math

import motion_primitive

class Planner:
    def __init__(self):
        self.use_arcs = False
        self.openset = []
        self.closedset = set()
        self.came_from = {}

    def reset_state(self):
        self.openset = []
        self.closedset = set()
        self.came_from = {}

    def make_plan(self, start, goal):
        """
        Start and goal are 3-tuples (x,y,theta)
        """
        start_prim = make_simple_state(start)
        goal_prim = make_simple_state(goal)

        start_prim.g_score = 0.0
        start_prim.h_score = motion_primitive.calculate_heuristic(start_prim, goal_prim)
        start_prim.f_score = start_prim.g_score + start_prim.h_score

        heapq.heappush(self.openset, (start_prim.f_score, start_prim))
        while self.openset:
            x = heapq.heappop(self.openset)
            if x.action == goal_prim.action:
                #This is the goal, so break out by returning the path
                self.rebuild_path(x)
            self.closedset.add(x)


   def rebuild_path(self, current_node):
        if current_node in self.came_from:
            path = rebuild_path(self.came_from[current_node])
            return path.append(current_node)
        else:
            return [current_node]

   @staticmethod
   def make_simple_state(state):
       """
       State is a 3-tuple (x,y,theta)
       This makes a simple spin in place for making either the start or goal
       state
       """
       seg = motion_primitive.PathSegment()
       seg.seg_type = motion_primitive.PathSegment.SPIN_IN_PLACE
       seg.ref_point.x = state[0]
       seg.ref_point.y = state[1]
       quat = Quaternion(*(tf_math.quaternion_from_euler(0, 0,\
               state[2],'sxyz')))
       seg.init_tan_angle = quat
       seg.curvature = 1.0
       return motion_primitive.MotionPrimitive(seg)



if __name__ == "__main__":
    pass
