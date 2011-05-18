#!/usr/bin/env python

import roslib
roslib.load_manifest("precision_navigation_planner")

import time
from tf import transformations as tf_math

import motion_primitives
from pq import PriorityQueue

class Planner:
    def __init__(self):
        self.use_arcs = False
        self.openset = PriorityQueue()
        self.closedset = []
        self.came_from = {}
        self.g_score = {}
        self.h_score = {}
        self.f_score = {}
        self.max_duration = 60. * 5

    def reset_state(self):
        self.openset = PriorityQueue()
        self.closedset = []
        self.came_from = {}
        self.g_score = {}
        self.h_score = {}
        self.f_score = {}


    def make_plan(self, start, goal):
        """
        Start and goal are 3-tuples (x,y,theta)
        """
        start_time = time.time()

        start_prim = Planner.make_simple_state(start)
        goal_prim = Planner.make_simple_state(goal)

        self.g_score[start_prim] = 0.0
        self.h_score[start_prim] = motion_primitives.calculate_heuristic(start_prim, goal_prim)
        self.f_score[start_prim] = self.g_score[start_prim] + self.h_score[start_prim]

        self.openset.add_state(self.f_score[start_prim], start_prim)
        while not self.openset.empty():
            x = self.openset.get_top_priority()
            if x == goal_prim:
                #This is the goal, so break out by returning the path
                return self.rebuild_path(x)
            if (time.time() - start_time) > self.max_duration:
                print "Couldn't find path in %f seconds" % (self.max_duration)
                return None
            self.closedset.append(x)
            succs = motion_primitives.get_successors(x)
            for y in succs:
                if y in self.closedset:
                    continue
                tentative_g_score = self.g_score[x] + \
                        motion_primitives.calculate_path_cost(x,y)

                if not self.openset.contains_state(y):
                    self.came_from[y] = x
                    self.g_score[y] = tentative_g_score
                    self.h_score[y] = motion_primitives.calculate_heuristic(y, \
                            goal_prim)
                    self.f_score[y] = self.g_score[y] + self.h_score[y]
                    self.openset.add_state(self.f_score[y], y)
                elif tentative_g_score < self.g_score[y]:
                    self.came_from[y] = x
                    self.g_score[y] = tentative_g_score
                    self.h_score[y] = motion_primitives.calculate_heuristic(y, \
                            goal_prim)
                    self.f_score[y] = self.g_score[y] + self.h_score[y]
                    self.openset.reprioritize(self.f_score[y], y)
                else:
                    pass
        return None

    def rebuild_path(self, current_node):
        if current_node in self.came_from:
            path = self.rebuild_path(self.came_from[current_node])
            path.append(current_node)
            return path
        else:
            return [current_node]

    @staticmethod
    def make_simple_state(state):
        """
        State is a 3-tuple (x,y,theta)
        This makes a simple spin in place for making either the start or goal
        state
        """
        seg = motion_primitives.PathSegment()
        seg.seg_type = motion_primitives.PathSegment.SPIN_IN_PLACE
        seg.ref_point.x = state[0]
        seg.ref_point.y = state[1]
        quat = motion_primitives.Quaternion(*(tf_math.quaternion_from_euler(0, 0,\
               state[2],'sxyz')))
        seg.init_tan_angle = quat
        seg.curvature = 1.0
        return motion_primitives.MotionPrimitive(seg)

if __name__ == "__main__":
    pass
