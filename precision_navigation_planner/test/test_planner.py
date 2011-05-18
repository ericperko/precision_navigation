#!/usr/bin/env python
import roslib
roslib.load_manifest("precision_navigation_planner")

import unittest
import inspect
import math

import planner
import motion_primitives

def make_multitest_lots_of_ends(num_steps):
    x_high = 50.
    x_low = -50.
    y_high = 50.
    y_low = -50.
    theta_low = 0.
    theta_high = math.pi

    x_inc = (x_high - x_low)/num_steps
    y_inc = (y_high - y_low)/num_steps
    theta_inc = (theta_high - theta_low)/num_steps

    ends = []
    for i in range(0,num_steps+1):
        x = x_low + x_inc*i
        y = y_low + y_inc*i
        theta = theta_low + theta_inc*i

        end = (x,y,theta)
        ends.append(end)

    return ends

class TestPlannerNoMap(unittest.TestCase):
    def setUp(self):
        self.planner = planner.Planner()

    def test_simple_path_one_line(self):
        start = (0,0,0)
        end = (1,0,0)
        line_seg = motion_primitives.PathSegment()
        line_seg.seg_type = motion_primitives.PathSegment.LINE
        line_seg.seg_length = 1.0
        line_seg.init_tan_angle.w = 1.0
        correct_plan = [planner.Planner.make_simple_state(start),
                motion_primitives.MotionPrimitive(line_seg),
                planner.Planner.make_simple_state(end)]
        plan = self.planner.make_plan(start, end)
        self.assertIsNotNone(plan)
        self.assertEqual(plan, correct_plan)

    def test_start_end_are_same(self):
        start = (0,0,0)
        end = (0,0,0)
        plan_using_start = [planner.Planner.make_simple_state(start)]
        plan_using_end = [planner.Planner.make_simple_state(end)]
        plan = self.planner.make_plan(start, end)
        self.assertIsNotNone(plan)
        self.assertEqual(plan, plan_using_start, "Plan != plan using start state")
        self.assertEqual(plan, plan_using_end, "Plan != plan using end state")

    def test_simple_spin_in_place(self):
        start = (0,0,0)
        end = (0,0,(math.pi / 2.0))
        spin_seg = motion_primitives.PathSegment()
        spin_seg.seg_type = motion_primitives.PathSegment.SPIN_IN_PLACE
        spin_seg.seg_length = math.pi / 2.0
        spin_seg.curvature = 1.0
        spin_seg.init_tan_angle.w = 1.0
        correct_plan = [planner.Planner.make_simple_state(start),
                motion_primitives.MotionPrimitive(spin_seg),
                planner.Planner.make_simple_state(end)]
        plan = self.planner.make_plan(start, end)
        self.assertIsNotNone(plan)
        self.assertEqual(plan, correct_plan)

    def test_long_straight_path(self):
        start = (-10,0,0)
        end = (10,0,0)
        line_seg = motion_primitives.PathSegment()
        line_seg.seg_type = motion_primitives.PathSegment.LINE
        line_seg.ref_point.x = -10.
        line_seg.seg_length = 20.
        line_seg.init_tan_angle.w = 1.0
        correct_plan = [planner.Planner.make_simple_state(start),
                motion_primitives.MotionPrimitive(line_seg),
                planner.Planner.make_simple_state(end)]
        plan = self.planner.make_plan(start, end)
        self.assertIsNotNone(plan)
        self.assertEqual(plan, correct_plan)

    def test_spin_to_odd_angle(self):
        odd_angle = 1.5565
        start = (0,0,0)
        end = (0,0, odd_angle)
        spin_seg = motion_primitives.PathSegment()
        spin_seg.seg_type = motion_primitives.PathSegment.SPIN_IN_PLACE
        spin_seg.seg_length = odd_angle
        spin_seg.curvature = 1.0
        spin_seg.init_tan_angle.w = 1.0
        correct_plan = [planner.Planner.make_simple_state(start),
                motion_primitives.MotionPrimitive(spin_seg),
                planner.Planner.make_simple_state(end)]
        plan = self.planner.make_plan(start, end)
        self.assertIsNotNone(plan)
        self.assertEqual(plan, correct_plan)

    def test_complicated_path(self):
        start = (0,0,0)
        end = (15.0, 27.0, math.pi/2.)
        plan = self.planner.make_plan(start, end)
        self.assertIsNotNone(plan)

    multitest_complicated_paths_start_at_origin_values = make_multitest_lots_of_ends(100)
    def multitest_complicated_paths_start_at_origin(self,end):
       start = (0,0,0)
       plan = self.planner.make_plan(start, end)
       self.assertIsNotNone(plan, "Failed from origin to %s" % (str(end)))

    multitest_complicated_paths_end_at_origin_values = make_multitest_lots_of_ends(100)
    def multitest_complicated_paths_end_at_origin(self,start):
       end = (0,0,0)
       plan = self.planner.make_plan(start, end)
       self.assertIsNotNone(plan, "Failed from %s to origin" % (str(start)))

def add_test_cases(cls):
    values = {}
    functions = {}
    # Find all the 'multitest*' functions and
    # matching list of test values.
    for key, value in inspect.getmembers(cls):
        if key.startswith("multitest"):
            if key.endswith("_values"):
                values[key[:-7]] = value
            else:
                functions[key] = value

    # Put them together to make a list of new test functions.
    # One test function for each value
    for key in functions:
        if key in values:
            function = functions[key]
            for i, value in enumerate(values[key]):
                def test_function(self, function=function, value=value):
                    function(self, value)
                name ="test%s_%d" % (key[9:], i+1)
                test_function.__name__ = name
                setattr(cls, name, test_function)

add_test_cases(TestPlannerNoMap)

if __name__ == "__main__":
    import rostest
    rostest.unitrun('precision_navigation_planner', \
            'test_prec_nav_planner_no_map', TestPlannerNoMap)
