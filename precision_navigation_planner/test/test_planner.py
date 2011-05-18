#!/usr/bin/env python
import roslib
roslib.load_manifest("precision_navigation_planner")

import unittest
import math

import planner
import motion_primitives

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
        self.assertEqual(plan, correct_plan)

    def test_start_end_are_same(self):
        start = (0,0,0)
        end = (0,0,0)
        plan_using_start = [planner.Planner.make_simple_state(start)]
        plan_using_end = [planner.Planner.make_simple_state(end)]
        plan = self.planner.make_plan(start, end)
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
        self.assertEqual(plan, correct_plan)

    def test_long_straight_path(self):
        start = (-10,0,0)
        end = (10,0,0)
        plan = self.planner.make_plan(start, end)
        self.assertEqual(len(plan), 6, "Number of segments != 6")

    @unittest.skip("Not supported yet")
    def test_complicated_path(self):
        start = (0,0,0)
        end = (15.0, 27.0, math.pi/2.)
        plan = self.planner.make_plan(start, end)
        self.assertIsNotNone(plan)

if __name__ == "__main__":
    import rostest
    rostest.unitrun('precision_navigation_planner', \
            'test_prec_nav_planner_no_map', TestPlannerNoMap)
