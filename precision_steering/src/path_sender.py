#! /usr/bin/env python
import roslib
roslib.load_manifest('precision_steering')
import rospy

import actionlib

from std_msgs.msg import String
from precision_navigation_msgs.msg import ExecutePathAction
from precision_navigation_msgs.msg import ExecutePathGoal
from precision_navigation_msgs.msg import PathSegment
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from tf import transformations as tf_math

def makeDummyPaths():
	paths = {}
	p1 = ExecutePathGoal()
	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x = 0.0436
	p.ref_point.y = 2.18822
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(2.42426, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 2.3202
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p1.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x = -1.6972
	p.ref_point.y = 3.70692
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(2.42426, (0,0,1))))
	p.curvature = -100.0
	p.seg_length = 0.0157
	p.max_speeds.linear.x = 0.005
	p.accel_limit = 0.05
	p.decel_limit = 0.05
	p1.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x = -1.7532
	p.ref_point.y = 3.847
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.82720, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 13.0
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p1.segments.append(p)
	paths['hallway'] = p1

	# start path to elevator
	p2 = ExecutePathGoal()
	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x = 7.0274
	p.ref_point.y = 13.4659
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.71210, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 1.7752
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.2
	p.decel_limit = 0.2
	p2.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x = 7.6093
	p.ref_point.y = 15.5085
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.7121, (0,0,1))))
	p.curvature = 0.8576
	p.seg_length = 1.8316
	p.max_speeds.linear.x = 0.4
	p.accel_limit = 0.2
	p.decel_limit = 0.2
	p2.segments.append(p)
	paths['elevator'] = p2

	# start go to bathroom
	p3 = ExecutePathGoal()
	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x = 8.4844
	p.ref_point.y = 16.2639
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(2.2829, (0,0,1))))
	p.curvature = 100.0
	p.seg_length = 0.0157
	p.max_speeds.linear.x = 0.008
	p.accel_limit = 0.02
	p.decel_limit = 0.02
	p3.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x = 8.4778
	p.ref_point.y = 16.2715
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-2.2326, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 4.3851
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p3.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x = 5.2784
	p.ref_point.y = 13.2053
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-2.2326, (0,0,1))))
	p.curvature =  -1.5628
	p.seg_length = 1.0865
	p.max_speeds.linear.x = 0.3
	p.accel_limit = 0.2
	p.decel_limit = 0.2
	p3.segments.append(p)


	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x =  4.8243
	p.ref_point.y = 12.7544
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(2.3526, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 5.6957
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.2
	p.decel_limit = 0.2
	p3.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x = 0.8112
	p.ref_point.y = 16.7962
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(2.35260, (0,0,1))))
	p.curvature = -100.0
	p.seg_length = 0.0157
	p.max_speeds.linear.x = 0.008
	p.accel_limit = 0.02
	p.decel_limit = 0.02
	p3.segments.append(p)
	paths['bathroom'] = p3

	# start path from bathroom to vending machines
	p4 = ExecutePathGoal()
	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x =  0.8042
	p.ref_point.y = 16.8174
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.7818, (0,0,1))))
	p.curvature = 100.0
	p.seg_length = 0.0157
	p.max_speeds.linear.x = 0.003
	p.accel_limit = 0.01
	p.decel_limit = 0.01
	p4.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x =  0.8113
	p.ref_point.y = 16.8244
	p.init_tan_angle =Quaternion(*(tf_math.quaternion_about_axis(2.3606, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 4.7258
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p4.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x = -2.0264
	p.ref_point.y = 20.6746
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(2.3606, (0,0,1))))
	p.curvature =  -1.3571
	p.seg_length = 1.1600
	p.max_speeds.linear.x = 0.4
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p4.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x =  -2.5480
	p.ref_point.y = 21.1951
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.7864, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 2.7161
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p4.segments.append(p)
	paths['vending'] = p4

#start return from vending to lab door
#spin from pt 12 to pt13
	p5 = ExecutePathGoal()
	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x =  -0.6364
	p.ref_point.y = 23.1246
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.7864, (0,0,1))))
	p.curvature = -100.0
	p.seg_length =  0.0314
	p.max_speeds.linear.x = 0.003
	p.accel_limit = 0.01
	p.decel_limit = 0.01
	p5.segments.append(p)
#seg from p13 to p14
	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x =  -0.6293
	p.ref_point.y = 23.1175
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-2.3222, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 2.9072
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p5.segments.append(p)

#turn from p14 to p15
	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x = -1.8012
	p.ref_point.y = 20.2338
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-2.3222, (0,0,1))))
	p.curvature = 0.8990
	p.seg_length = 1.5431
	p.max_speeds.linear.x = 0.4
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p5.segments.append(p)

#lineseg from p15 to p16
	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x = -2.5908
	p.ref_point.y = 19.4503
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-0.7892, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 11.4321
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p5.segments.append(p)
	paths['lab'] = p5

# start path from crawford conference room to women's restroom
	p6 = ExecutePathGoal()
	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x = 0.0
	p.ref_point.y = 0.0
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.0000, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 5.2 
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p6.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x =  5.2
	p.ref_point.y = 0.0
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.0000, (0,0,1))))
	p.curvature = -100.0
	p.seg_length = 0.0153
	p.max_speeds.linear.x = 0.005
	p.accel_limit = 0.05
	p.decel_limit = 0.05
	p6.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x =  5.2
	p.ref_point.y = 0.0
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-1.5300, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 11.3 
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p6.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x =  5.66
	p.ref_point.y = -11.291
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-1.5300, (0,0,1))))
	p.curvature = -100.0
	p.seg_length = 0.0153
	p.max_speeds.linear.x = 0.005
	p.accel_limit = 0.05
	p.decel_limit = 0.05
	p6.segments.append(p)
	paths['womens_room'] = p6

# start path from crawford women's restroom back to conference room 
	p7 = ExecutePathGoal()
	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x =  5.66
	p.ref_point.y = -11.291
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-3.0600, (0,0,1))))
	p.curvature = -100.0
	p.seg_length = 0.0157 
	p.max_speeds.linear.x = 0.005
	p.accel_limit = 0.05
	p.decel_limit = 0.05
	p7.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x =  5.66
	p.ref_point.y = -11.291
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-4.6300, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 11.4
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p7.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x = 4.26
	p.ref_point.y = 0.07
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-4.6300, (0,0,1))))
	p.curvature = 100.0
	p.seg_length = 0.0157
	p.max_speeds.linear.x = 0.005
	p.accel_limit = 0.05
	p.decel_limit = 0.05
	p7.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.LINE
	p.ref_point.x =  4.26
	p.ref_point.y = 0.07
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-3.0600, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 2.9
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p7.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "map"
	p.seg_type = PathSegment.ARC
	p.ref_point.x =  1.3696
	p.ref_point.y = -0.166
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(-3.0600, (0,0,1))))
	p.curvature = -100.0
	p.seg_length = 0.016
	p.max_speeds.linear.x = 0.005
	p.accel_limit = 0.05
	p.decel_limit = 0.05
	p7.segments.append(p)
	paths['conference_room'] = p7

# a test path for the lab
	p8 = ExecutePathGoal()
	p = PathSegment()
	p.header.frame_id = "odom"
	p.seg_type = PathSegment.LINE
	p.ref_point.x = 0.0
	p.ref_point.y = 0.0
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.00000, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 2.0
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p8.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "odom"
	p.seg_type = PathSegment.ARC
	p.ref_point.x =  2.0
	p.ref_point.y = 1.0
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.00000, (0,0,1))))
	p.curvature = 1.0
	p.seg_length = 3.1416
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p8.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "odom"
	p.seg_type = PathSegment.LINE
	p.ref_point.x =  2.0
	p.ref_point.y = 2.0
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(3.1416, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 1.0
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p8.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "odom"
	p.seg_type = PathSegment.ARC
	p.ref_point.x =  1.0
	p.ref_point.y = 2.01
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(3.14160, (0,0,1))))
	p.curvature = -100.0
	p.seg_length = 0.0157
	p.max_speeds.linear.x = 0.01
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p8.segments.append(p)

	p = PathSegment()
	p.header.frame_id = "odom"
	p.seg_type = PathSegment.LINE
	p.ref_point.x =  0.99
	p.ref_point.y = 2.01
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(1.57080, (0,0,1))))
	p.curvature = 0.0
	p.seg_length = 1.0
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p8.segments.append(p)

    	p = PathSegment()
	p.header.frame_id = "odom"
	p.seg_type = PathSegment.ARC
	p.ref_point.x =  1.49
	p.ref_point.y = 3.01
	p.init_tan_angle = Quaternion(*( tf_math.quaternion_about_axis(1.5708, (0,0,1))))
	p.curvature = -2.0
	p.seg_length = 0.7854
	p.max_speeds.linear.x = 0.5
	p.accel_limit = 0.1
	p.decel_limit = 0.1
	p8.segments.append(p)
	paths['lab_test'] = p8

        p9 = ExecutePathGoal()
        p = PathSegment()
        p.header.frame_id = "odom"
	p.seg_type = PathSegment.SPIN_IN_PLACE
	p.ref_point.x =  0.0
	p.ref_point.y = 0.0
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.00000, (0,0,1))))
	p.curvature = 1.0
	p.seg_length = 3.14159
	p.max_speeds.linear.x = 0.0
        p.max_speeds.angular.z = 1.0
	p.accel_limit = 0.1
	p.decel_limit = 0.1
        p9.segments.append(p)

        p = PathSegment()
        p.header.frame_id = "odom"
	p.seg_type = PathSegment.SPIN_IN_PLACE
	p.ref_point.x =  0.0
	p.ref_point.y = 0.0
	p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(3.14159, (0,0,1))))
	p.curvature = -1.0
	p.seg_length = 3.14159
	p.max_speeds.linear.x = 0.0
        p.max_speeds.angular.z = 1.0
	p.accel_limit = 0.1
	p.decel_limit = 0.1
        p9.segments.append(p)
        paths['spin_in_place_test'] = p9

        p10 = ExecutePathGoal()
        p = PathSegment()
        p.header.frame_id = "odom"
        p.seg_type = PathSegment.LINE
        p.ref_point.x = 0.0
        p.ref_point.y = 0.0
        p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.00000, (0,0,1))))
        p.curvature = 0.0
        p.seg_length = 10.0
        p.max_speeds.linear.x = 0.5
        p.max_speeds.angular.z = 0.0
        p.accel_limit = 0.1
        p.decel_limit = 0.1
        p10.segments.append(p)

        p = PathSegment()
        p.header.frame_id = "odom"
        p.seg_type = PathSegment.SPIN_IN_PLACE
        p.ref_point.x = 10.0
        p.ref_point.y = 0.0
        p.init_tan_angle = Quaternion(*(tf_math.quaternion_about_axis(0.00000, (0,0,1))))
        p.curvature = 1.0
        p.seg_length = 3.14
        p.max_speeds.linear.x = 0.0
        p.max_speeds.angular.z = 0.5
        p.accel_limit = 0.1
        p.decel_limit = 0.1
        p10.segments.append(p)
        paths['straight_odom_test'] = p10

	return paths

class PathSender:
	def __init__(self):
		self.paths = makeDummyPaths()
		self.action_client = \
                        actionlib.SimpleActionClient("execute_path", ExecutePathAction)
		self.command_sub = rospy.Subscriber('chatter', String, self.handle_chatter)
                while not rospy.is_shutdown() and not self.action_client.wait_for_server(rospy.Duration(5.0)):
                    rospy.logwarn("Unable to connect to execute_path server. Retrying")
                rospy.loginfo("Execute path server found. Ready for commands")

	def handle_chatter(self, msg):
		if 'open' in msg.data or 'close' in msg.data:
			pass
		else:
		    path_goal = self.paths.get(msg.data, None)
                    if path_goal:
                        self.action_client.send_goal(path_goal)
                        rospy.loginfo("Sent %s" % (msg.data))
                    else:
                        self.action_client.cancel_goal()
                        rospy.loginfo("Stop received")

if __name__ == "__main__":
	rospy.init_node("path_sender")
	ps = PathSender()
	rospy.spin()
