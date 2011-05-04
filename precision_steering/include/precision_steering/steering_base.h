#include <ros/ros.h>
#include <precision_navigation_msgs/DesiredState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#ifndef PRECISION_STEERING_STEERING_BASE_H_
#define PRECISION_STEERING_STEERING_BASE_H_

namespace precision_steering
{
	class SteeringBase
	{
		public:
			virtual void initialize(ros::NodeHandle nh_) = 0;
			/*
			 * x,y in meters in ROS frame
			 * psi in rads in ROS frame, 0 points to true north
			 * v in meters/sec forwards velocity
			 * omega in rads/sec
			 */
			virtual void computeVelocities(const precision_navigation_msgs::DesiredState& des_state, const nav_msgs::Odometry& current_odom, geometry_msgs::Twist &command_vel) = 0;
			virtual ~SteeringBase(){}

		protected:
			SteeringBase(){}
	};
};
#endif
