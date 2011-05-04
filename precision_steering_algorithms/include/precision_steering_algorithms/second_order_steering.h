#include <ros/ros.h>
#include <precision_steering/steering_base.h>

#ifndef PRECISION_STEERING_SECOND_ORDER_STEERING_H_
#define PRECISION_STEERING_SECOND_ORDER_STEERING_H_

namespace second_order_steering
{
	class SecondOrderSteering : public precision_steering::SteeringBase
	{
		public:
			SecondOrderSteering();
			void initialize(ros::NodeHandle nh_);
			void computeVelocities(const precision_navigation_msgs::DesiredState& des_state, const nav_msgs::Odometry& current_odom, geometry_msgs::Twist &command_vel);
		private:
			double k_psi;
			double k_v;
			double k_d;

	};
};
#endif
