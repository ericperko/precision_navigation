#include <ros/ros.h>
#include <precision_steering/steering_base.h>

#ifndef PRECISION_STEERING_PHASE_SPACE_STEERING_H_
#define PRECISION_STEERING_PHASE_SPACE_STEERING_H_

namespace phase_space_steering
{
	class PhaseSpaceSteering : public precision_steering::SteeringBase
	{
		public:
			PhaseSpaceSteering();
			void initialize(ros::NodeHandle nh_);
			void computeVelocities(const precision_navigation_msgs::DesiredState& des_state, const nav_msgs::Odometry& current_odom, geometry_msgs::Twist &command_vel);
		private:
			double k_psi;
			double k_v;
			double omega_cmd_sat;
			double phase_space_slope;
			double psiOfD(double d);
	};
};
#endif
