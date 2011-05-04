#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <precision_navigation_msgs/DesiredState.h>
#include <pluginlib/class_loader.h>
#include <precision_steering/steering_base.h>
#include <string>

class PrecisionSteering {
	public:
		PrecisionSteering();
		virtual ~PrecisionSteering();
	private:
		//callback to put odometry information into the class 
		void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);	
		void desStateCallback(const precision_navigation_msgs::DesiredState::ConstPtr& desState);

		//last updated Odometry information
		nav_msgs::Odometry current_odom;
		precision_navigation_msgs::DesiredState curDesState;

		//Loop rate in Hz
		double loop_rate;

		//put gains here in whatever type you need (int, double, etc) (though more descriptive names than k would make me happier)
		bool firstCall;
		double x_init,y_init,psi_init;

		//ROS communcators
		ros::NodeHandle nh_;
		ros::NodeHandle priv_nh_;
		ros::Subscriber odom_sub_;
		ros::Subscriber desState_sub_;
		ros::Publisher twist_pub_;

};


PrecisionSteering::PrecisionSteering() : priv_nh_("~") {
	//Read parameters from the ROS parameter server, defaulting to value if the parameter is not there
	priv_nh_.param("loop_rate", loop_rate, 20.0);
	std::string steering_algo_name;
	priv_nh_.param("steering_algorithm", steering_algo_name, std::string("precision_steering_algorithms/SecondOrderSteering"));

	pluginlib::ClassLoader<precision_steering::SteeringBase> steering_loader("precision_steering", "precision_steering::SteeringBase");
	precision_steering::SteeringBase *steering_algo = NULL;

	try {
		steering_algo = steering_loader.createClassInstance(steering_algo_name);
		steering_algo->initialize(priv_nh_);

		ROS_INFO("Initialized the steering algorithm");
	} catch(pluginlib::PluginlibException& ex) {
		ROS_ERROR("Failed to create the steering algorithm due to a plugin loading error. Error: %s", ex.what());
		exit(0);
	}

	firstCall=true;
	//Subscribe to Odometry Topic
	odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odometry", 10, &PrecisionSteering::odomCallback, this); 
	desState_sub_ = nh_.subscribe<precision_navigation_msgs::DesiredState>("idealState", 10, &PrecisionSteering::desStateCallback, this);

	//Setup velocity publisher
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1); 

	//Setup the rate limiter
	ros::Rate rate(loop_rate);

	//temps
	geometry_msgs::Twist twist;

	//Don't shutdown till the node shuts down
	while(ros::ok()) {
		if (!firstCall) // do this only when PSO is warmed up
		{
			steering_algo->computeVelocities(curDesState, current_odom, twist);

			//Publish twist message
			twist_pub_.publish(twist);
		}
		//Make sure this node's ROS stuff gets to run if we are hogging CPU
		ros::spinOnce();
		
		//Sleep till it's time to go again
		rate.sleep();
		ROS_DEBUG("Cycle time was %.4f seconds", rate.cycleTime().toSec());
		if(rate.cycleTime() > ros::Duration(1 / loop_rate)) {
			ROS_WARN("Steering loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", loop_rate, rate.cycleTime().toSec());
		}
	}
}

void PrecisionSteering::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
	current_odom = *odom;
	if (firstCall)
	{
		firstCall=false;
		x_init=  current_odom.pose.pose.position.x;
		y_init = current_odom.pose.pose.position.y;
		psi_init = tf::getYaw(current_odom.pose.pose.orientation);
	}
}

void PrecisionSteering::desStateCallback(const precision_navigation_msgs::DesiredState::ConstPtr& desState)
{
	curDesState= *desState;
}

PrecisionSteering::~PrecisionSteering() {
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "precision_steering");
	PrecisionSteering steering;
}
