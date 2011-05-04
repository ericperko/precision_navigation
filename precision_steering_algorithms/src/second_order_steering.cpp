#include <ros/ros.h>
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <precision_steering/steering_base.h>
#include <precision_steering_algorithms/second_order_steering.h>
#include <precision_navigation_msgs/PathSegment.h>
#include <tf/tf.h>

PLUGINLIB_DECLARE_CLASS(precision_steering_algorithms, SecondOrderSteering, second_order_steering::SecondOrderSteering, precision_steering::SteeringBase)

  namespace second_order_steering {
    SecondOrderSteering::SecondOrderSteering() {}

    void SecondOrderSteering::initialize(ros::NodeHandle nh_) {
      double convergence_rate; //convergence rate in meters
      nh_.param("convergence_rate", convergence_rate, 2.0); 
      nh_.param("k_v", k_v, 1.0);

      //Computing gains based on convergence_rate parameter
      k_d = 1.0/pow(convergence_rate, 2.0);
      k_psi = 2.0/convergence_rate;
      ROS_INFO("Second Order Steering initialized");
    }

    void SecondOrderSteering::computeVelocities(const precision_navigation_msgs::DesiredState& des_state, const nav_msgs::Odometry& current_odom, geometry_msgs::Twist &command_vel) {
      const double pi = 3.1415926;
      if (des_state.seg_type == precision_navigation_msgs::PathSegment::SPIN_IN_PLACE) {
        ROS_WARN("Spin in place not implemented by second order steering right now");
        command_vel.linear.x = 0.0;
        command_vel.angular.z = 0.0;
        return;
      }
      double tanVec[2],nVec[2],dx_vec[2],d,v,omega;
      double deltaPsi;

      double psi_des = tf::getYaw(des_state.des_pose.orientation);
      double v_des = des_state.des_speed;
      double rho_des = des_state.des_rho;
      double x_des = des_state.des_pose.position.x;
      double y_des = des_state.des_pose.position.y;
      double x_PSO = current_odom.pose.pose.position.x;
      double y_PSO = current_odom.pose.pose.position.y;
      double psi_PSO = tf::getYaw(current_odom.pose.pose.orientation);

      tanVec[0]= cos(psi_des); //-sin(psi_des); // vector tangent to desired lineseg
      tanVec[1]= sin(psi_des); //cos(psi_des);

      nVec[0]= -tanVec[1];  // normal vector of desired (directed) lineseg--points "left" of heading
      nVec[1]=  tanVec[0];
      dx_vec[0] = x_des-x_PSO;
      dx_vec[1] = y_des-y_PSO; //position error
      double Lfollow = tanVec[0]*dx_vec[0]+tanVec[1]*dx_vec[1];

      //Check if the desired speed is 0
      //If so we will just set it there and be done with velocity computation
      if (fabs(v_des) < 1e-7) {
        ROS_DEBUG("v_des was %f, abs was %f, so setting it to 0", v_des, fabs(v_des));
        v = 0.0;
      } else {	
        v = v_des + k_v * Lfollow;
      }
      ROS_DEBUG("V_des was %f, v was %f, k_v was %f, Lfollow was %f", v_des, v, k_v, Lfollow);
      // d = -n'*dx_vec;
      d = -nVec[0]*dx_vec[0]-nVec[1]*dx_vec[1];
      deltaPsi = psi_PSO-psi_des;
      //std::cout << "psi_PSO " << psi_PSO << " " << "psi_des " << psi_des << std::endl;
      while (deltaPsi>pi)
        deltaPsi-=2*pi;
      while (deltaPsi< -pi)
        deltaPsi+=2*pi;
      double rho_cmd = -k_d*d -k_psi*deltaPsi + rho_des;
      //std::cout << "dPsi = " << deltaPsi  << " " << "d = " << d << std::endl;
      omega = v*rho_cmd;
      //std::cout << "omega = " << omega << std::endl;
      command_vel.linear.x = v;
      command_vel.angular.z = omega;
    }
  };
