#include <ros/ros.h>
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <precision_steering/steering_base.h>
#include <precision_steering_algorithms/phase_space_steering.h>
#include <precision_navigation_msgs/PathSegment.h>
#include <tf/tf.h>

PLUGINLIB_DECLARE_CLASS(precision_steering_algorithms, PhaseSpaceSteering, phase_space_steering::PhaseSpaceSteering, precision_steering::SteeringBase)

  namespace phase_space_steering {
    const double pi = 3.1415926;

    PhaseSpaceSteering::PhaseSpaceSteering() {}

    void PhaseSpaceSteering::initialize(ros::NodeHandle nh_) {
      nh_.param("k_psi", k_psi, 0.0); 
      nh_.param("k_v", k_v, 1.0);
      nh_.param("omega_cmd_sat", omega_cmd_sat, 2.0);
      nh_.param("phase_space_slope",phase_space_slope, -1.0);

      ROS_INFO("Phase Space Steering initialized");
    }

    double PhaseSpaceSteering::psiOfD(double d) {
      //map desired (relative) as a function of displacement from path
      //slope should be negative; large magnitude is more aggressive
      //sign of psi: positive if actual psi is CCW relative to path psi
      double psi = phase_space_slope * d;	
      if (psi > pi/2.) {
        psi = pi/2.;
      } else if (psi < -pi/2.) {
        psi = -pi/2.;
      }
      return psi;
    }

    void PhaseSpaceSteering::computeVelocities(const precision_navigation_msgs::DesiredState& des_state, const nav_msgs::Odometry& current_odom, geometry_msgs::Twist &command_vel) {
      double tanVec[2],nVec[2],dx_vec[2],d,v,omega;
      double deltaPsi;

      double psi_des = tf::getYaw(des_state.des_pose.orientation);
      double v_des = des_state.des_speed;
      double rho_des = des_state.des_rho;
      tanVec[0]= cos(psi_des); //-sin(psi_des); // vector tangent to desired lineseg
      tanVec[1]= sin(psi_des); //cos(psi_des); 

      nVec[0]= -tanVec[1];  // normal vector of desired (directed) lineseg--points "left" of heading
      nVec[1]=  tanVec[0];
      dx_vec[0] = des_state.des_pose.position.x - current_odom.pose.pose.position.x;
      dx_vec[1] = des_state.des_pose.position.y - current_odom.pose.pose.position.y; //position error
      double Lfollow = tanVec[0]*dx_vec[0]+tanVec[1]*dx_vec[1];

      //Check if the desired speed is 0
      //If so we will just set it there and be done with velocity computation
      //if (fabs(v_des) < 1e-7) {
      if (false) {
        ROS_DEBUG("v_des was %f, abs was %f, so setting it to 0", v_des, fabs(v_des));
        v = 0.0;
      } else {	
        v = v_des + k_v * Lfollow;
      }
      ROS_DEBUG("V_des was %f, v was %f, k_v was %f, Lfollow was %f", v_des, v, k_v, Lfollow);
      // d = -n'*dx_vec;
      d = -nVec[0]*dx_vec[0]-nVec[1]*dx_vec[1];
      ROS_DEBUG("d was %f", d);
      if (des_state.seg_type == precision_navigation_msgs::PathSegment::SPIN_IN_PLACE) {
        double deltaPsi = psi_des - tf::getYaw(current_odom.pose.pose.orientation);
        while (deltaPsi>pi) {
          deltaPsi-=2*pi;
        }
        while (deltaPsi< -pi) {
          deltaPsi+=2*pi;
        }
        ROS_DEBUG("DeltaPSI for spin in place was %f", deltaPsi);
        double omega_cmd = k_psi*deltaPsi + des_state.des_speed;
        command_vel.linear.x = 0.0;
        command_vel.angular.z = omega_cmd;
      } else {
        double deltaPsiPSOtoPath = tf::getYaw(current_odom.pose.pose.orientation) - psi_des;
        while (deltaPsiPSOtoPath > pi)
          deltaPsiPSOtoPath -= 2*pi;
        while (deltaPsiPSOtoPath < -pi)
          deltaPsiPSOtoPath += 2*pi;

        ROS_DEBUG("deltaPsiPSOtoPath was %f",deltaPsiPSOtoPath);
        //get the phase space mapping of the desired delta psi
        //for the current offset d
        double dPsiMappedToPath = psiOfD(d);
        ROS_DEBUG("dPsiMappedToPath was %f", dPsiMappedToPath);

        //compute difference between heading error and ideal
        //heading error
        deltaPsi = dPsiMappedToPath - deltaPsiPSOtoPath;
        ROS_DEBUG("deltaPsi was %f", deltaPsi);
        double omega_cmd = k_psi*deltaPsi + v*rho_des;
        ROS_DEBUG("omega_cmd was %f", omega_cmd);
        if(omega_cmd > omega_cmd_sat) {
          omega = omega_cmd_sat;
        } else if(omega_cmd < -omega_cmd_sat) {
          omega = -omega_cmd_sat;
        } else {
          omega = omega_cmd;
        }
        ROS_DEBUG("omega was %f", omega);
        command_vel.linear.x = v;
        command_vel.angular.z = omega;
      }
    }
    };
