#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <precision_navigation_msgs/DesiredState.h>
#include <precision_navigation_msgs/PathSegment.h>
#include <precision_navigation_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>
#include <precision_navigation_msgs/ExecutePathAction.h>
#include <octocostmap/costmap_3d.h>
#include <vector>
#include <cmath>
#include <algorithm>

class IdealStateGenerator {
  public:
    IdealStateGenerator();
  private:
    bool computeState(precision_navigation_msgs::DesiredState& new_des_state);
    //Handle new path
    void newPathCallback();
    //Cancel current path
    void preemptPathCallback();
    precision_navigation_msgs::DesiredState makeHaltState(bool command_last_state);
    void computeStateLoop(const ros::TimerEvent& event);
    bool checkCollisions(bool checkEntireVolume, const precision_navigation_msgs::DesiredState& des_state);

    //Loop rate in Hz
    double loop_rate_;
    double dt_;

    double seg_length_done_;
    uint32_t seg_number_;

    //Whether or not to do collision checking
    bool check_for_collisions_;

    //Current path to be working on
    std::vector<precision_navigation_msgs::PathSegment> path_;

    //The last desired state we output	
    precision_navigation_msgs::DesiredState desiredState_;

    //ROS communcators
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::Publisher ideal_state_pub_;
    ros::Publisher ideal_pose_marker_pub_;
    ros::Subscriber path_sub_;
    tf::TransformListener tf_listener_;	
    geometry_msgs::PoseStamped temp_pose_in_, temp_pose_out_;
    boost::shared_ptr<octocostmap::Costmap3D> costmap_;
    ros::Timer compute_loop_timer_;
    std::string action_name_;
    actionlib::SimpleActionServer<precision_navigation_msgs::ExecutePathAction> as_;
    precision_navigation_msgs::ExecutePathFeedback feedback_;
};

IdealStateGenerator::IdealStateGenerator(): 
  check_for_collisions_(true),
  priv_nh_("~"),
  action_name_("execute_path"), 
  as_(nh_, action_name_, false)
{
  //Setup the ideal state pub
  ideal_state_pub_= nh_.advertise<precision_navigation_msgs::DesiredState>("idealState",1);   
  ideal_pose_marker_pub_= nh_.advertise<geometry_msgs::PoseStamped>("ideal_pose",1);   
  priv_nh_.param("loop_rate",loop_rate_,20.0); // default 20Hz
  dt_ = 1.0/loop_rate_;
  priv_nh_.param("check_for_collisions", check_for_collisions_, true);
  if (check_for_collisions_) {
    costmap_ = boost::shared_ptr<octocostmap::Costmap3D>(new octocostmap::Costmap3D("octocostmap", tf_listener_));
  } else {
    ROS_WARN("Collision checking disabled by parameter. Robot may be unsafe");
  }
    //Initialze private class variables
  seg_number_ = 0;
  seg_length_done_ = 0.0;

  while (!tf_listener_.waitForTransform("odom", "map", ros::Time::now(), ros::Duration(10)) && ros::ok()) {
    ROS_WARN("Waiting for odom to map transform");
    ros::spinOnce();
  }
  while (!tf_listener_.waitForTransform("odom", "base_link", ros::Time::now(), ros::Duration(10)) && ros::ok()) {
    ROS_WARN("Waiting for odom to base_link transform");
    ros::spinOnce();
  }
  desiredState_ = makeHaltState(false);

  //Setup the loop timer
  compute_loop_timer_ = nh_.createTimer(ros::Duration(dt_), boost::bind(&IdealStateGenerator::computeStateLoop, this, _1));

  as_.registerGoalCallback(boost::bind(&IdealStateGenerator::newPathCallback, this));
  as_.registerPreemptCallback(boost::bind(&IdealStateGenerator::preemptPathCallback, this));
  as_.start();
}

//We want to take the current location of the base and set that as the desired state with 0 velocity and rho. 
precision_navigation_msgs::DesiredState IdealStateGenerator::makeHaltState(bool command_last_state) {
  precision_navigation_msgs::DesiredState halt_state;
  //If we should command our current position, command_last_state will be false. Otherwise, command the last desired state
  if (command_last_state) {
    halt_state = desiredState_;
    halt_state.des_speed = 0.0;
  } else {
    //Convert into the odometry frame from whatever frame the path segments are in
    temp_pose_in_.header.frame_id = "base_link";
    temp_pose_in_.pose.position.x = 0.0;
    temp_pose_in_.pose.position.y = 0.0;
    temp_pose_in_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    ros::Time current_transform = ros::Time::now();
    tf_listener_.getLatestCommonTime(temp_pose_in_.header.frame_id, "odom", current_transform, NULL);
    temp_pose_in_.header.stamp = current_transform;
    tf_listener_.transformPose("odom", temp_pose_in_, temp_pose_out_);

    halt_state.header.frame_id = "odom";
    halt_state.seg_type = precision_navigation_msgs::PathSegment::SPIN_IN_PLACE;
    halt_state.header.stamp = ros::Time::now();
    halt_state.des_pose = temp_pose_out_.pose;
    halt_state.des_speed = 0.0;
    halt_state.des_rho = 0.0;
  }
  return halt_state;
}

void IdealStateGenerator::computeStateLoop(const ros::TimerEvent& event) {
  ROS_DEBUG("Last callback took %f seconds", event.profile.last_duration.toSec());

  precision_navigation_msgs::DesiredState new_desired_state;
  new_desired_state.header.frame_id = "odom";
  new_desired_state.header.stamp = ros::Time::now();
  // if we actually have a path to execute, try to execute it, otherwise just output our current position as the goal
  if (as_.isActive()) {
    ROS_DEBUG("We have an active goal. Compute state");
    if (computeState(new_desired_state)) {
      ROS_DEBUG("State computation failed. Command current position");
      new_desired_state = makeHaltState(false);
    }
    if(!checkCollisions(false, new_desired_state)) {
      ROS_DEBUG("No collision detected. Passing on current desired state");
      desiredState_ = new_desired_state;
    } else {
      ROS_DEBUG("Collision detected. Commanding current position");
      desiredState_ = makeHaltState(false); 
    }
  } else {
    ROS_DEBUG("No active goal, so sending last desired state");
    desiredState_ = makeHaltState(false);
  }

  //Publish desired state
  ideal_state_pub_.publish(desiredState_);
  geometry_msgs::PoseStamped des_pose;
  des_pose.header = desiredState_.header;
  des_pose.pose = desiredState_.des_pose;
  ideal_pose_marker_pub_.publish(des_pose);
}

bool IdealStateGenerator::checkCollisions(bool checkEntireVolume, const precision_navigation_msgs::DesiredState& des_state) {
  if (check_for_collisions_) {
    geometry_msgs::PoseStamped origin, origin_des_frame;
    origin_des_frame.header = des_state.header;
    /*origin_des_frame.header.frame_id = std::string("base_link");
      origin_des_frame.point.x = 0.0;
      origin_des_frame.point.y = 0.0;
      origin_des_frame.point.z = 0.0; */
    origin_des_frame.pose = des_state.des_pose;
    try {
      /*tf_listener_.transformPose("base_link", origin_des_frame, origin);
        origin.pose.position.x += -0.711;
        origin.pose.position.y += -0.3048;
        origin.pose.position.z += 0.0; */
      double width = 0.6096;
      double length = 1.422;
      double height = 2.00;
      tf::Stamped<tf::Pose > tf_origin;
      tf::poseStampedMsgToTF(origin_des_frame, tf_origin);
      tf::Pose shift_amount(tf::createQuaternionFromYaw(0.0), tf::Vector3(-length/2., -width/2., 0.));
      tf_origin.setData(tf_origin * shift_amount);
      tf::poseStampedTFToMsg(tf_origin, origin);
      double resolution = 0.05;
      ROS_DEBUG_STREAM("origin: " << origin);
      bool collision_detected = costmap_->checkRectangularPrismBase(origin, width, height, length, resolution, checkEntireVolume);
      if (collision_detected) {
        ROS_WARN("collision_detected");
        return true;
      } else {
        return false;
      }
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return true;
    }
  }
  else {
    ROS_DEBUG("Not checking for collisions, so let any desired state pass");
    return false;
  }
}

bool IdealStateGenerator::computeState(precision_navigation_msgs::DesiredState& new_des_state)
{
  double v = 0.0;
  bool end_of_path = false;
  double dL = desiredState_.des_speed * dt_;
  if(seg_number_ >= path_.size()) {
    //Out of bounds
    seg_number_ = path_.size()-1;
    end_of_path = true;
  }

  ros::Time current_transform = ros::Time::now();
  if (path_.at(seg_number_).seg_type == precision_navigation_msgs::PathSegment::SPIN_IN_PLACE) {
    seg_length_done_ = seg_length_done_ + dL;
  } else {
    temp_pose_in_.header.frame_id = "base_link";
    temp_pose_in_.pose.position.x = 0.0;
    temp_pose_in_.pose.position.y = 0.0;
    temp_pose_in_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    tf_listener_.getLatestCommonTime(temp_pose_in_.header.frame_id, "odom", current_transform, NULL);
    temp_pose_in_.header.stamp = current_transform;
    tf_listener_.transformPose("odom", temp_pose_in_, temp_pose_out_);
    double psiPSO = tf::getYaw(temp_pose_out_.pose.orientation);
    double psiDes = tf::getYaw(desiredState_.des_pose.orientation);
    //Need to only advance by the projection of what we did onto the desired heading	
    // formula is v * dt * cos(psiDes - psiPSO)
    seg_length_done_ = seg_length_done_ + dL * cos(psiDes - psiPSO);
  }
  double lengthSeg = path_.at(seg_number_).seg_length;
  if(seg_length_done_ > lengthSeg) {
    seg_length_done_ = 0.0;
    seg_number_++;
  }

  if(seg_number_ >= path_.size()) {
    //Out of bounds
    seg_number_ = path_.size()-1;
    end_of_path = true;
  }

  lengthSeg = path_.at(seg_number_).seg_length;
  if(end_of_path) {
    //If we should stop because we ran out of path, we should command the state corresponding to s=1
    seg_length_done_ = lengthSeg;
  }

  precision_navigation_msgs::PathSegment currentSeg = path_.at(seg_number_);

  double vNext;
  if (currentSeg.seg_type == precision_navigation_msgs::PathSegment::SPIN_IN_PLACE) {
    vNext = 0.0;
    v = currentSeg.max_speeds.angular.z;
  } else {
    v = currentSeg.max_speeds.linear.x;
    if (seg_number_ < path_.size()-1) {
      vNext = path_.at(seg_number_+1).max_speeds.linear.x;
    } 
    else {
      vNext = 0.0;	
    }
  }

  double tDecel = (v - vNext)/currentSeg.decel_limit;
  double vMean = (v + vNext)/2.0;
  double distDecel = vMean*tDecel;

  double lengthRemaining = currentSeg.seg_length - seg_length_done_;
  if(lengthRemaining < 0.0) {
    lengthRemaining = 0.0;
  }
  else if (lengthRemaining < distDecel) {
    v = sqrt(2*lengthRemaining*currentSeg.decel_limit + pow(vNext, 2));
  }
  else {
    v = v + currentSeg.accel_limit*dt_;
  }

  if (currentSeg.seg_type == precision_navigation_msgs::PathSegment::SPIN_IN_PLACE) {
    v = std::min(v, currentSeg.max_speeds.angular.z);
  } else {
    v = std::min(v, currentSeg.max_speeds.linear.x); //gonna fail for negative v commands along the path
  }

  //done figuring out our velocity commands

  //Convert into the odometry frame from whatever frame the path segments are in
  temp_pose_in_.header.frame_id = currentSeg.header.frame_id;
  temp_pose_in_.pose.position = currentSeg.ref_point;
  temp_pose_in_.pose.orientation = currentSeg.init_tan_angle;
  current_transform = ros::Time::now();
  tf_listener_.getLatestCommonTime(temp_pose_in_.header.frame_id, "odom", current_transform, NULL);
  temp_pose_in_.header.stamp = current_transform;
  tf_listener_.transformPose("odom", temp_pose_in_, temp_pose_out_);

  double tanAngle = tf::getYaw(temp_pose_out_.pose.orientation);
  //std::cout << "seg_number_ " << seg_number_ << std::endl;
  //std::cout << "seg_length_done_ " << seg_length_done_ << std::endl;
  //std::cout << "tan angle " << tanAngle << std::endl;
  double radius, tangentAngStart, arcAngStart, dAng, arcAng, rho;
  bool should_halt = false;
  //std::cout << seg_number_ << std::endl;
  switch(currentSeg.seg_type){
    case precision_navigation_msgs::PathSegment::LINE:
      new_des_state.seg_type = currentSeg.seg_type;
      new_des_state.des_pose.position.x = temp_pose_out_.pose.position.x + seg_length_done_*cos(tanAngle);
      new_des_state.des_pose.position.y = temp_pose_out_.pose.position.y + seg_length_done_*sin(tanAngle);
      new_des_state.des_pose.orientation = tf::createQuaternionMsgFromYaw(tanAngle);
      new_des_state.des_rho = currentSeg.curvature;
      new_des_state.des_speed = v;
      new_des_state.des_lseg = seg_length_done_;
      break;
    case precision_navigation_msgs::PathSegment::ARC:
      rho = currentSeg.curvature;
      //std::cout << "rho " << rho << std::endl;
      radius = 1.0/fabs(rho);
      //std::cout << "radius = " << radius << std::endl;
      tangentAngStart = tanAngle;
      arcAngStart = 0.0;
      if(rho >= 0.0) {
        arcAngStart = tangentAngStart - M_PI / 2.0;	
      } else {
        arcAngStart = tangentAngStart + M_PI / 2.0;
      }
      dAng = seg_length_done_*rho;
      //std::cout << "dAng " << dAng << std::endl;
      arcAng = arcAngStart + dAng;
      new_des_state.seg_type = currentSeg.seg_type;
      new_des_state.des_pose.position.x = temp_pose_out_.pose.position.x + radius * cos(arcAng);
      new_des_state.des_pose.position.y = temp_pose_out_.pose.position.y  + radius * sin(arcAng);
      new_des_state.des_pose.orientation = tf::createQuaternionMsgFromYaw(tanAngle + dAng);
      new_des_state.des_rho = currentSeg.curvature;
      new_des_state.des_speed = v;
      new_des_state.des_lseg = seg_length_done_;
      break;
    case precision_navigation_msgs::PathSegment::SPIN_IN_PLACE:
      rho = currentSeg.curvature;
      tangentAngStart = tanAngle;
      arcAngStart = 0.0;
      if(rho >= 0.0) {
        arcAngStart = tangentAngStart - M_PI / 2.0;	
      } else {
        arcAngStart = tangentAngStart + M_PI / 2.0;
      }
      dAng = seg_length_done_*rho;
      arcAng = arcAngStart + dAng;
      new_des_state.seg_type = currentSeg.seg_type;
      new_des_state.des_pose.position.x = temp_pose_out_.pose.position.x;
      new_des_state.des_pose.position.y = temp_pose_out_.pose.position.y;
      new_des_state.des_pose.orientation = tf::createQuaternionMsgFromYaw(tanAngle + dAng);
      new_des_state.des_rho = currentSeg.curvature;
      new_des_state.des_speed = v;
      new_des_state.des_lseg = seg_length_done_;
      break;
    default:
      ROS_WARN("Unknown segment type. Type was %d. Halting", currentSeg.seg_type);
      new_des_state.des_speed = 0.0;
      should_halt = true;
  }
  feedback_.seg_number = seg_number_;
  feedback_.current_segment = currentSeg;
  feedback_.seg_distance_done = seg_length_done_;
  as_.publishFeedback(feedback_);
  return should_halt;
}

void IdealStateGenerator::newPathCallback() {
  seg_number_ = 0;
  seg_length_done_ = 0.0;
  path_ = as_.acceptNewGoal()->segments; 
  ROS_DEBUG("%s: New goal accepted", action_name_.c_str());
}

void IdealStateGenerator::preemptPathCallback() {
  ROS_INFO("%s: Preempted. Holding current position", action_name_.c_str());
  as_.setPreempted();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ideal_state_generator");
  IdealStateGenerator idealState;

  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();
  return 0;
}
