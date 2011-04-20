#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <precision_navigation_msgs/DesiredState.h>
#include <precision_navigation_msgs/PathSegment.h>
#include <precision_navigation_msgs/Path.h>
#include <octocostmap/costmap_3d.h>
#include <vector>
#include <cmath>
#include <algorithm>

class IdealStateGenerator {
  public:
    IdealStateGenerator();
  private:
    void computeState(float& x, float& y, float& theta, float& v, float& rho);
    void pathCallback(const precision_navigation_msgs::Path::ConstPtr& p);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    precision_navigation_msgs::DesiredState makeHaltState();
    void computeStateLoop(const ros::TimerEvent& event);
    bool checkCollisions(bool checkEntireVolume);

    //Loop rate in Hz
    double loop_rate;
    double dt;
    bool halt;

    double segDistDone;
    uint32_t iSeg;

    //Current path to be working on
    std::vector<precision_navigation_msgs::PathSegment> path;
    //Last cmd_vel
    geometry_msgs::Twist last_cmd_;
    //Last odometry
    nav_msgs::Odometry last_odom_;
    //Is the odometry ready?
    bool first_call_;

    //The last desired state we output	
    precision_navigation_msgs::DesiredState desiredState_;

    //ROS communcators
    ros::NodeHandle nh_;
    ros::Publisher ideal_state_pub_;
    ros::Publisher ideal_pose_marker_pub_;
    ros::Subscriber path_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber odom_sub_;
    tf::TransformListener tf_listener_;	
    geometry_msgs::PoseStamped temp_pose_in_, temp_pose_out_;
    boost::shared_ptr<octocostmap::Costmap3D> costmap_;
    ros::Timer compute_loop_timer_;
};

const double pi = acos(-1.0);

IdealStateGenerator::IdealStateGenerator() {
  //Setup the ideal state pub
  ideal_state_pub_= nh_.advertise<precision_navigation_msgs::DesiredState>("idealState",1);   
  ideal_pose_marker_pub_= nh_.advertise<geometry_msgs::PoseStamped>("ideal_pose",1);   
  path_sub_ = nh_.subscribe<precision_navigation_msgs::Path>("desired_path", 1, &IdealStateGenerator::pathCallback, this);
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &IdealStateGenerator::cmdVelCallback, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &IdealStateGenerator::odomCallback, this);
  nh_.param("loop_rate",loop_rate,20.0); // default 20Hz
  dt = 1.0/loop_rate;
  costmap_ = boost::shared_ptr<octocostmap::Costmap3D>(new octocostmap::Costmap3D("octocostmap", tf_listener_));

  first_call_ = true;

  //Setup the loop timer
  compute_loop_timer_ = nh_.createTimer(ros::Duration(dt), boost::bind(&IdealStateGenerator::computeStateLoop, this, _1));

  //Initialze private class variables
  iSeg = 0;
  segDistDone = 0.0;
  halt = true;


  tf_listener_.waitForTransform("odom", "map", ros::Time::now(), ros::Duration(10));
  tf_listener_.waitForTransform("odom", "base_link", ros::Time::now(), ros::Duration(10));
  desiredState_ = makeHaltState();
}

//We want to take the current location of the base and set that as the desired state with 0 velocity and rho. 
precision_navigation_msgs::DesiredState IdealStateGenerator::makeHaltState() {
  precision_navigation_msgs::DesiredState halt_state;
  //If the path is empty, we want to actually halt where we are
  //If the path is non empty, we should just keep sending the last desired state, but with 0 v, just to be safe
  if (path.size() > 0) {
    halt_state = desiredState_;
    halt_state.v = 0.0;
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

    double tanAngle = tf::getYaw(temp_pose_out_.pose.orientation);
    halt_state.header.frame_id = "odom";
    halt_state.header.stamp = ros::Time::now();
    halt_state.x = temp_pose_out_.pose.position.x;
    halt_state.y = temp_pose_out_.pose.position.y;
    halt_state.theta = tanAngle;
    halt_state.v = 0.0;
    halt_state.rho = 0.0;
  }
  return halt_state;

}

void IdealStateGenerator::computeStateLoop(const ros::TimerEvent& event) {
  ROS_DEBUG("Last callback took %f seconds", event.profile.last_duration.toSec());
  //temps
  float x = 0.0;
  float y = 0.0;
  float theta = 0.0;
  float v = 0.0;
  float rho = 0.0;

  if(!first_call_) {
    //Orientation is a quaternion, so need to get yaw angle in rads.. unless you want a quaternion
    theta = tf::getYaw(last_odom_.pose.pose.orientation);
    //v = last_cmd_.linear.x; //last velocity that was actually commanded by steering
    v = desiredState_.v; //last desired velocity
    computeState(x,y,theta,v,rho);

    //Put the temp vars into the desiredState
    desiredState_.header.frame_id = "odom";
    desiredState_.header.stamp = ros::Time::now();
    if(halt) {
      desiredState_ = makeHaltState();
    }
    else {
      desiredState_.x = x;
      desiredState_.y = y;
      desiredState_.theta = theta;
      desiredState_.v = v;
      desiredState_.rho = rho;
    }
    if(checkCollisions(false)) {
      desiredState_ = makeHaltState();
    }
    //Publish twist message
    ideal_state_pub_.publish(desiredState_);
    geometry_msgs::PoseStamped des_pose;
    des_pose.header = desiredState_.header;
    des_pose.pose.position.x = desiredState_.x;
    des_pose.pose.position.y = desiredState_.y;
    des_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    ideal_pose_marker_pub_.publish(des_pose);
  }
}

bool IdealStateGenerator::checkCollisions(bool checkEntireVolume) {
  geometry_msgs::PoseStamped origin, origin_des_frame;
  origin_des_frame.header = desiredState_.header;
  /*origin_des_frame.header.frame_id = std::string("base_link");
    origin_des_frame.point.x = 0.0;
    origin_des_frame.point.y = 0.0;
    origin_des_frame.point.z = 0.0; */
  origin_des_frame.pose.position.x = desiredState_.x;
  origin_des_frame.pose.position.y = desiredState_.y;
  origin_des_frame.pose.position.z = 0.0;
  origin_des_frame.pose.orientation = tf::createQuaternionMsgFromYaw(desiredState_.theta);
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

void IdealStateGenerator::computeState(float& x, float& y, float& theta, float& v, float& rho)
{
  double dL = v * dt;
  if(iSeg >= path.size()) {
    //Out of bounds
    halt = true;
    v = 0.0;
    return;
  }

  //Need to only advance by the projection of what we did onto the desired heading	
  // formula is v * dt * cos(psiDes - psiPSO)
  segDistDone = segDistDone + dL * cos(desiredState_.theta - theta);
  double lengthSeg = path.at(iSeg).length;
  if(segDistDone > lengthSeg) {
    segDistDone = 0.0;
    iSeg++;
  }

  if(iSeg >= path.size()) {
    //Out of bounds
    halt = true;
    v = 0.0;
    return;
  }

  precision_navigation_msgs::PathSegment currentSeg = path.at(iSeg);

  double vNext;
  v = currentSeg.vDes;
  if (iSeg < path.size()-1) {
    vNext = path.at(iSeg+1).vDes;
  } 
  else {
    vNext = 0.0;	
  }

  double tDecel = (v - vNext)/currentSeg.accel;
  double vMean = (v + vNext)/2.0;
  double distDecel = vMean*tDecel;

  double lengthRemaining = currentSeg.length - segDistDone;
  if(lengthRemaining < 0.0) {
    lengthRemaining = 0.0;
  }
  else if (lengthRemaining < distDecel) {
    v = sqrt(2*lengthRemaining*currentSeg.accel + pow(vNext, 2));
  }
  else {
    v = v + currentSeg.accel*dt;
  }

  v = std::min(v, currentSeg.vDes); //gonna fail for negative v commands along the path

  //done figuring out our velocity commands

  //Convert into the odometry frame from whatever frame the path segments are in
  temp_pose_in_.header.frame_id = currentSeg.frame_id;
  temp_pose_in_.pose.position.x = currentSeg.xRef;
  temp_pose_in_.pose.position.y = currentSeg.yRef;
  temp_pose_in_.pose.orientation = tf::createQuaternionMsgFromYaw(currentSeg.tangentAng);
  ros::Time current_transform = ros::Time::now();
  tf_listener_.getLatestCommonTime(temp_pose_in_.header.frame_id, "odom", current_transform, NULL);
  temp_pose_in_.header.stamp = current_transform;
  tf_listener_.transformPose("odom", temp_pose_in_, temp_pose_out_);

  double tanAngle = tf::getYaw(temp_pose_out_.pose.orientation);
  //std::cout << "iSeg " << iSeg << std::endl;
  //std::cout << "segDistDone " << segDistDone << std::endl;
  //std::cout << "tan angle " << tanAngle << std::endl;
  double radius, tangentAngStart, arcAngStart, dAng, arcAng;
  //std::cout << iSeg << std::endl;
  switch(currentSeg.segType){
    case 1:
      theta = tanAngle;
      rho = currentSeg.rho;
      x = temp_pose_out_.pose.position.x + segDistDone*cos(theta);
      y = temp_pose_out_.pose.position.y + segDistDone*sin(theta);
      halt = false;
      break;
    case 2:
      rho = currentSeg.rho;
      //std::cout << "rho " << rho << std::endl;
      radius = 1.0/fabs(rho);
      //std::cout << "radius = " << radius << std::endl;
      tangentAngStart = tanAngle;
      arcAngStart = 0.0;
      if(rho >= 0.0) {
        arcAngStart = tangentAngStart - pi / 2.0;	
      } else {
        arcAngStart = tangentAngStart + pi / 2.0;
      }
      dAng = segDistDone*rho;
      //std::cout << "dAng " << dAng << std::endl;
      arcAng = arcAngStart + dAng;
      x = temp_pose_out_.pose.position.x + radius * cos(arcAng);
      y = temp_pose_out_.pose.position.y  + radius * sin(arcAng);
      theta = tanAngle + dAng;
      halt = false;
      break;
    default:
      halt = true;
      v = 0.0;
  }
}

void IdealStateGenerator::pathCallback(const precision_navigation_msgs::Path::ConstPtr& p) {
  //Reset initial state cause the path is about to change
  iSeg = 0;
  segDistDone = 0.0;
  halt = true;   
  path = p->segs;
}

void IdealStateGenerator::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
  last_cmd_ = *cmd_vel;
}

void IdealStateGenerator::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
  first_call_ = false;
  last_odom_ = *odom;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ideal_state_generator");
  IdealStateGenerator idealState;

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}
