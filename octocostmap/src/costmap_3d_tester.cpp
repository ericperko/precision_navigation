/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Eric Perko nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <octocostmap/costmap_3d.h>

void timerCallback(octocostmap::Costmap3D *costmap, const ros::TimerEvent& event) {
  ROS_DEBUG("Last callback took %f seconds", event.profile.last_duration.toSec());
  geometry_msgs::PointStamped origin;
  origin.header.frame_id = "base_link";
  origin.header.stamp = ros::Time::now();
  origin.point.x = -0.711;
  origin.point.y = -0.3048;
  origin.point.z = 0.0;
  double width = 0.6096;
  double length = 1.422;
  double height = 2.00;
  double resolution = 0.05;

  if (costmap->checkRectangularPrismBase(origin, width, height, length, resolution, false)) {
        ROS_DEBUG("collision detected");
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc,argv, "costmap_3d_tester");
  ros::NodeHandle nh;
  tf::TransformListener tfl;
  octocostmap::Costmap3D costmap("costmap", tfl);

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), boost::bind(timerCallback, &costmap, _1));

  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin(); // spin() will not return until the node has been shutdown
  return 0;
}
