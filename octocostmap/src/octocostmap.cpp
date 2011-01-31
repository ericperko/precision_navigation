/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Eric Perko
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

#include <octocostmap/octocostmap.h>

#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>

namespace octocostmap {
  OctoCostmap::OctoCostmap() : tfl_(), priv_nh_("~"), map_frame_("map"), map_resolution_(0.05) {
    priv_nh_.param("map_frame", map_frame_, map_frame_);
    priv_nh_.param("map_resolution", map_resolution_, map_resolution_);
    laser_sub_.subscribe(nh_, "scan", 100);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tfl_, map_frame_, 100);
    tf_filter_->registerCallback(boost::bind(&OctoCostmap::laserCallback, this, _1));

    octree_ = new octomap::OcTree(map_resolution_);
  
  }

  void OctoCostmap::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ros::Time start = ros::Time::now();
    octomap::point3d octomap_3d_point;
    octomap::Pointcloud octomap_pointcloud;

    sensor_msgs::PointCloud cloud;
    try {
      projector_.projectLaser(*scan, cloud);
      //projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, cloud, tfl_);
      for(size_t i = 0; i < cloud.points.size(); i++) {
        octomap_3d_point(0) = cloud.points[i].x;
        octomap_3d_point(1) = cloud.points[i].y;
        octomap_3d_point(2) = cloud.points[i].z;

        octomap_pointcloud.push_back(octomap_3d_point);
      }

      tf::StampedTransform transform;
      tfl_.lookupTransform(map_frame_, scan->header.frame_id, scan->header.stamp, transform); 
      octomath::Vector3 laser_point(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
      octomath::Quaternion laser_quat(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
      octomath::Pose6D laser_pose(laser_point, laser_quat);
      octree_->insertScan(octomap_pointcloud, laser_pose);
    } catch (tf::TransformException ex) {
      ROS_ERROR("Error finding origin of the laser in the map frame. Error was %s", ex.what());
    }
    ROS_DEBUG("Laser callback took %f milliseconds", (ros::Time::now() - start).toSec() * 1000.0);
  }

  void OctoCostmap::writeBinaryMap(const std::string& filename) {
    octree_->writeBinary(filename);
  }

  OctoCostmap::~OctoCostmap() {
   delete octree_;
   delete tf_filter_;
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "octocostmap");
  octocostmap::OctoCostmap octocostmap;
  //ros::MultiThreadedSpinner spinner(4);
  //spinner.spin();
  ros::spin();
  octocostmap.writeBinaryMap("octomap.bt");
  return 0;
}
