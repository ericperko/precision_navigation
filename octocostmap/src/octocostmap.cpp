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

#include <octocostmap/octocostmap.h>
#include <octomap_ros/conversions.h>
#include <octomap_ros/OctomapBinary.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>

namespace octocostmap {
  OctoCostmap::OctoCostmap() : tfl_(), priv_nh_("~"), map_frame_("map"), map_resolution_(0.05) {
    priv_nh_.param("map_frame", map_frame_, map_frame_);
    priv_nh_.param("map_resolution", map_resolution_, map_resolution_);
    double publish_frequency = 0.0;
    priv_nh_.param("publish_frequency", publish_frequency, 10.0);
    if (publish_frequency > 0.0) {
      publish_period_ = ros::Duration(1/publish_frequency);
    } else {
      publish_period_ = ros::Duration(0.0);
    }
    laser_sub_.subscribe(nh_, "scan", 100);
    pc_sub_.subscribe(nh_, "cloud", 100);
    laser_tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tfl_, map_frame_, 100);
    laser_tf_filter_->registerCallback(boost::bind(&OctoCostmap::laserCallback, this, _1));
    pc_tf_filter_ = new tf::MessageFilter<pcl::PointCloud<pcl::PointXYZ> >(pc_sub_, tfl_, map_frame_, 100);
    pc_tf_filter_->registerCallback(boost::bind(&OctoCostmap::pointCloudCallback, this, _1));

    map_pub_ = nh_.advertise<octomap_ros::OctomapBinary>("octomap", 1, true);
    viz_point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("octomap_visualization_cloud", 1);

    octree_ = boost::shared_ptr<octomap::OcTreeROS>(new octomap::OcTreeROS(map_resolution_));
    ROS_DEBUG("Octocostmap constructed");
  }

  bool OctoCostmap::insertPointCloudXYZ(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl_ros::transformPointCloud(map_frame_, *cloud, transformed_cloud, tfl_);
    tf::StampedTransform transform;
    tfl_.lookupTransform(map_frame_, cloud->header.frame_id, cloud->header.stamp, transform); 
    pcl::PointXYZ origin(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    octree_->insertScan(transformed_cloud, origin);
    return true;
  }

  void OctoCostmap::pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    ros::WallTime start = ros::WallTime::now();
    /*octomap::point3d octomap_3d_point;
    octomap::Pointcloud octomap_pointcloud;

    try {
      BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points) {
        if (pt.x == pt.x && pt.y == pt.y && pt.z == pt.z) { // checking for NaNs
          octomap_3d_point(0) = pt.x;
          octomap_3d_point(1) = pt.y;
          octomap_3d_point(2) = pt.z;

          octomap_pointcloud.push_back(octomap_3d_point);
        }
      }
      tf::StampedTransform transform;
      tfl_.lookupTransform(map_frame_, cloud->header.frame_id, cloud->header.stamp, transform); 
      octomath::Vector3 laser_point(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
      octomath::Quaternion laser_quat(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
      octomath::Pose6D laser_pose(laser_point, laser_quat);
      octree_->octree.insertScan(octomap_pointcloud, laser_pose);
    } catch (tf::TransformException ex) {
      ROS_ERROR("Error finding origin of the cloud in the map frame. Error was %s", ex.what());
    } */
    insertPointCloudXYZ(cloud);
    ROS_DEBUG("Point cloud callback took %f milliseconds for %d points", (ros::WallTime::now() - start).toSec() * 1000.0, cloud->points.size());
    publishOctomapMsg();
    publishVisualizationPointCloud();
  }

  void OctoCostmap::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ros::WallTime start = ros::WallTime::now();
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    /*octomap::point3d octomap_3d_point;
    octomap::Pointcloud octomap_pointcloud;

    sensor_msgs::PointCloud2 cloud;
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
      //octree_->insertScan(octomap_pointcloud, laser_pose);
    } catch (tf::TransformException ex) {
      ROS_ERROR("Error finding origin of the laser in the map frame. Error was %s", ex.what());
    } */
    projector_.projectLaser(*scan, cloud);
    pcl::fromROSMsg(cloud, *pcl_cloud);
    insertPointCloudXYZ(pcl_cloud);
    ROS_DEBUG("Laser callback took %f milliseconds", (ros::WallTime::now() - start).toSec() * 1000.0);
    publishOctomapMsg();
    publishVisualizationPointCloud();
  }

  void OctoCostmap::publishOctomapMsg() {
    if (map_pub_.getNumSubscribers() > 0) {
      if (last_sent_time_ + publish_period_ < ros::Time::now()) {
        octomap_ros::OctomapBinary::Ptr map_ptr = boost::make_shared<octomap_ros::OctomapBinary>();
        octomap::octomapMapToMsg(octree_->octree, *map_ptr);
        map_ptr->header.frame_id = map_frame_;
        map_pub_.publish(map_ptr);
        last_sent_time_ = ros::Time::now();
        ROS_DEBUG("Published an octocostmap");
      }
    }
  }

  void OctoCostmap::publishVisualizationPointCloud() {
    if (viz_point_cloud_pub_.getNumSubscribers() > 0) {
        pcl::PointCloud<pcl::PointXYZ> viz_cloud;
        viz_cloud.header.frame_id = map_frame_;
        octomap::point3d_list points;
        octree_->octree.getOccupied(points);
        octomap::pointsOctomapToPCL(points, viz_cloud);
        viz_point_cloud_pub_.publish(viz_cloud);
        ROS_DEBUG("Published a visualization PointCloud");
      }
  }

  void OctoCostmap::writeBinaryMap(const std::string& filename) {
    octree_->octree.writeBinary(filename);
  }

  OctoCostmap::~OctoCostmap() {
    delete laser_tf_filter_;
    delete pc_tf_filter_;
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "octocostmap");
  octocostmap::OctoCostmap octocostmap;
  //ros::MultiThreadedSpinner spinner(4);
  //spinner.spin();
  ros::spin();
  /*ros::WallTime start = ros::WallTime::now();
    int counter = 0;
    double val = 0.0;
    for (double x = 0.0; x < 5.0; x += 0.01) {
    for (double y = 0.0; y < 5.0; y += 0.01) {
    for (double z = 0.0; z < 5.0; z += 0.01) {
    val = std::max(octocostmap.lookupPoint(x,y,z), val);
    counter++;
    }
    }
    }
    ROS_INFO("%f max occupancy value", val);
    ROS_INFO("%d lookups took %f milliseconds", counter, (ros::WallTime::now() - start).toSec() * 1000.0);
    */
  octocostmap.writeBinaryMap("octomap.bt");
  return 0;
}
