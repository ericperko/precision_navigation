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

#include <octocostmap/costmap_3d.h>
#include <octomap_ros/conversions.h>
#include <boost/foreach.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace octocostmap {
  Costmap3D::Costmap3D(const std::string &name, tf::TransformListener &tfl):
    name_(name), map_frame_("map"), tfl_(tfl), nh_(), priv_nh_("~/" + name_), octree_(0.5) {
      octomap_sub_ = nh_.subscribe<octomap_ros::OctomapBinary>("octomap", 1, boost::bind(&Costmap3D::octomapCallback, this, _1));
      collision_volume_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("collison_volume_cloud", 1);
    }

  void Costmap3D::octomapCallback(const octomap_ros::OctomapBinary::ConstPtr& map) {
    bool locked = octree_lock_.try_lock();
    if (locked) {
      octomap::octomapMsgToMap(*map, octree_);
      map_frame_= map->header.frame_id;
      octree_lock_.unlock();
    }
  }

  bool Costmap3D::checkRectangularPrismBase(const geometry_msgs::PointStamped &origin, double width, double height, double length, double resolution) {
    std::vector<tf::Point > collision_pts;
    tf::Stamped<tf::Point > temp_origin;
    tf::pointStampedMsgToTF(origin, temp_origin);
    tf::StampedTransform transform;
    double num_pts = width * height * length / (resolution * resolution * resolution);
    try {
      tfl_.lookupTransform(map_frame_, temp_origin.frame_id_, temp_origin.stamp_, transform);
      tf::Point origin_pt = transform * temp_origin;
      tf::Transform rotation_only(transform.getRotation());
      tf::Vector3 x_vec = rotation_only * tf::Vector3(length,0,0);
      tf::Vector3 y_vec = rotation_only * tf::Vector3(0,width,0);
      tf::Vector3 z_vec = rotation_only * tf::Vector3(0,0,height);
      tf::Point temp;
      collision_pts.reserve( (size_t) (num_pts + 1));
      double x_resolution = resolution/length;
      double y_resolution = resolution/width;
      double z_resolution = resolution/height;
      for (double dx = 0.0; dx <= 1.0; dx += x_resolution) {
        for (double dy = 0.0; dy <= 1.0; dy += y_resolution) {
          for (double dz = 0.0; dz <= 1.0; dz += z_resolution) {
            /*temp.setX(temp_origin.x() + dx);
            temp.setY(temp_origin.y() + dy);
            temp.setZ(temp_origin.z() + dz);
            */
            temp = origin_pt + dx*x_vec + dy*y_vec + dz*z_vec;
            collision_pts.push_back(temp);
          }
        }
      }
      if(collision_volume_pub_.getNumSubscribers() > 0) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_ptr->header.frame_id = map_frame_;
        cloud_ptr->header.stamp = temp_origin.stamp_;
        BOOST_FOREACH( tf::Point pt, collision_pts) {
          cloud_ptr->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
        }
        collision_volume_pub_.publish(cloud_ptr);
      }
      return checkCollisionVolume(collision_pts);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      //Err on the side of caution if we can't transform a point
      return true;
    }
  }

  bool Costmap3D::checkCollisionVolume(const std::vector<tf::Point> &collision_volume) {
    uint32_t num_points_checked = 0;
    bool retval = false;
    octomap::OcTreeNode *current_cell;
    octree_lock_.lock_shared();
    BOOST_FOREACH( tf::Point pt, collision_volume) {
      current_cell = octree_.search(pt.x(), pt.y(), pt.z());
      num_points_checked++;
      if(current_cell) {
        if (current_cell->isOccupied()) {
          retval = true;
          break;
        }
      }
    }
    octree_lock_.unlock_shared();
    ROS_DEBUG("Checked %d out of %d points before breaking out", num_points_checked, collision_volume.size());
    return retval;
  }
};
