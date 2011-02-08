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
#include <octomap_server/octomap_server.h>
#include <boost/foreach.hpp>

namespace octocostmap {
  Costmap3D::Costmap3D(const std::string &name, tf::TransformListener &tfl):
    name_(name), tfl_(tfl), nh_(), priv_nh_("~/" + name_), octree_(0.5) {
      octomap_sub_ = nh_.subscribe<octomap_server::OctomapBinary>("octomap", 1, boost::bind(&Costmap3D::octomapCallback, this, _1));
    }

  void Costmap3D::octomapCallback(const octomap_server::OctomapBinary::ConstPtr& map) {
    bool locked = octree_lock_.try_lock();
    if (locked) {
      octomap_server::octomapMsgToMap(*map, octree_);
      map_frame_= map->header.frame_id;
      octree_lock_.unlock();
    }
  }

  bool Costmap3D::checkRectangularPrismBase(const geometry_msgs::PointStamped origin, double width, double height, double length, double resolution) {
    std::vector<geometry_msgs::PointStamped> pts;
    geometry_msgs::PointStamped temp = origin;
    double num_pts = width * height * length / (resolution * resolution * resolution);
    pts.reserve( (size_t) (num_pts + 1));
    for (double dx = 0.0; dx < length; dx += resolution) {
      for (double dy = 0.0; dy < width; dy += resolution) {
        for (double dz = 0.0; dz < height; dz += resolution) {
          temp.point.x = origin.point.x + dx;
          temp.point.y = origin.point.y + dy;
          temp.point.z = origin.point.z + dz;
          pts.push_back(temp);
        }
      }
    }
    return checkCollisionVolume(pts);
  }

  bool Costmap3D::checkCollisionVolume(const std::vector<geometry_msgs::PointStamped> &collision_volume) {
    bool retval = false;
    geometry_msgs::PointStamped temp;
    octomap::OcTreeNode *current_cell;
    octree_lock_.lock_shared();
    BOOST_FOREACH( geometry_msgs::PointStamped pt, collision_volume) {
      try {
        tfl_.transformPoint(map_frame_, pt, temp);
        current_cell = octree_.search(temp.point.x, temp.point.y, temp.point.z);
        if (current_cell->isOccupied()) {
          retval = true;
          break;
        }
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
      }
    }
    octree_lock_.unlock_shared();
    return retval;
  }
};
