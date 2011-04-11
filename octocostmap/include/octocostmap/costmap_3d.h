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

#ifndef COSTMAP_3D_H_
#define COSTMAP_3D_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <octomap/octomap.h>
#include <octomap_ros/OctomapBinary.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/thread/shared_mutex.hpp>
#include <vector>

namespace octocostmap {
        class Costmap3D {
            public:
                Costmap3D(const std::string &name, tf::TransformListener &tfl);

                void octomapCallback(const octomap_ros::OctomapBinary::ConstPtr& map);

                bool checkCollisionVolume(const std::vector<tf::Point > &collision_volume);

                bool checkRectangularPrismBase(const geometry_msgs::PoseStamped &origin, double width, double height, double length, double resolution, bool check_full_resolution = true);
            private:
                std::string name_;
                std::string map_frame_;
                tf::TransformListener& tfl_;
                ros::NodeHandle nh_;
                ros::NodeHandle priv_nh_;
                ros::Subscriber octomap_sub_;
                ros::Publisher collision_volume_pub_;
                octomap::OcTree octree_;
                boost::shared_mutex octree_lock_;
        };
};

#endif
