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

#ifndef OCTOCOSTMAP_H_
#define OCTOCOSTMAP_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <octomap/octomap.h>

namespace octocostmap {
    class OctoCostmap {
        public:
            /**
              * @brief Constructor
              */
            OctoCostmap();

            /**
              * @brief Destructor
              */
            ~OctoCostmap();

            void writeBinaryMap(const std::string& filename);

            double lookupPoint(double &x, double &y, double &z) {
                octomap::OcTreeNode *cell = octree_->search(x,y,z);
                if (cell) {
                  return cell->getOccupancy();
                } else {
                  return -1.0;
                }
            }
        private:
            tf::TransformListener tfl_;
            message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
            tf::MessageFilter<sensor_msgs::LaserScan>* tf_filter_;
            ros::NodeHandle nh_;
            ros::NodeHandle priv_nh_;
            std::string map_frame_;
            double map_resolution_;
            laser_geometry::LaserProjection projector_;
            octomap::OcTree *octree_;

            void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    };
};

#endif
