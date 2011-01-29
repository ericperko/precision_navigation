#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <octomap/octomap.h>
#include <pcl/point_types.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

tf::TransformListener* tfl;
laser_geometry::LaserProjection projector;
octomap::OcTree octree_(0.01);


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  octomap::point3d octomap_3d_point;
  octomap::Pointcloud octomap_pointcloud;
  
  sensor_msgs::PointCloud cloud;
  try {
  projector.transformLaserScanToPointCloud(scan->header.frame_id, *scan, cloud, *tfl);
  for(size_t i = 0; i < cloud.points.size(); i++) {
        octomap_3d_point(0) = cloud.points[i].x;
        octomap_3d_point(1) = cloud.points[i].y;
        octomap_3d_point(2) = cloud.points[i].z;

        octomap_pointcloud.push_back(octomap_3d_point);
  }

  tf::StampedTransform transform;
  tfl->lookupTransform("map", scan->header.frame_id, scan->header.stamp, transform); 
  octomath::Vector3 laser_point(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
  octomath::Quaternion laser_quat(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
  octomath::Pose6D laser_pose(laser_point, laser_quat);
  octree_.insertScan(octomap_pointcloud, laser_pose);
  } catch (tf::TransformException ex) {
        ROS_ERROR(ex.what());
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "octocostmap");
  ros::NodeHandle nh;
  tfl = new tf::TransformListener();
  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserCallback);
  ros::spin();
  octree_.writeBinary("octomap.bt");
  delete tfl;
  return 0;
}
