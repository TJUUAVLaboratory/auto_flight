#pragma once
#ifndef STEREOMAP_H
#define STEREOMAP_H

#include <flight/common_include.h>

#define OCTREE_RESOLUTION 0.3f
class StereoMap
{

public:
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloud;

  ros::NodeHandle nh;
  ros::Publisher Octomap_pub;
  ros::Publisher auto_flight_octomap_state_pub;
  double map_resolution;
  double camera_factor;
  octomap::OcTree current_tree, building_tree;
  octomap_msgs::Octomap msg_octomap;
  ros::Timer octomap_timer;
  void octomap_visualization(const ros::TimerEvent &te);

  double map_exist_time = 4.0;
  double current_octomap_exist_time;
  double closest_distance, building_closest_distance;
  auto_flight::auto_flight_octomap_state auto_flight_octomap_state;

  sensor_msgs::PointCloud2 obstacle_pointcloud_output;
  ros::Publisher pclmap_pub;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *current_octree_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *building_octree_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr building_cloud_;
  std::vector<Point3f> cameraHitPoints;
  std::vector<Point3f> worldHitPoints;
  ros::Time current_octree_timestamp_, building_octree_timestamp_;
  bool flag_firstStart = true;
  void cameraToWorld();
  void RemoveOldPoints(ros::Time ros_time, pcl::PointXYZ &curent_position);
  void InsertPointsIntoOctree(vector<Point3f> hitPointsWorld);
  StereoMap();
  void stereoMapStatus();
};


#endif