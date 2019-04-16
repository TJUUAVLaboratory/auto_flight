#pragma once
#ifndef COMMONINCLUDE_H
#define COMMONINCLUDE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>

// for Sophus
//#include <sophus/se3.h>
//#include <sophus/so3.h>
//using Sophus::SE3;
//using Sophus::SO3;

// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

//octomap
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include "auto_flight/auto_flight_octomap_state.h"
#include "tf/transform_datatypes.h"
#include "mavros/frame_tf.h"

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <omp.h>

using namespace cv;
using namespace std;

#endif