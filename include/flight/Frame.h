#pragma once
#ifndef FRAME_H
#define FRAME_H


#include <flight/common_include.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
class Frame
{
public:
  cv::Mat matQ;
  double camera_factor;
  ros::Timer obstacle_timer;
  pcl::PointXYZ current_position;
  Eigen::Vector3d camera_rpy;

  Eigen::Isometry3d T_mavros,T;
  int image_pose_num;
  std::vector<Point3f> hitPointsCamera;
  std::vector<Point3f> hitPointsWorld;

  geometry_msgs::Point32 obstacle_point;
  bool flag_simoutanous;

  ros::NodeHandle nh;
  ros::Publisher obstacle_point_publisher;
  ros::Subscriber mavros_pose_sub;
  image_transport::Publisher imageResultleft;
  bool odom_flag;

public:
  void flashFrame();
  ~Frame()
  {
    hitPointsCamera.clear();
    vector<Point3f>(hitPointsCamera).swap(hitPointsCamera);

    hitPointsWorld.clear();
    vector<Point3f>(hitPointsWorld).swap(hitPointsWorld);

    cout << "Frame points has been removed" << endl;
  }
  Frame(cv::Mat Q);
  void getOdom(const nav_msgs::Odometry &msg);
  void get_mavros_pose(const geometry_msgs::PoseStamped &msg);
  sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t);
  void visualizaFrame(cv::Mat &displayL, vector<Point3i> pointVector2d, int blockSize);
  void pixelToCamera(std::vector<Point3f> hitPointsPixel);
  void cameraToWorld();

  void obstacle_visualization(const ros::TimerEvent &te);
  pcl::PointXYZ &get_current_position();
};


#endif