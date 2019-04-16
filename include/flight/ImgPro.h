#pragma once
#ifndef IMGPRO_H
#define IMGPRO_H

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <flight/common_include.h>
#include <flight/Config.h>
#include "cuda/edge_detector.cuh"
#include <assert.h>
#include "cuda/img_process_gpu.cuh"

using namespace std;
struct imgPro
{
public:
  ros::NodeHandle nh;
  ros::Subscriber Disparity_sub;
  ros::Publisher piex_point_pub;
  image_transport::Publisher image_pub_left, image_pub_right, image_pub_laplacianLeft;

  std_msgs::Float32MultiArray Disparity_array, Disparity_array_current;

  geometry_msgs::Point32 piex_obstacle_point;
  int image_width = 1280, image_height = 720;
  float up_error, down_error, depth2disp_ratio;
  float true_disparity;
  double true_probobility = 0.0, all_count = 0.0, true_count = 0.0;

  void call_Disparitymsg(const std_msgs::Float32MultiArray &msg);
  bool check_Disparity(int pxX, int pxY, std_msgs::Float32MultiArray &Disparity_array_);
  bool check_Depth(int pxX, int pxY);

  //checkHorizontalInvariance
  cv::Mat imgLeft, imgRight, laplacianLeft, laplacianRight, matQ, imgdepth;

  int flag;
  int image_num;
  int zero_disparity, sobelLimit, disparity;
  bool check_horizontal_invariance = true;
  int INVARIANCE_CHECK_HORZ_OFFSET_MIN, INVARIANCE_CHECK_HORZ_OFFSET_MAX, INVARIANCE_CHECK_VERT_OFFSET_INCREMENT;
  int blockSize, INVARIANCE_CHECK_VERT_OFFSET_MIN, INVARIANCE_CHECK_VERT_OFFSET_MAX, horizontalInvarianceMultiplier;
  int sadThreshold;
  //int sad;

  // int *pixel_points;
  // float *camera_points;

   unsigned char *src_img1, *src_img2;
   unsigned char *lapla_img1, *lapla_img2;

   int *pixel_points;
   float *camera_points;

  imgPro *dev_ptr;
  image_process *dev_imgpro;
  image_process *tmp_imgpro;

  
  void getImgRight(const sensor_msgs::ImageConstPtr &msg);
  void getImgLeft(const sensor_msgs::ImageConstPtr &msg);
  void getImgDepth(const sensor_msgs::ImageConstPtr &msg);
  int getSAD(Mat &leftImage, Mat &rightImage, Mat &laplacianL, Mat &laplacianR, int pxX, int pxY);
  bool checkHorizontalInvariance(Mat &leftImage, Mat &rightImage, Mat &sobelL, Mat &sobelR, int pxX, int pxY);
  imgPro(string strSettingPath);
    ~imgPro()
  {
      cudaDeviceReset();
  }
  //vector<Point3f> hitPoints();
  //void LaplacianPro(cv::Mat& src_left,cv::Mat& src_right,cv::Mat& dst_left,cv::Mat& dst_right);
  void LaplacianPro();


  void HitPoints(vector<Point3f> &localHitPoints, vector<Point3i> &pointVector2d);
  void HitPoints_gpu();
  void get_camera_points_cuda(vector<Point3f> &localHitPoints);
  void get_pixel_points_cuda(vector<Point3i> &pointVector2d);

  void laplacian_cpu(Mat &srcImg, Mat &dstImg, int imgHeight, int imgWidth);
};


#endif
