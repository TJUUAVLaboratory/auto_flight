#include <ros/ros.h>
#include "ros_traj_plan/searchdistance.h"
#include "ros_traj_plan/uavtrans.h"
#include "mavros/frame_tf.h"
#include <iostream>

using namespace std;
typedef Eigen::Quaternion<double> Quaterniond;
typedef Eigen::Vector3d Vector3d;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "trajplan_test");
  Uav_trajplannimg traj_planning;

  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *current_octree_ = nullptr;
  clock_t fly_start, fly_finish;
  double flytime;
  std::vector<cv::Mat> img_lr;
  ros::Time time_current = ros::Time::now();

  ROS_INFO_STREAM_NAMED("FLYING TASK", "START"
                                           << " NOW ");

  traj_planning.fly_init(); //飞机初始化

  cloud1 = traj_planning.get_cloudmap();            //可去
  current_octree_ = InsertPointsIntoOctree(cloud1); //可去

  traj_planning.fly_takeoff(current_octree_); //飞机起飞(需要一个地图，里面可以没有点)

  fly_start = clock();

  while (ros::ok()) //&&!traj_planning.arrive_destination(traj_planning.get_trans_msg_a(),traj_planning.get_destinate_coor())  &&!traj_planning.airsim_arrive_destination()
  {
    /**********
      此处加入跟新的地图 pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>* octree_map
      **********/
    traj_planning.fly_traj_plan(current_octree_);
    ros::spinOnce();
    if (traj_planning.arrive_destination(traj_planning.get_trans_msg_a(), traj_planning.get_destinate_coor()))
    {
      break;
    }

    //   if(ros::Time::now()-time_current >= ros::Duration(0.03))
    //   {
    //       traj_planning.airsim_image_capture_thread(img_lr);
    //       time_current = ros::Time::now();
    //   }
  }

  fly_finish = clock();
  flytime = (double)(fly_finish - fly_start) / CLOCKS_PER_SEC;
  std::cout << "\n**************************\ndestination coordinate:\n"
            << traj_planning.get_destinate_coor() << std::endl;
  //cout << "飞行时间：" << flytime << "秒" << endl;
  std::cout << "********达到终点********" << std::endl;

  //airsim降落
  //traj_planning.airsim_land();
  //循环发布终点航点，等待手动切换

  traj_planning.mavros_auto_land();
  // while (ros::ok())
  // {
  //   traj_planning.pub_waypoint(traj_planning.get_destinate_coor());
  // }
  return 0;
}
