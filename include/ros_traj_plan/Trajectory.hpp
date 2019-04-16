#ifndef TRAJECTORY_TRAJLIB_HPP
#define TRAJECTORY_TRAJLIB_HPP

/*
 * Trajectory class representing a trajectory in state space
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013-2015
 *
 */

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <cmath>

#include <dirent.h>
#include "ros_traj_plan/csvparser.h"
#include "ros_traj_plan/uavtrans.h"
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseStamped.h"

#include <Eigen/Core>

class Trajectory //laoad CSV中的矩阵数据（利用上面的得到的文件索引）
                 //存入到matrix中共其他成员函数使用，计算各个路径轨迹点距离障碍物的最短距离
{

public:
  Trajectory();
  Trajectory(int trajectory_number, Eigen::MatrixXd matrix, int dimension, double dt, double min_altitude);
  Trajectory(std::string filename_prefix, bool quiet = false); // loads a trajectory from a .csv file

  void LoadTrajectory(std::string filename_prefix, bool quiet = false);

  int GetDimension() const { return dimension_; } //获得位置信息
  int GetUDimension() const { return udimension_; }
  int GetTrajectoryNumber() const { return trajectory_number_; } //获得轨迹编号
  double GetDT() const { return dt_; }                           //设置时间隔

  void GetXyzYawTransformedPoint(double t, const uavTrans &transform, float xyz[]) const; //获得坐标信息
                                                                                          //    void Draw(bot_lcmgl_t *lcmgl, const BotTrans *transform = nullptr, double final_time = -1) const;

  int GetIndexAtTime(double t) const;                                   //获得在t时刻的索引
  double GetTimeAtIndex(int index) const { return xpoints_(index, 0); } //获得在t时刻的x坐标

  double GetMaxTime() const { return xpoints_(xpoints_.rows() - 1, 0); } //获得最大时间

  bool IsTimeInvariant() const { return upoints_.rows() == 1; }

  int GetNumberOfPoints() const { return int(xpoints_.rows()); }

  Eigen::VectorXd GetState(double t) const;
  //       Eigen::VectorXd GetUCommand(double t) const;
  //      Eigen::MatrixXd GetGainMatrix(double t) const;

  Eigen::MatrixXd GetXpoints() const { return xpoints_; }

  double ClosestObstacleInRemainderOfTrajectory(double current_t, uavTrans &body_to_local, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_, double min_altitude_allowed) const;

  void Print() const;

  double GetMinimumAltitude() const { return min_altitude_; }

private:
  Eigen::MatrixXd xpoints_;
  Eigen::MatrixXd upoints_;

  Eigen::MatrixXd kpoints_;
  Eigen::MatrixXd affine_points_;

  double dt_;
  double min_altitude_;

  int dimension_;  // state space dimension
  int udimension_; // control input dimension

  int trajectory_number_;
  std::string filename_prefix_;

  void LoadMatrixFromCSV(const std::string &filename, Eigen::MatrixXd &matrix, bool quiet = false);

  int GetNumberOfLines(std::string filename) const;
};
#endif
