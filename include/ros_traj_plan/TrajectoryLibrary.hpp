#ifndef TRAJECTORYLIB_TRAJLIB_HPP
#define TRAJECTORYLIB_TRAJLIB_HPP

/*
 * Trajectory library class
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <omp.h>
#include <cmath>

#include <dirent.h>
#include <sys/types.h>
#include <tuple>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "ros_traj_plan/Trajectory.hpp"
#include "ros_traj_plan/uavtrans.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "geometry_msgs/PoseStamped.h"

class TrajectoryLibrary //负责寻找路劲库，并将文件名（绝对路劲+文件名）传入Trajectory类的实参中
{						//利用Trajectory中的成员函数求得最佳路劲，并得到路径库的编号

  public:
	TrajectoryLibrary(double ground_safety_distance = 0, int this_traj = 0); //构造函数

	Trajectory *GetTrajectoryByNumber(int number); //选择路径，返回指针

	bool LoadLibrary(std::string dirname, bool quiet = false); // loads a trajectory from a directory of .csv files

	bool set_desstraight_traj(geometry_msgs::PoseStamped des_, const uavTrans &start_p);

	bool airsim_set_desstraight_traj(geometry_msgs::PoseStamped des_, const uavTrans &start_p);

	std::tuple<int, double, Trajectory *, std::vector<geometry_msgs::PoseStamped>> FindFarthestTrajectory(double threshold, const uavTrans body_to_local, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_, int preferred_traj[] = {});
	void Print() const;

	int GetNumberTrajectories() const { return int(traj_vec_.size()); } //路径的个数
	int GetThisTrajNum() const { return this_traj_; }
	void SetGroundSafetyDistance(double dist) { double ground_safety_distance_ = dist; } //设置安全距离

	//	void Draw(lcm_t *lcm, const BotTrans *transform = nullptr) const;

  private:
	std::vector<Trajectory> traj_vec_;
	double ground_safety_distance_;
	int this_traj_;
};

#endif
