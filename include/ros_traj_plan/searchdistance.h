#pragma once
#ifndef _SEARCHDISTANCE_H
#define _SEARCHDISTANCE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include "tf/transform_datatypes.h"
#include <ros/ros.h>

#include "auto_flight/destinate.h"
#include "auto_flight/auto_flight_state.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/WaypointList.h>

#include <GeographicLib/Geocentric.hpp>
#include <eigen_conversions/eigen_msg.h>

#include "ros_traj_plan/uavtrans.h"
#include "ros_traj_plan/Trajectory.hpp"
#include "ros_traj_plan/TrajectoryLibrary.hpp"
#include "ros_traj_plan/airsimcommon.h"
#include "mavros/frame_tf.h"

class Uav_trajplannimg
{
  public:
	Uav_trajplannimg();

	uavTrans get_trans_msg() const;
	geometry_msgs::PoseStamped get_trans_msg_a() const;

	geometry_msgs::PoseStamped get_destinate_coor() const;
	int get_get_des_() const;

	pcl::PointCloud<pcl::PointXYZ> get_cloudmap() const;

	void pub_waypoint(geometry_msgs::PoseStamped way_point__);

	bool arrive_destination(geometry_msgs::PoseStamped trans_msg_a, geometry_msgs::PoseStamped des_coordinate) const;

	//终点读取轨迹库初始化
	void fly_init();
	void fly_takeoff(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_);
	void fly_traj_plan(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_);

	void mavros_visualization(const ros::TimerEvent &te);
	void fly_position_calibration(geometry_msgs::PoseStamped &test_pose);
	void mavros_auto_land();

	//Airsim
	typedef common_utils::FileSystem FileSystem;
	// typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaterniond;
	// typedef Eigen::Vector3d Vector3d;

	void airsim_set_pose();
	void airsim_set_waypoint(int airsim_way_point_i_);
	void airsim_mavros_pub_waypoint();
	void airsim_init();
	void airsim_takeoff(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_);
	void airsim_traj_plan(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_);
	bool airsim_arrive_destination();
	void airsim_land();
	bool check_arrive_waypoint();
	void airsim_setting_flying_yaw();
	void execute_waypoints();

	void airsim_visualization(const ros::TimerEvent &te);
	//void airsim_image_capture(std::vector<cv::Mat> &img_lr);
	//void airsim_image_dowmload();

	//threads
	void fly_init_thread();
	void fly_takeoff_thread(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_);
	void fly_traj_plan_thread(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_);
	void airsim_image_capture_thread(std::vector<cv::Mat> &img_lr);

  private:
	void call_transmsg(const geometry_msgs::PoseStamped &msg);
	void map_callback(const sensor_msgs::PointCloud2 &map_msg);
	void state_Callback(const mavros_msgs::State::ConstPtr &msg);
	bool res_for_des(auto_flight::destinate::Request &req, auto_flight::destinate::Response &res);

	ros::NodeHandle tp_nh;
	ros::NodeHandle private_nh;
	ros::Publisher waypoint_pub;
	ros::Publisher raw_waypoint_pub;
	ros::Publisher true_way_pub;
	ros::Publisher plan_way_pub;
	ros::Publisher plan_way_marker_pub;
	ros::Publisher map_pub;
	ros::Publisher auto_flight_state_pub;

	//订阅飞机位姿
	ros::Subscriber trans_sub;
	//订阅点云地图
	ros::Subscriber map_sub;
	//creat subscriber  订阅飞控的状态【连接状态／解锁状态／外部控制的标志】
	ros::Subscriber state_sub;

	//终点服务器端
	ros::ServiceServer des_ser;
	//飞机解锁相关服务
	ros::ServiceClient arming_client;
	ros::ServiceClient set_mode_client;

	//订阅飞机姿态数据
	uavTrans trans_msg;
	geometry_msgs::PoseStamped trans_msg_a;
	uavTrans mavros_iter_pose;
	geometry_msgs::PoseStamped start_position;
	mavros_msgs::State current_state;
	//发布航点与轨迹
	geometry_msgs::PoseStamped way_point_;
	mavros_msgs::PositionTarget raw_way_point_;
	float mavros_speed = 2.5f;
	nav_msgs::Path gui_way;
	nav_msgs::Path way_point_path;
	visualization_msgs::Marker way_point_path_maker, way_point_path_maker_6;
	//点云地图信息
	sensor_msgs::PointCloud2 output;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	//终点信息
	geometry_msgs::PoseStamped des_coordinate;
	int get_des_;
	double safe_dis;
	double takeoff_height;

	TrajectoryLibrary TrajLibrarymain; //地面安全距离
	double point_distances;
	double closest_point_distances;
	double current_point_distance;
	double closest_current_point_distance;
	float current_position[3];
	double waypoint8_distances = 0.0;

	clock_t start, finish;

	double totaltime;
	double t_last;
	double thisseartime;
	double wp_last_time;

	int this_tra;
	bool if_replan;
	int this_tra_last;
	double traj_closest_dist;
	Trajectory *farthest_traj = nullptr;
	std::vector<geometry_msgs::PoseStamped> way_point;
	int way_point_i;

	uavTrans trans_msg1;
	geometry_msgs::PoseStamped trans_msg_a1;
	geometry_msgs::PoseStamped des_coordinate_;
	double plan_fly_yaw_;
	double true_fly_yaw_;
	std::tuple<int, double, Trajectory *, std::vector<geometry_msgs::PoseStamped>> chosetraj;

	std::string traj_lib;
	static int straight_pre_traj_no[5];
	static int left_pre_traj_no[5];
	static int right_pre_traj_no[5];
	static int straightl_pre_traj[5];
	static int straightr_pre_traj[5];
	static int left_pre_traj[5];
	static int right_pre_traj[5];
	static int take_off_traj[5];

	ros::Timer mavros_vis_timer,airsim_vis_timer;

	//airsim
	msr::airlib::MultirotorRpcLibClient client;
	geometry_msgs::PoseStamped airsim_start_position;
	Eigen::Vector3d position_ned;
	Eigen::Vector3d position_enu;
	geometry_msgs::PoseStamped airsim_mavros_position;
	Eigen::Vector3d waypoint_enu;
	Eigen::Vector3d waypoint_ned;
	geometry_msgs::PoseStamped airsim_mavros_waypoint;
	Eigen::Quaterniond quaternion_enu;
	Eigen::Quaterniond quaternion_ned;
	uavTrans airsim_pose, airsim_iter_pose;
	Eigen::Vector3d airsim_rpy;
	float airsim_plan_fly_yaw_;
	float airsim_roll, airsim_pitch, airsim_yaw;
	float airsim_x; // current position (ENU coordinate system).
	float airsim_y; // current position (ENU coordinate system).
	float airsim_z; // current position (ENU coordinate system).

	// std::vector<ImageRequest> airsim_img_request ;
	// std::vector<ImageResponse> airsim_img_response;
	std::vector<cv::Mat> lr_depth;
	//std::string file_pat

	float speed = 2.4f;
	const float takeoff_speed = 1.0;
	float max_wait_seconds = 0.25f;
	msr::airlib::DrivetrainType driveTrain = msr::airlib::DrivetrainType::ForwardOnly;
	msr::airlib::YawMode yaw_mode_position;
	float lookahead = -1.0f, adaptive_lookahead = 1.00f;

	static int airsim_straight_pre_traj_no[5];
	static int airsim_left_pre_traj_no[5];
	static int airsim_right_pre_traj_no[5];
	static int airsim_straightl_pre_traj[5];
	static int airsim_straightr_pre_traj[5];
	static int airsim_left_pre_traj[5];
	static int airsim_right_pre_traj[5];
	static int airsim_take_off_traj[5];

	TrajectoryLibrary airsim_TrajLibrarymain;
	std::tuple<int, double, Trajectory *, std::vector<geometry_msgs::PoseStamped>> airsim_chosetraj;
	int airsim_this_tra;
	bool airsim_if_replan;
	int airsim_this_tra_last;
	double airsim_traj_closest_dist;
	Trajectory *airsim_farthest_traj = nullptr;
	std::vector<geometry_msgs::PoseStamped> airsim_way_point;
	int airsim_way_point_i;
	double airsim_point_distances;
	double airsim_closest_point_distances;
	double airsim_current_point_distance;
	double airsim_closest_current_point_distance;
	float airsim_current_position[3];

	double airsim_waypoint8_distances = 0.0;

	clock_t airsim_start, airsim_finish;
	double airsim_totaltime;
	double airsim_t_last;
	double airsim_thisseartime;
	double run_end_straight_time = 0.0, run_end_straight_timestamp = 0.0;

	std::chrono::steady_clock::time_point t_point;
    double delay_time = 0;
};

#endif // !_SEARCHDISTANCE_H