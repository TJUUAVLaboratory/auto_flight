
#include "ros_traj_plan/searchdistance.h"
#include "ros_traj_plan/Trajectory.hpp"
#include "ros_traj_plan/airsimcommon.h"
#include <iostream>

using namespace std;

Uav_trajplannimg::Uav_trajplannimg() : private_nh("~"),
									   closest_point_distances(-1),
									   closest_current_point_distance(-1),
									   airsim_closest_point_distances(-1),
									   airsim_closest_current_point_distance(-1),
									   if_replan(false),
									   airsim_if_replan(false),
									   way_point_i(0),
									   get_des_(0),
									   airsim_way_point_i(0),
									   yaw_mode_position(false, 0),
									   client("")
{
	//closest_point_distances=-1.0;
	//closest_current_point_distance=-1.0;
	//way_point_i=0;

	private_nh.param("safe_dis", safe_dis, 1.5);
	private_nh.param("wp_last_time", wp_last_time, 10.0);
	private_nh.param("traj_lib", traj_lib, std::string("src/auto_flight/trajlib"));
	private_nh.param("takeoff_height", takeoff_height, 5.0);

	gui_way.header.frame_id = "map";
	way_point_path.header.frame_id = "map";
	trans_sub = tp_nh.subscribe("/mavros/local_position/pose", 1, &Uav_trajplannimg::call_transmsg, this);
	map_sub = tp_nh.subscribe("pcl_output_map", 1, &Uav_trajplannimg::map_callback, this);
	state_sub = tp_nh.subscribe("mavros/state", 10, &Uav_trajplannimg::state_Callback, this);

	waypoint_pub = tp_nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
	raw_waypoint_pub = tp_nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

	true_way_pub = tp_nh.advertise<nav_msgs::Path>("/auto_flight/true_guiji_path", 1);
	plan_way_pub = tp_nh.advertise<nav_msgs::Path>("/auto_flight/way_point_path", 1);
	plan_way_marker_pub = tp_nh.advertise<visualization_msgs::Marker>("/auto_flight/way_point_path_maker", 1);

	auto_flight_state_pub = tp_nh.advertise<auto_flight::auto_flight_state>("/auto_flight/mavros/auto_flight_state", 1);
	//map_pub = tp_nh.advertise<sensor_msgs::PointCloud2> ("map_output", 1);

	des_ser = tp_nh.advertiseService("set_destination", &Uav_trajplannimg::res_for_des, this);
	arming_client = tp_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client = tp_nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	mavros_vis_timer = tp_nh.createTimer(ros::Duration(0.3), &Uav_trajplannimg::mavros_visualization, this);
	//airsim_vis_timer = tp_nh.createTimer(ros::Duration(0.3), &Uav_trajplannimg::airsim_visualization, this);
}

/***********mavros**********/

int Uav_trajplannimg::straight_pre_traj_no[5] = {4, 0, 4, 4, 4};
int Uav_trajplannimg::left_pre_traj_no[5] = {4, 0, 1, 1, 4};
int Uav_trajplannimg::right_pre_traj_no[5] = {4, 0, 2, 2, 4};
int Uav_trajplannimg::straightl_pre_traj[5] = {0, 2, 1, 1, 1};
int Uav_trajplannimg::straightr_pre_traj[5] = {0, 1, 2, 2, 2};
int Uav_trajplannimg::left_pre_traj[5] = {6, 1, 1, 1, 1};
int Uav_trajplannimg::right_pre_traj[5] = {6, 2, 2, 2, 2};
int Uav_trajplannimg::take_off_traj[5] = {3, 3, 3, 3, 3};

void Uav_trajplannimg::map_callback(const sensor_msgs::PointCloud2 &map_msg)
{
	pcl::fromROSMsg(map_msg, cloud);
}

void Uav_trajplannimg::state_Callback(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
}

void Uav_trajplannimg::call_transmsg(const geometry_msgs::PoseStamped &msg)
{
	trans_msg_a = msg;
	trans_msg.uav_position[0] = msg.pose.position.x;
	trans_msg.uav_position[1] = msg.pose.position.y;
	trans_msg.uav_position[2] = msg.pose.position.z + 0.1f;
	trans_msg.uav_quat[0] = msg.pose.orientation.w;
	trans_msg.uav_quat[1] = msg.pose.orientation.x,
	trans_msg.uav_quat[2] = msg.pose.orientation.y;
	trans_msg.uav_quat[3] = msg.pose.orientation.z;
	//ROS_INFO_STREAM("get mavros pose");

	/*
	pcl::toROSMsg(*current_cloud_, output); 
	output.header.frame_id = "map";
	map_pub.publish(output);
	*/
}

/**/
bool Uav_trajplannimg::res_for_des(auto_flight::destinate::Request &req, auto_flight::destinate::Response &res)
{
	//ROS_INFO("get destination successfully");
	get_des_ = 1;
	des_coordinate.pose.position.x = req.x;
	des_coordinate.pose.position.y = req.y;
	des_coordinate.pose.position.z = req.z;
	res.des_r = 1;
	//std::cout<<"终点：\n"<<des_coordinate<<std::endl;
	return true;
}

uavTrans Uav_trajplannimg::get_trans_msg() const
{
	return trans_msg;
}
geometry_msgs::PoseStamped Uav_trajplannimg::get_trans_msg_a() const
{
	return trans_msg_a;
}

geometry_msgs::PoseStamped Uav_trajplannimg::get_destinate_coor() const
{
	return des_coordinate;
}

int Uav_trajplannimg::get_get_des_() const
{
	return get_des_;
}

pcl::PointCloud<pcl::PointXYZ> Uav_trajplannimg::get_cloudmap() const
{
	return cloud;
}

void Uav_trajplannimg::pub_waypoint(geometry_msgs::PoseStamped way_point__)
{
	double rpy_[3];
	double quat_[4];
	rpy_[0] = 0.0;
	rpy_[1] = 0.0;
	rpy_[2] = atan2(way_point.at(8).pose.position.y - mavros_iter_pose.uav_position[1], way_point.at(8).pose.position.x - mavros_iter_pose.uav_position[0]);
	//bot_roll_pitch_yaw_to_quat(rpy_, quat_);
	way_point_.pose.position.x = way_point__.pose.position.x;
	way_point_.pose.position.y = way_point__.pose.position.y;
	way_point_.pose.position.z = way_point__.pose.position.z;

	raw_way_point_.position.x = way_point__.pose.position.x;
	raw_way_point_.position.y = way_point__.pose.position.y;
	raw_way_point_.position.z = way_point__.pose.position.z;

	if (rpy_[2] * 180 / 3.1415 != 0)
	{
		way_point_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rpy_[0], rpy_[1], rpy_[2]);
		plan_fly_yaw_ = rpy_[2] * 180 / 3.1415;
	}
	//waypoint_pub.publish(way_point_);

	//速度控制
	double delta_x = way_point.at(8).pose.position.x - mavros_iter_pose.uav_position[0];
	double delta_y = way_point.at(8).pose.position.y - mavros_iter_pose.uav_position[1];
	double delta_z = way_point.at(8).pose.position.z - mavros_iter_pose.uav_position[2];
	double delta_x2 = pow(delta_x, 2.0);
	double delta_y2 = pow(delta_y, 2.0);
	double delta_z2 = pow(delta_z, 2.0);
	double delta_p = sqrt(delta_x2 + delta_y2 + delta_z2);
	raw_way_point_.velocity.x = mavros_speed * delta_x / delta_p;
	raw_way_point_.velocity.y = mavros_speed * delta_y / delta_p;
	raw_way_point_.velocity.z = mavros_speed * delta_z / delta_p;
	raw_way_point_.type_mask = (7 << 6) | (7 << 3); //ignore accelerate and yaw_rate (1 << 11) |
	unsigned char MAV_FRAME = 1;
	raw_way_point_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; //LOCAL_NED
	raw_way_point_.yaw_rate = (plan_fly_yaw_ - true_fly_yaw_) / 60;
	//rpy_[2] -= 3.1415 / 2.0;

	raw_way_point_.yaw = rpy_[2];
	raw_waypoint_pub.publish(raw_way_point_);
}

bool Uav_trajplannimg::arrive_destination(geometry_msgs::PoseStamped trans_msg_a, geometry_msgs::PoseStamped des_coordinate) const
{
	double x_distance = abs(trans_msg_a.pose.position.x - des_coordinate.pose.position.x);
	double y_distance = abs(trans_msg_a.pose.position.y - des_coordinate.pose.position.y);
	double z_distance = abs(trans_msg_a.pose.position.z - des_coordinate.pose.position.z);
	if (x_distance < 2.0 && y_distance < 2.0 && z_distance < 2.0)
	{
		return true;
	}
	else
		return false;
}

void Uav_trajplannimg::fly_position_calibration(geometry_msgs::PoseStamped &test_pose)
{
	waypoint_pub.publish(test_pose);
}

void Uav_trajplannimg::mavros_auto_land()
{
	mavros_msgs::SetMode land_set_mode; //这是一个服务的消息类型　

	land_set_mode.request.custom_mode = "AUTO.LOITER";
	while (current_state.mode != "AUTO.LOITER")
	{
		ROS_INFO_STREAM("The current state of mode is  " << current_state.mode); //打印mode状态

		if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
		{
			break;
		}
	}

	std::cout << "cin to land" << std::endl;
	std::cin.get();

	land_set_mode.request.custom_mode = "AUTO.LAND";

	while (current_state.mode != "AUTO.LAND")
	{
		ROS_INFO_STREAM("The current state of mode is  " << current_state.mode); //打印mode状态

		if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
		{
			break;
		}
	}
}

void Uav_trajplannimg::fly_init()
{
	ROS_INFO_STREAM("MAVROS:please set destination");
	//std::cout<<"please set destination"<<std::endl;
	while (get_des_ != 1)
	{

		ros::spinOnce();
		des_coordinate_ = des_coordinate;
	} //../../../src/auto_flight/trajlib
	while (TrajLibrarymain.LoadLibrary(traj_lib, 0) != 1)
	{
		ros::spinOnce();
	}
	uavTrans start_p = set_trans_init();
	ROS_INFO_STREAM("MAVROS:setting destination straight trajectory");
	//std::cout<<"setting destination straight trajectory"<<std::endl;
	if (TrajLibrarymain.set_desstraight_traj(des_coordinate_, start_p) != true)
	{
		ROS_ERROR_STREAM("MAVROS:failed to set destination straight trajectory");
		//std::cout<<"failed to set destination straight trajectory"<<std::endl;
	}
	TrajLibrarymain.Print();

	std::cout << "\n**************************\ndestination coordinate:\n"
			  << des_coordinate.pose.position << std::endl;

	std::cout << "cin to takeoff" << std::endl;
	std::cin.get();

	ROS_INFO("The current state of connect is %d connect", current_state.connected);

	while (ros::ok() && !current_state.connected) //感觉这个地方有问题，应该是没连接，等待，
	{

		ros::spinOnce();
		//rate.sleep();
	}
	ROS_INFO("MAVROS:he current state of connect is %d connect", current_state.connected);

	//publish some topic
	for (int i = 0; ros::ok() && i < 80; i++)
	{
		delay_msec(50.0);
		ROS_INFO("MAVROS:ros set begin point");
		waypoint_pub.publish(trans_msg_a);
		ros::spinOnce(); //在这个过程中也授控制权，以便订阅话题和服务
	}

	mavros_msgs::SetMode offb_set_mode; //这是一个服务的消息类型　
	offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd; //这个是一个服务的消息类型
	arm_cmd.request.value = true;	 //true是解锁

	//若当前不是　offboard模式　
	while (current_state.mode != "OFFBOARD")
	{
		//  (ros::Time::now()-last_request>ros::Duration(3.0));
		ROS_INFO_STREAM("The current state of mode is  " << current_state.mode); //打印mode状态
		// delay(2);
		if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
		{
			break;
		}
		//发布offboard服务请求，并等待回复成功
	}
	ROS_INFO("Offboard enabled");

	//若当前是为解锁状态   这中间有一个问题，
	while (!current_state.armed)
	{
		ROS_INFO("The current state of arming is %d  disarmed", current_state.armed); //发布解锁服务请求，并等待回复成功
		// delay(2);
		if (arming_client.call(arm_cmd) && arm_cmd.response.success)
		{
			break;
		}
	}
	ROS_INFO("Vehicle armed success");
	ROS_INFO_STREAM("MAVROS:current position:\n"
					<< trans_msg_a);
	ROS_INFO_STREAM("MAVROS:current position:\n"
					<< trans_msg.uav_position[0] << "\n"
					<< trans_msg.uav_position[1] << "\n"
					<< trans_msg.uav_position[2]);
	start_position = trans_msg_a;
	//std::cout<<"当前位置:\n"<< trans_msg_a<<std::endl;
	//std::cout<<"当前位置:\n"<< trans_msg.uav_position[0]<<"\n"<<trans_msg.uav_position[1]<<"\n"<<trans_msg.uav_position[2]<<std::endl;
}

void Uav_trajplannimg::fly_takeoff(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_)
{
	std::tuple<int, double, Trajectory *, std::vector<geometry_msgs::PoseStamped>> takeoff_traj{TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg, octree_map_, take_off_traj)};
	farthest_traj = std::get<2>(takeoff_traj);
	way_point.assign(std::get<3>(takeoff_traj).begin(), std::get<3>(takeoff_traj).end());
	way_point.at(farthest_traj->GetNumberOfPoints() - 1).pose.position.z += takeoff_height;
	this_tra = TrajLibrarymain.GetThisTrajNum();

	way_point.at(farthest_traj->GetNumberOfPoints() - 1).pose.orientation = start_position.pose.orientation;

	while (trans_msg_a1.pose.position.z <= start_position.pose.position.z + takeoff_height)
	{
		ros::spinOnce();
		trans_msg_a1 = trans_msg_a;
		ROS_INFO_STREAM("MAVROS:flying height:" << trans_msg_a1.pose.position.z << "m");
		ROS_INFO_STREAM("MAVROS:choose traj:" << this_tra);
		//std::cout<<"飞行高度:"<<trans_msg_a1.pose.position.z<<"米"<<std::endl;
		//std::cout << "选择轨迹：" << this_tra << endl;
		// if (way_point_i < farthest_traj->GetNumberOfPoints() - 1)
		// {
		// 	way_point_i++;
		// }
		//way_point.at(farthest_traj->GetNumberOfPoints()-1).pose.position.z=6.0f;
		waypoint_pub.publish(way_point.at(farthest_traj->GetNumberOfPoints() - 1));
		delay_msec(10.0);
	}
	//此处要调用回调函数中订阅到的位姿话题和地图话题，进行初始话的选择路径以及检测飞行高度，在达到设定导读之前需要
	//设置一个循环不断的调用 ros::spinOnce();语句来得到位姿，防止程序向下运行

	run_end_straight_timestamp = ros::Time::now().toSec();

	trans_msg_a1 = trans_msg_a;
	mavros_iter_pose = trans_msg;
	if (TrajLibrarymain.set_desstraight_traj(des_coordinate_, trans_msg) != true)
	{
		ROS_ERROR_STREAM("MAVROS:failed to set destination straight trajectory");
		//std::cout<<"failed to set destination straight trajectory"<<std::endl;
	}

	chosetraj = TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg, octree_map_, straight_pre_traj_no); //初始飞行 朝向终点飞行
	this_tra = TrajLibrarymain.GetThisTrajNum();																//路径编号（初始值一般为4）
	traj_closest_dist = std::get<1>(chosetraj);																	//路劲离最近障碍物的距离
	farthest_traj = std::get<2>(chosetraj);																		//得到路径对象（位置坐标，姿态角） 将这个信息传给飞控
	way_point.clear();
	way_point.assign(std::get<3>(chosetraj).begin(), std::get<3>(chosetraj).end());

	way_point_i = 0;
	start = clock();
}

void Uav_trajplannimg::fly_traj_plan(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_)
{
	//delay_msec(wp_last_time); //在发布航点与重新做轨迹选择中间加入延时
	// if (way_point_i < farthest_traj->GetNumberOfPoints() - 1)
	// {
	// 	way_point_i++;
	// }
	way_point_i++;
	ROS_INFO_STREAM("MAVROS:way_point_i:"
					<< way_point_i);

	pub_waypoint(way_point.at(8));
	ROS_INFO_STREAM("MAVROS:publish_way_point:\n"
					<< way_point.at(8).pose.position);
	ROS_INFO_STREAM("MAVROS:publish fly yaw=" << plan_fly_yaw_ << "d");

	//ros::spinOnce();

	double true_rpy[3];
	//bot_quat_to_roll_pitch_yaw(trans_msg.uav_quat, true_rpy);
	true_rpy[2] = tf::getYaw(trans_msg_a.pose.orientation);
	true_fly_yaw_ = true_rpy[2] * 180 / 3.1415;
	ROS_INFO_STREAM("MAVROS:true fly yaw=" << true_fly_yaw_ << "d");
	trans_msg_a1 = trans_msg_a;

	finish = clock();
	totaltime = (float)(finish - start) / CLOCKS_PER_SEC;
	ROS_INFO_STREAM("MAVROS:library running time:" << totaltime << "s!");
	ROS_INFO_STREAM("MAVROS:current position:\n"
					<< trans_msg.uav_position[0] << "\n"
					<< trans_msg.uav_position[1] << "\n"
					<< trans_msg.uav_position[2]);

	current_position[0] = trans_msg.uav_position[0];
	current_position[1] = trans_msg.uav_position[1];
	current_position[2] = trans_msg.uav_position[2];
	current_point_distance = NearestNeighbor(current_position, octree_map_);
	if (current_point_distance != -1.0 && (closest_current_point_distance > current_point_distance || closest_current_point_distance == -1.0))
	{
		closest_current_point_distance = current_point_distance;
	}

	ROS_INFO_STREAM("MAVROS:closest distance current pos to obs=" << current_point_distance << "m");
	ROS_INFO_STREAM("MAVROS:history closest distance current pos to obs=" << closest_current_point_distance << "m");

	if (current_point_distance < 0.6 && current_point_distance > 0)
	{
		ROS_INFO_STREAM("MAVROS:flying dangerous");
		ROS_INFO_STREAM("MAVROS:hover!");

		while (ros::ok() && current_state.mode == "OFFBOARD")
		{
			ros::spinOnce();
			pub_waypoint(trans_msg_a1);
			//ROS_INFO_STREAM("MAVROS:hover!");
		}
		ROS_INFO_STREAM("MAVROS:waiting to switch to offboard!");
		while (ros::ok() && current_state.mode != "OFFBOARD")
		{
			ros::spinOnce();
			ROS_INFO_STREAM("MAVROS:waiting to switch to offboard!");
		}
		fly_init();
		//切换成手动，不要让程序终止,手动调整之后能切回来,要重新初始化连接
	}

	if (ros::ok() && current_state.mode != "OFFBOARD")
	{
		ROS_INFO_STREAM("MAVROS:waiting to switch to offboard!");
		while (ros::ok() && current_state.mode != "OFFBOARD")
		{
			ros::spinOnce();
			//ROS_INFO_STREAM("MAVROS:waiting to switch to offboard!");
		}
	}

	point_distances = farthest_traj->ClosestObstacleInRemainderOfTrajectory(totaltime, mavros_iter_pose, octree_map_, 0); //想一想这个时间怎么去计时利用time函数？优化  检测最后一点
	if (point_distances != -1.0 && (closest_point_distances > point_distances || closest_point_distances == -1.0))
	{
		closest_point_distances = point_distances;
	}
	ROS_INFO_STREAM("MAVROS:current traj:" << this_tra);
	ROS_INFO_STREAM("MAVROS:if_replan:" << if_replan);
	ROS_INFO_STREAM("MAVROS:time of this library search:" << thisseartime * 1000 << "ms!");

	auto_flight::auto_flight_state auto_flight_state;
	auto_flight_state.header.stamp = ros::Time::now();
	auto_flight_state.traj_point_distance = point_distances;
	auto_flight_state.closest_traj_point_distance = closest_point_distances;
	auto_flight_state.last_traj = this_tra_last;
	auto_flight_state.if_replan = if_replan;
	auto_flight_state.current_traj = this_tra;
	auto_flight_state.way_point_i = way_point_i;
	auto_flight_state.way_point = way_point.at(8);
	auto_flight_state.plan_fly_yaw = plan_fly_yaw_;
	auto_flight_state.current_pose = trans_msg_a;
	auto_flight_state.true_fly_yaw = true_fly_yaw_;
	auto_flight_state.current_point_distance = current_point_distance;
	auto_flight_state.closest_current_point_distance = closest_current_point_distance;
	auto_flight_state_pub.publish(auto_flight_state);

	ROS_INFO_STREAM("\n\n\n\n***********************  iteration  ****************************");
	ROS_INFO_STREAM("MAVROS:closest distance current traj point to obs=" << point_distances << "m");
	ROS_INFO_STREAM("MAVROS:history closest distance current traj point to obs=" << closest_point_distances << "m");

	if_replan = false;
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;

	//有障碍物
	if (point_distances < safe_dis && point_distances > 0)
	{
		if (this_tra == 0 || this_tra == 4 || this_tra == 3 || this_tra == 6) //悬停部分可以放在这里
		{
			if (this_tra_last == 1 || this_tra_last == 4 || this_tra_last == 0 || this_tra_last == 6)
			{

				if (TrajLibrarymain.set_desstraight_traj(des_coordinate_, trans_msg) != true)
				{
					ROS_ERROR_STREAM("MAVROS:failed to set destination straight trajectory");
				}
				ROS_INFO_STREAM("MAVROS:have obastacle straight(last left)");
				chosetraj = TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg, octree_map_, straightl_pre_traj);
				//上次左飞，这次直线飞行后优先右飞
			}

			else
			{
				if (TrajLibrarymain.set_desstraight_traj(des_coordinate_, trans_msg) != true)
				{
					ROS_ERROR_STREAM("MAVROS:failed to set destination straight trajectory");
				}
				ROS_INFO_STREAM("MAVROS:have obastacle straight(last right)");
				chosetraj = TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg, octree_map_, straightr_pre_traj);
				//上次右飞，这次直线飞行后优先左飞
			}
		}
		else if (this_tra == 1)
		{
			if (TrajLibrarymain.set_desstraight_traj(des_coordinate_, trans_msg) != true)
			{
				ROS_ERROR_STREAM("MAVROS:failed to set destination straight trajectory");
			}
			ROS_INFO_STREAM("MAVROS:have obastacle left");
			chosetraj = TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg, octree_map_, left_pre_traj);
		}
		else
		{
			if (TrajLibrarymain.set_desstraight_traj(des_coordinate_, trans_msg) != true)
			{
				ROS_ERROR_STREAM("MAVROS:failed to set destination straight trajectory");
			}
			ROS_INFO_STREAM("MAVROS:have obastacle right");
			chosetraj = TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg, octree_map_, right_pre_traj);
		}
		this_tra_last = this_tra; //上次的路劲编号
		mavros_iter_pose = trans_msg;
		trans_msg_a1 = trans_msg_a;
		this_tra = TrajLibrarymain.GetThisTrajNum();
		traj_closest_dist = std::get<1>(chosetraj); //路劲离最近障碍物的距离
		farthest_traj = std::get<2>(chosetraj);		//得到路径对象（位置坐标，姿态角） 将这个信息传给飞控，利用ROS消息的形式
		way_point.clear();
		way_point.assign(std::get<3>(chosetraj).begin(), std::get<3>(chosetraj).end());
		//在获取路径之后记录一下此时的无人机位置以及姿态角，
		//并将轨迹中的数据全部转化成地面坐标系的数据存入到x_ground_point数组当中
		t_last = totaltime;
		finish = clock();
		totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
		thisseartime = totaltime - t_last;

		ROS_INFO_STREAM("MAVROS:replan traj:" << this_tra);
		if_replan = true;

		start = clock();
		way_point_i = 0;
	}

	//一直无障碍
	//比较当前时刻和轨迹库中的时刻的最大值（或者是此时无人机的位置与之前转化的x_ground_point中的差值小于设定误差值），
	//若大于，则选择无障碍物的时的选择
	else if (way_point_i >= 8 && totaltime > 0.4 / mavros_speed) //(farthest_traj->GetNumberOfPoints()-1)  && point_distances>safe_dis
	{
		if (this_tra == 1)
		{

			if (TrajLibrarymain.set_desstraight_traj(des_coordinate_, trans_msg) != true)
			{
				ROS_ERROR_STREAM("MAVROS:failed to set destination straight trajectory");
			}
			ROS_INFO_STREAM("MAVROS:no obastacle left");
			chosetraj = TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg, octree_map_, left_pre_traj_no);
		}
		else if (this_tra == 2)
		{

			if (TrajLibrarymain.set_desstraight_traj(des_coordinate_, trans_msg) != true)
			{
				ROS_ERROR_STREAM("MAVROS:failed to set destination straight trajectory");
			}
			ROS_INFO_STREAM("MAVROS:no obastacle rihgt");
			chosetraj = TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg, octree_map_, right_pre_traj_no);
		}
		else
		{

			if (TrajLibrarymain.set_desstraight_traj(des_coordinate_, trans_msg) != true)
			{
				ROS_ERROR_STREAM("MAVROS:failed to set destination straight trajectory");
			}
			ROS_INFO_STREAM("MAVROS:no obastacle straight end or straight");
			chosetraj = TrajLibrarymain.FindFarthestTrajectory(safe_dis, trans_msg, octree_map_, straight_pre_traj_no);
			//return 0;
		}

		this_tra_last = this_tra; //上次的路劲编号
		mavros_iter_pose = trans_msg;
		trans_msg_a1 = trans_msg_a;
		this_tra = TrajLibrarymain.GetThisTrajNum();
		traj_closest_dist = std::get<1>(chosetraj); //路劲离最近障碍物的距离
		farthest_traj = std::get<2>(chosetraj);		//得到路径对象（位置坐标，姿态角） 将这个信息传给飞控，利用ROS消息的形式
		way_point.clear();
		way_point.assign(std::get<3>(chosetraj).begin(), std::get<3>(chosetraj).end());

		t_last = totaltime;
		finish = clock();
		totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
		thisseartime = totaltime - t_last;

		ROS_INFO_STREAM("MAVROS:relpan traj:" << this_tra);
		if_replan = true;

		start = clock();
		way_point_i = 0;
	}

	if (this_tra != 4)
	{
		run_end_straight_timestamp = ros::Time::now().toSec();
		mavros_speed = 4.0;
		ROS_INFO_STREAM("flying speed:" << mavros_speed);

		if (trans_msg.uav_position[2] > start_position.pose.position.z + takeoff_height + 0.5)
		{
			way_point.at(8).pose.position.z = start_position.pose.position.z + takeoff_height + 0.5;
		}
	}
	else if (ros::Time::now().toSec() - run_end_straight_timestamp >= 4.0)
	{
		run_end_straight_time = ros::Time::now().toSec() - run_end_straight_timestamp;
		way_point.at(8).pose.position.z = start_position.pose.position.z + takeoff_height;
		ROS_INFO_STREAM(setprecision(4) << "MAVROS:run_end_straight_time:" << run_end_straight_time << "s");
		mavros_speed = 4.0;
		ROS_INFO_STREAM("flying speed:" << mavros_speed);
	}
	else
	{
		ROS_INFO_STREAM("flying speed:" << mavros_speed);
	}

	if (trans_msg.uav_position[2] < start_position.pose.position.z + takeoff_height - 0.5)
	{
		way_point.at(8).pose.position.z = start_position.pose.position.z + takeoff_height;
	}

	ROS_INFO_STREAM("MAVROS:Cycle once done");
}

void Uav_trajplannimg::mavros_visualization(const ros::TimerEvent &te)
{
	// way_point_path.poses.push_back(way_point_);
	// way_point_path.header.frame_id = "map";
	// plan_way_pub.publish(way_point_path);
	//pan_path

	gui_way.poses.push_back(trans_msg_a1);
	gui_way.header.frame_id = "map";
	true_way_pub.publish(gui_way);
	//true_path

	if (this_tra == 6)
	{
		way_point_path_maker_6.header.frame_id = "map";
		way_point_path_maker_6.header.stamp = ros::Time::now();
		way_point_path_maker_6.ns = "plan_waypoint";
		way_point_path_maker_6.action = visualization_msgs::Marker::ADD;
		way_point_path_maker_6.pose.orientation.w = 1.0;
		way_point_path_maker_6.type = visualization_msgs::Marker::LINE_LIST;
		way_point_path_maker_6.lifetime = ros::Duration(0.0);
		way_point_path_maker_6.scale.x = 0.04;
		way_point_path_maker_6.id = 2;
		way_point_path_maker_6.color.g = 0.0f;
		way_point_path_maker_6.color.b = 1.0f;
		way_point_path_maker_6.color.a = 1.0;
		way_point_path_maker_6.points.push_back(trans_msg_a1.pose.position);
		way_point_path_maker_6.points.push_back(way_point_.pose.position);
		plan_way_marker_pub.publish(way_point_path_maker_6);
	}
	else
	{
		way_point_path_maker.header.frame_id = "map";
		way_point_path_maker.header.stamp = ros::Time::now();
		way_point_path_maker.ns = "plan_waypoint";
		way_point_path_maker.action = visualization_msgs::Marker::ADD;
		way_point_path_maker.pose.orientation.w = 1.0;
		way_point_path_maker.type = visualization_msgs::Marker::LINE_LIST;
		way_point_path_maker.lifetime = ros::Duration(0.0);
		way_point_path_maker.scale.x = 0.04;
		way_point_path_maker.id = 1;
		way_point_path_maker.color.g = 1.0f;
		way_point_path_maker.color.b = 0.0f;
		way_point_path_maker.color.a = 1.0;
		way_point_path_maker.points.push_back(trans_msg_a1.pose.position);
		way_point_path_maker.points.push_back(way_point_.pose.position);
		plan_way_marker_pub.publish(way_point_path_maker);
	}
}

/*********Airsim*********/

int Uav_trajplannimg::airsim_straight_pre_traj_no[5] = {5, 5, 5, 0, 5};
int Uav_trajplannimg::airsim_left_pre_traj_no[5] = {5, 0, 1, 5, 5};
int Uav_trajplannimg::airsim_right_pre_traj_no[5] = {5, 0, 2, 5, 5};  //
int Uav_trajplannimg::airsim_straightl_pre_traj[5] = {0, 2, 1, 1, 2}; ///
int Uav_trajplannimg::airsim_straightr_pre_traj[5] = {0, 1, 2, 1, 1};
int Uav_trajplannimg::airsim_left_pre_traj[5] = {6, 1, 1, 1, 1};
int Uav_trajplannimg::airsim_right_pre_traj[5] = {6, 2, 2, 2, 2};
int Uav_trajplannimg::airsim_take_off_traj[5] = {3, 3, 3, 3, 3};

void Uav_trajplannimg::airsim_set_pose()
{
	position_ned = Eigen::Vector3d(client.getPosition().x(), client.getPosition().y(), client.getPosition().z());
	position_enu = mavros::ftf::transform_frame_ned_enu(position_ned);
	quaternion_ned = Eigen::Quaterniond(client.getOrientation().w(), client.getOrientation().x(), client.getOrientation().y(), client.getOrientation().z());
	quaternion_enu = mavros::ftf::transform_orientation_ned_enu(mavros::ftf::transform_orientation_aircraft_baselink(quaternion_ned));
	// airsim_x = position_enu.x();
	// airsim_y = position_enu.y();
	// airsim_z = position_enu.z();
	// ROS_INFO_STREAM( "airsim flight position:\nairsim_x:" << airsim_x
	//           << "\nairsim_y:" << airsim_y
	//           << "\nairsim_z:" << airsim_z);
	// std::cout << "airsim flight position:\nairsim_x:" << airsim_x
	//           << "\nairsim_y:" << airsim_y
	//           << "\nairsim_z:" << airsim_z << std::endl;
	airsim_rpy = mavros::ftf::quaternion_to_rpy(quaternion_enu);
	airsim_roll = airsim_rpy(0);
	airsim_pitch = airsim_rpy(1);
	airsim_yaw = airsim_rpy(2);
	// ROS_INFO_STREAM( "airsim flight oritation(ENU):\nairsim_roll:" << airsim_roll * 180 / 3.1415
	// 		  << "\nairsim_pitch:" << airsim_pitch * 180 / 3.1415
	// 		  << "\nairsim_yaw:" << airsim_yaw * 180 / 3.1415);
	// std::cout << "airsim flight oritation(ENU):\nairsim_roll:" << airsim_roll * 180 / 3.1415
	// 		  << "\nairsim_pitch:" << airsim_pitch * 180 / 3.1415
	// 		  << "\nairsim_yaw:" << airsim_yaw * 180 / 3.1415 << std::endl;
	airsim_pose.uav_position[0] = position_enu.x();
	airsim_pose.uav_position[1] = position_enu.y();
	airsim_pose.uav_position[2] = position_enu.z() + 0.0f;
	airsim_pose.uav_quat[0] = quaternion_enu.w();
	airsim_pose.uav_quat[1] = quaternion_enu.x();
	airsim_pose.uav_quat[2] = quaternion_enu.y();
	airsim_pose.uav_quat[3] = quaternion_enu.z();

	//airsim guiji
	airsim_mavros_position.pose.position.x = position_enu.x();
	airsim_mavros_position.pose.position.y = position_enu.y();
	airsim_mavros_position.pose.position.z = position_enu.z();
	airsim_mavros_position.pose.orientation.w = quaternion_enu.w();
	airsim_mavros_position.pose.orientation.x = quaternion_enu.x();
	airsim_mavros_position.pose.orientation.y = quaternion_enu.y();
	airsim_mavros_position.pose.orientation.z = quaternion_enu.z();
}

void Uav_trajplannimg::airsim_set_waypoint(int airsim_way_point_i_)
{
	waypoint_enu.x() = airsim_way_point.at(airsim_way_point_i_).pose.position.x;
	waypoint_enu.y() = airsim_way_point.at(airsim_way_point_i_).pose.position.y;
	waypoint_enu.z() = airsim_way_point.at(airsim_way_point_i_).pose.position.z;
	waypoint_ned = mavros::ftf::transform_frame_enu_ned(waypoint_enu);
}

void Uav_trajplannimg::airsim_mavros_pub_waypoint()
{
	airsim_mavros_waypoint.pose.position.x = waypoint_enu.x();
	airsim_mavros_waypoint.pose.position.y = waypoint_enu.y();
	airsim_mavros_waypoint.pose.position.z = waypoint_enu.z();
	airsim_mavros_waypoint.pose.orientation.w = quaternion_enu.w();
	airsim_mavros_waypoint.pose.orientation.x = quaternion_enu.x();
	airsim_mavros_waypoint.pose.orientation.y = quaternion_enu.y();
	airsim_mavros_waypoint.pose.orientation.z = quaternion_enu.z();

	//waypoint_pub.publish(airsim_mavros_waypoint);

	way_point_path.poses.push_back(airsim_mavros_waypoint);
	way_point_path.header.frame_id = "map";
	plan_way_pub.publish(way_point_path);
}

void Uav_trajplannimg::airsim_init()
{
	ROS_INFO_STREAM("waitting for airsim connect");
	client.confirmConnection();
	ROS_INFO_STREAM("AIRSIM:please set destination!");
	while (get_des_ != 1)
	{
		ros::spinOnce();
		des_coordinate_ = des_coordinate;
	}

	while (airsim_TrajLibrarymain.LoadLibrary(traj_lib, 0) != 1)
	{
		airsim_set_pose();
	}
	uavTrans start_p = set_trans_init();

	ROS_INFO_STREAM("destination coordinate:\n"
					<< des_coordinate_);
	ROS_INFO_STREAM("AIRSIM:setting destination straight trajectory");
	//std::cout << "setting destination straight trajectory" << std::endl;
	if (airsim_TrajLibrarymain.airsim_set_desstraight_traj(des_coordinate_, start_p) != true)
	{
		ROS_ERROR_STREAM("AIRSIM:failed to set destination straight trajectory");
	}
	ROS_INFO_STREAM("AIRSIM:setting destination straight trajectory successfully");
	airsim_TrajLibrarymain.Print();

	client.enableApiControl(true);
	client.armDisarm(true);
	airsim_set_pose();
	airsim_start_position.pose.position.x = airsim_mavros_position.pose.position.x;
	airsim_start_position.pose.position.y = airsim_mavros_position.pose.position.y;
	airsim_start_position.pose.position.z = airsim_mavros_position.pose.position.z;
}

void Uav_trajplannimg::airsim_takeoff(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_)
{
	// std::cout << "takeoff" << std::endl;
	// //std::cin.get();
	// float takeoffTimeout = 3;
	// client.takeoff(takeoffTimeout);

	// client.hover();

	// airsim_chosetraj = airsim_TrajLibrarymain.FindFarthestTrajectory(safe_dis, airsim_pose, octree_map_, airsim_take_off_traj); //初始飞行 朝向终点飞行
	// airsim_this_tra = std::get<0>(airsim_chosetraj);
	// airsim_farthest_traj = std::get<2>(airsim_chosetraj);
	// airsim_way_point.assign(std::get<3>(airsim_chosetraj).begin(), std::get<3>(airsim_chosetraj).end());
	// waypoint_enu.x() = airsim_way_point.at(airsim_farthest_traj->GetNumberOfPoints() - 1).pose.position.x;
	// waypoint_enu.y() = airsim_way_point.at(airsim_farthest_traj->GetNumberOfPoints() - 1).pose.position.y;
	// waypoint_enu.z() = airsim_way_point.at(airsim_farthest_traj->GetNumberOfPoints() - 1).pose.position.z + takeoff_height;
	// // waypoint_enu.x() = 0.0f;
	// // waypoint_enu.y() = 0.0f;
	// //waypoint_enu.z() = 4.5f;

	// waypoint_ned = mavros::ftf::transform_frame_enu_ned(waypoint_enu);
	// do
	// {
	// 	airsim_set_pose();

	// 	ROS_INFO_STREAM("AIRSIM:choose traj:" << airsim_this_tra);
	// 	ROS_INFO_STREAM("AIRSIM:flying height:" << airsim_mavros_position.pose.position.z << "m");
	// 	ROS_INFO_STREAM("AIRSIM:publish height" << waypoint_enu.z() << "m");
	// 	//ROS_INFO_STREAM("AIRSIM:publish height(NED)"<<waypoint_ned.z()<<"m");

	// 	// std::cout << "AIRSIM: flying height:" << position_enu.z() << "米" << std::endl;
	// 	// std::cout << "AIRSIM: choose traj:" << airsim_this_tra << endl;
	// 	// if (airsim_way_point_i < airsim_farthest_traj->GetNumberOfPoints() - 1)
	// 	// {
	// 	// 	airsim_way_point_i++;
	// 	// }

	// 	client.moveToPosition(waypoint_ned.x(), waypoint_ned.y(), waypoint_ned.z(), takeoff_speed, max_wait_seconds + 0.4f, driveTrain, yaw_mode_position, lookahead, adaptive_lookahead);
	// 	//delay_msec(10.0);
	// } while (airsim_pose.uav_position[2] <= airsim_start_position.pose.position.z + takeoff_height); //airsim_start_position.pose.position.z +

	waypoint_enu.x() = airsim_start_position.pose.position.x;
	waypoint_enu.y() = airsim_start_position.pose.position.y;
	waypoint_enu.z() = airsim_start_position.pose.position.z + takeoff_height;

	waypoint_ned = mavros::ftf::transform_frame_enu_ned(waypoint_enu);

	//悬停5s
	ROS_INFO_STREAM("hover for some time! then fly to destination!");
	ros::Time time_current = ros::Time::now();
	while (ros::Time::now() - time_current <= ros::Duration(12))
	{
		client.moveToPosition(waypoint_ned.x(), waypoint_ned.y(), waypoint_ned.z(), takeoff_speed, max_wait_seconds, driveTrain, yaw_mode_position, lookahead, adaptive_lookahead);
		airsim_set_pose();
		ROS_INFO_STREAM("AIRSIM:flying height:" << airsim_mavros_position.pose.position.z << "m");
		client.enableApiControl(true);
	}

	airsim_set_pose();
	airsim_iter_pose = airsim_pose;
	run_end_straight_timestamp = ros::Time::now().toSec();

	if (airsim_TrajLibrarymain.airsim_set_desstraight_traj(des_coordinate_, airsim_pose) != true)
	{
		ROS_ERROR_STREAM("AIRSIM:failed to set destination straight trajectory");
		//std::cout << "airsim failed to set destination straight trajectory" << std::endl;
	}
	airsim_chosetraj = airsim_TrajLibrarymain.FindFarthestTrajectory(safe_dis, airsim_pose, octree_map_, airsim_straight_pre_traj_no); //初始飞行 朝向终点飞行
	airsim_this_tra = std::get<0>(airsim_chosetraj);																				   //路径编号（初始值一般为4）
	airsim_traj_closest_dist = std::get<1>(airsim_chosetraj);																		   //路劲离最近障碍物的距离
	airsim_farthest_traj = std::get<2>(airsim_chosetraj);																			   //得到路径对象（位置坐标，姿态角） 将这个信息传给飞控
	airsim_way_point.clear();
	airsim_way_point.assign(std::get<3>(airsim_chosetraj).begin(), std::get<3>(airsim_chosetraj).end());

	airsim_way_point_i = 0;
	airsim_start = clock();
}

void Uav_trajplannimg::airsim_traj_plan(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_)
{
	// if (airsim_way_point_i < airsim_farthest_traj->GetNumberOfPoints() - 1)
	// {
	// 	airsim_way_point_i++;
	// }
	//airsim_image_capture(img_lr);
	//airsim_TrajLibrarymain.Print();
	airsim_way_point_i++;
	airsim_set_waypoint(8);
	airsim_setting_flying_yaw();
	//client.moveToPosition(waypoint_ned.x(), waypoint_ned.y(), waypoint_ned.z(), speed, max_wait_seconds, driveTrain, yaw_mode_position, lookahead, adaptive_lookahead);
	//std::this_thread::sleep_for(std::chrono::duration<double>(max_wait_seconds));
	ROS_INFO_STREAM("AIRSIM:publish fly yaw=" << yaw_mode_position.yaw_or_rate * 180 / 3.1415 << "d");

	//client.hover();
	ROS_INFO_STREAM("AIRSIM:way_point_i:" << airsim_way_point_i);
	//std::cout << "airsim_way_point_i:\n"<< airsim_way_point_i << std::endl;

	airsim_finish = clock();
	airsim_totaltime = (float)(airsim_finish - airsim_start) / CLOCKS_PER_SEC;
	ROS_INFO_STREAM("AIRSIM:library running time:" << airsim_totaltime << "s!");

	// ROS_INFO_STREAM("AIRSIM:publish waypoint(NED):\npublish_x:" <<waypoint_ned.x()
	//           << "\npublish_y:" <<waypoint_ned.y()
	//           << "\npublish_z:" <<waypoint_ned.z());

	ROS_INFO_STREAM("AIRSIM:flight oritation(ENU):\nairsim_roll:" << airsim_roll * 180 / 3.1415
																  << "\nairsim_pitch:" << airsim_pitch * 180 / 3.1415
																  << "\nairsim_yaw:" << airsim_yaw * 180 / 3.1415);

	ROS_INFO_STREAM("AIRSIM:current position:\n"
					<< airsim_mavros_position.pose.position.x << "\n"
					<< airsim_mavros_position.pose.position.y << "\n"
					<< airsim_mavros_position.pose.position.z);

	t_point = chrono::steady_clock::now();

	airsim_set_pose(); //重新获取位子信息

	delay_time = (delay_time + chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - t_point).count() * 1000);

	ROS_INFO_STREAM("AIRSIM:publish waypoint:\npublish_x:" << waypoint_enu.x()
														   << "\npublish_y:" << waypoint_enu.y()
														   << "\npublish_z:" << waypoint_enu.z());

	// cout << "airsim轨迹库运行时间：" << airsim_totaltime << "秒！" << endl;
	// std::cout << "airsim初始位置:\n"
	// 		  << airsim_start_position.pose.position.x << "\n"
	// 		  << airsim_start_position.pose.position.y<< "\n"
	// 		  << airsim_start_position.pose.position.z << std::endl;

	airsim_current_position[0] = airsim_pose.uav_position[0];
	airsim_current_position[1] = airsim_pose.uav_position[1];
	airsim_current_position[2] = airsim_pose.uav_position[2];
	airsim_current_point_distance = NearestNeighbor(airsim_current_position, octree_map_);
	if (airsim_current_point_distance != -1.0 && (airsim_closest_current_point_distance > airsim_current_point_distance || airsim_closest_current_point_distance == -1.0))
	{
		airsim_closest_current_point_distance = airsim_current_point_distance;
	}
	ROS_INFO_STREAM("AIRSIM:closest distance current pos to obs=" << airsim_current_point_distance << "m");
	ROS_INFO_STREAM("AIRSIM:history closest distance current pos to obs=" << airsim_closest_current_point_distance << "m");
	// std::cout << "airsim当前位置点离障碍物最近距离=" << airsim_current_point_distance << "米" << std::endl;
	// std::cout << "airsim历史当前位置点离障碍物最近距离=" << airsim_closest_current_point_distance << "米\n"
	// 		  << std::endl;

	if (airsim_current_point_distance < 0.6 && airsim_current_point_distance > 0)
	{
		ROS_ERROR_STREAM("AIRSIM:flying dangerous");

		std::cout << "Press Enter to go home" << std::endl;
		std::cin.get();
		client.reset();

		std::cout << "Press Enter to disarm" << std::endl;
		std::cin.get();
		client.armDisarm(false);
		airsim_init();
		//切换成手动，不要让程序终止,手动调整之后能切回来,要重新初始化连接
	}
	//airsim_waypoint8_distances =
	airsim_point_distances = airsim_farthest_traj->ClosestObstacleInRemainderOfTrajectory(airsim_totaltime, airsim_iter_pose, octree_map_, 0); //想一想这个时间怎么去计时利用time函数？优化  检测最后一点
	if (airsim_point_distances != -1.0 && (airsim_closest_point_distances > airsim_point_distances || airsim_closest_point_distances == -1.0))
	{
		airsim_closest_point_distances = airsim_point_distances;
	}
	//std::min();
	ROS_INFO_STREAM("AIRSIM:current traj:" << airsim_this_tra);
	ROS_INFO_STREAM("AIRSIM:if_replan:" << airsim_if_replan);
	ROS_INFO_STREAM("AIRSIM:time of this library search:" << airsim_thisseartime << "s!");

	auto_flight::auto_flight_state airsim_auto_flight_state;
	airsim_auto_flight_state.header.stamp = ros::Time::now();
	airsim_auto_flight_state.traj_point_distance = airsim_point_distances;
	airsim_auto_flight_state.closest_traj_point_distance = airsim_closest_point_distances;
	airsim_auto_flight_state.last_traj = airsim_this_tra_last;
	airsim_auto_flight_state.if_replan = airsim_if_replan;
	airsim_auto_flight_state.current_traj = airsim_this_tra;
	airsim_auto_flight_state.way_point_i = airsim_way_point_i;
	airsim_auto_flight_state.way_point = airsim_way_point.at(8);
	//airsim_auto_flight_state.plan_fly_yaw = plan_fly_yaw_;
	//airsim_auto_flight_state.current_pose = trans_msg_a;
	//airsim_auto_flight_state.true_fly_yaw = true_fly_yaw_;
	airsim_auto_flight_state.current_point_distance = airsim_current_point_distance;
	airsim_auto_flight_state.closest_current_point_distance = airsim_closest_current_point_distance;
	auto_flight_state_pub.publish(airsim_auto_flight_state);

	ROS_INFO_STREAM("\n\n\n\n***********************  iteration  ****************************");
	ROS_INFO_STREAM("AIRSIM:closest distance current traj point to obs=" << airsim_point_distances << "m");
	ROS_INFO_STREAM("AIRSIM:closest history distance current traj point to obs=" << airsim_closest_point_distances << "m");
	// std::cout << "airsim当前运行轨迹离障碍物离最近距离=" << airsim_point_distances << "米" << std::endl;
	// cout << "airsim当前轨迹：" << airsim_this_tra << endl;
	// cout << "airsim此次轨迹库距离搜索时间：" << airsim_thisseartime << "秒！" << endl;
	// std::cout << "airsim历史选择轨迹离障碍物最近距离=" << airsim_closest_point_distances << "米" << std::endl;

	airsim_if_replan = false;

	//有障碍物
	if (airsim_point_distances < safe_dis && airsim_point_distances > 0)
	{
		if (airsim_this_tra == 0 || airsim_this_tra == 5 || airsim_this_tra == 3 || airsim_this_tra == 6) //悬停部分可以放在这里
		{
			if (airsim_this_tra_last = 1 || airsim_this_tra_last == 5 || airsim_this_tra_last == 0 || airsim_this_tra == 6)
			{
				//airsim_set_pose();

				if (airsim_TrajLibrarymain.airsim_set_desstraight_traj(des_coordinate_, airsim_pose) != true)
				{
					ROS_ERROR_STREAM("AIRSIM:failed to set destination straight trajectory");
					//std::cout << "airsim failed to set destination straight trajectory" << std::endl;
				}
				ROS_INFO_STREAM("AIRSIM:have obastacle straight(last left)");
				airsim_chosetraj = airsim_TrajLibrarymain.FindFarthestTrajectory(safe_dis, airsim_pose, octree_map_, airsim_straightl_pre_traj);
				//上次左飞，这次直线飞行后优先右飞
			}

			else
			{
				//airsim_set_pose();

				if (airsim_TrajLibrarymain.airsim_set_desstraight_traj(des_coordinate_, airsim_pose) != true)
				{
					ROS_ERROR_STREAM("AIRSIM:failed to set destination straight trajectory");
					//std::cout << "airsim failed to set destination straight trajectory" << std::endl;
				}
				ROS_INFO_STREAM("AIRSIM:have obastacle straight(last right)");
				airsim_chosetraj = airsim_TrajLibrarymain.FindFarthestTrajectory(safe_dis, airsim_pose, octree_map_, airsim_straightr_pre_traj);
				//上次右飞，这次直线飞行后优先左飞
			}
		}
		else if (airsim_this_tra == 1)
		{
			//airsim_set_pose();

			if (airsim_TrajLibrarymain.airsim_set_desstraight_traj(des_coordinate_, airsim_pose) != true)
			{
				ROS_ERROR_STREAM("AIRSIM:failed to set destination straight trajectory");
				//std::cout << "airsim failed to set destination straight trajectory" << std::endl;
			}
			ROS_INFO_STREAM("AIRSIM:have obastacle left");
			airsim_chosetraj = airsim_TrajLibrarymain.FindFarthestTrajectory(safe_dis, airsim_pose, octree_map_, airsim_left_pre_traj);
		}
		else
		{
			//airsim_set_pose();

			if (airsim_TrajLibrarymain.airsim_set_desstraight_traj(des_coordinate_, airsim_pose) != true)
			{
				ROS_ERROR_STREAM("AIRSIM:failed to set destination straight trajectory");
				//std::cout << "airsim failed to set destination straight trajectory" << std::endl;
			}
			ROS_INFO_STREAM("AIRSIM:have obastacle right");
			airsim_chosetraj = airsim_TrajLibrarymain.FindFarthestTrajectory(safe_dis, airsim_pose, octree_map_, airsim_right_pre_traj);
		}
		airsim_this_tra_last = airsim_this_tra; //上次的路劲编号
		airsim_iter_pose = airsim_pose;
		airsim_this_tra = std::get<0>(airsim_chosetraj);
		airsim_traj_closest_dist = std::get<1>(airsim_chosetraj); //路劲离最近障碍物的距离
		airsim_farthest_traj = std::get<2>(airsim_chosetraj);	 //得到路径对象（位置坐标，姿态角） 将这个信息传给飞控，利用ROS消息的形式
		airsim_way_point.clear();
		airsim_way_point.assign(std::get<3>(airsim_chosetraj).begin(), std::get<3>(airsim_chosetraj).end());
		//在获取路径之后记录一下此时的无人机位置以及姿态角，
		//并将轨迹中的数据全部转化成地面坐标系的数据存入到x_ground_point数组当中
		airsim_t_last = airsim_totaltime;
		airsim_finish = clock();
		airsim_totaltime = (double)(airsim_finish - airsim_start) / CLOCKS_PER_SEC;
		airsim_thisseartime = airsim_totaltime - airsim_t_last;

		ROS_INFO_STREAM("AIRSIM:replan traj:" << airsim_this_tra);
		airsim_mavros_pub_waypoint();
		airsim_if_replan = true;
		//cout << "airsim重新选择轨迹：" << airsim_this_tra << endl;
		//cout << "此次轨迹库距离搜索时间：" << thisseartime << "秒！" << endl;

		airsim_start = clock();
		airsim_way_point_i = 0;
	}

	//一直无障碍
	//比较当前时刻和轨迹库中的时刻的最大值（或者是此时无人机的位置与之前转化的x_ground_point中的差值小于设定误差值），
	//若大于，则选择无障碍物的时的选择
	else if (airsim_way_point_i >= 8 && airsim_totaltime > 0.35) // airsim_way_point_i >= 8 &&check_arrive_waypoint() == true
	{
		if (airsim_this_tra == 1)
		{
			//airsim_set_pose();

			if (airsim_TrajLibrarymain.airsim_set_desstraight_traj(des_coordinate_, airsim_pose) != true)
			{
				ROS_ERROR_STREAM("AIRSIM:failed to set destination straight trajectory");
				//std::cout << "airsim failed to set destination straight trajectory" << std::endl;
			}
			ROS_INFO_STREAM("AIRSIM:no obastacle left");
			airsim_chosetraj = airsim_TrajLibrarymain.FindFarthestTrajectory(safe_dis, airsim_pose, octree_map_, airsim_left_pre_traj_no);
		}
		else if (airsim_this_tra == 2)
		{
			//airsim_set_pose();

			if (airsim_TrajLibrarymain.airsim_set_desstraight_traj(des_coordinate_, airsim_pose) != true)
			{
				ROS_ERROR_STREAM("AIRSIM:failed to set destination straight trajectory");
				//std::cout << "airsim failed to set destination straight trajectory" << std::endl;
			}
			ROS_INFO_STREAM("AIRSIM:no obastacle right");
			airsim_chosetraj = airsim_TrajLibrarymain.FindFarthestTrajectory(safe_dis, airsim_pose, octree_map_, airsim_right_pre_traj_no);
		}
		else
		{
			//airsim_set_pose();

			if (airsim_TrajLibrarymain.airsim_set_desstraight_traj(des_coordinate_, airsim_pose) != true)
			{
				ROS_ERROR_STREAM("AIRSIM:failed to set destination straight trajectory");
				//std::cout << "airsim failed to set destination straight trajectory" << std::endl;
			}
			ROS_INFO_STREAM("AIRSIM:no obastacle straight end or straight");
			airsim_chosetraj = airsim_TrajLibrarymain.FindFarthestTrajectory(safe_dis, airsim_pose, octree_map_, airsim_straight_pre_traj_no);

			//return 0;
		}

		airsim_this_tra_last = airsim_this_tra; //上次的路劲编号
		airsim_iter_pose = airsim_pose;
		airsim_this_tra = std::get<0>(airsim_chosetraj);
		airsim_traj_closest_dist = std::get<1>(airsim_chosetraj); //路劲离最近障碍物的距离
		airsim_farthest_traj = std::get<2>(airsim_chosetraj);	 //得到路径对象（位置坐标，姿态角） 将这个信息传给飞控，利用ROS消息的形式
		airsim_way_point.clear();
		airsim_way_point.assign(std::get<3>(airsim_chosetraj).begin(), std::get<3>(airsim_chosetraj).end());

		airsim_t_last = airsim_totaltime;
		airsim_finish = clock();
		airsim_totaltime = (double)(airsim_finish - airsim_start) / CLOCKS_PER_SEC;
		airsim_thisseartime = airsim_totaltime - airsim_t_last;

		ROS_INFO_STREAM("AIRSIM:replan traj:" << airsim_this_tra);
		airsim_mavros_pub_waypoint();
		airsim_if_replan = true;

		// if (octree_map_->getLeafCount() < 1 && airsim_this_tra == 5)
		// {
		// 	// no points in octree
		// 	airsim_way_point.at(8).pose.position.z = takeoff_height;
		// 	ROS_INFO_STREAM("AIRSIM:no points in octoree_map!");
		// }

		airsim_start = clock();
		airsim_way_point_i = 0;
	}
	//run_end_straight_time >=1.5s
	if (airsim_this_tra != 5)
	{
		run_end_straight_timestamp = ros::Time::now().toSec();
		speed = 2.4;
		ROS_INFO_STREAM("flying speed:" << speed);

		if (airsim_pose.uav_position[2] > takeoff_height + 0.5)
		{
			airsim_way_point.at(8).pose.position.z = takeoff_height; // + 0.5;
		}
	}
	else if (ros::Time::now().toSec() - run_end_straight_timestamp >= 4.0)
	{
		run_end_straight_time = ros::Time::now().toSec() - run_end_straight_timestamp;
		airsim_way_point.at(8).pose.position.z = takeoff_height;
		ROS_INFO_STREAM("AIRSIM:run_end_straight_time:" << run_end_straight_time << "s");
		speed = 3.0;
		ROS_INFO_STREAM("flying speed:" << speed);
	}
	if (airsim_pose.uav_position[2] < takeoff_height - 0.5)
	{
		airsim_way_point.at(8).pose.position.z = takeoff_height;
	}
	if (airsim_this_tra == 1 || airsim_this_tra == 2)
	{
		max_wait_seconds = 0.25f;
	}
	else
	{
		max_wait_seconds = 0.25f;
	}
	ROS_INFO_STREAM("AIRSIM:Cycle once done");
	ROS_INFO_STREAM("AIRSIM:airism_delay_time:  " << delay_time << "ms");
}

void Uav_trajplannimg::airsim_setting_flying_yaw()
{
	airsim_plan_fly_yaw_ = atan2(waypoint_enu.y() - airsim_iter_pose.uav_position[1], waypoint_enu.x() - airsim_iter_pose.uav_position[0]);
	yaw_mode_position.yaw_or_rate = airsim_plan_fly_yaw_;
}

void Uav_trajplannimg::execute_waypoints()
{
	client.moveToPosition(waypoint_ned.x(), waypoint_ned.y(), waypoint_ned.z(), speed, max_wait_seconds, driveTrain, yaw_mode_position, lookahead, adaptive_lookahead);
	//ROS_INFO_STREAM("executing waypoints!");
}

bool Uav_trajplannimg::check_arrive_waypoint()
{
	double airsim_x_dist = abs(airsim_mavros_position.pose.position.x - waypoint_enu.x());
	double airsim_y_dist = abs(airsim_mavros_position.pose.position.y - waypoint_enu.y());
	double airsim_z_dist = abs(airsim_mavros_position.pose.position.z - waypoint_enu.z());

	if (airsim_x_dist < 1.0 && airsim_y_dist < 1.0 && airsim_z_dist < 1.0)
	{
		return true;
	}
	else
		return false;
}

bool Uav_trajplannimg::airsim_arrive_destination()
{
	double airsim_x_distance = abs(airsim_mavros_position.pose.position.x - des_coordinate_.pose.position.x);
	double airsim_y_distance = abs(airsim_mavros_position.pose.position.y - des_coordinate_.pose.position.y);
	double airsim_z_distance = abs(airsim_mavros_position.pose.position.z - des_coordinate_.pose.position.z);

	if (airsim_x_distance < 2.0 && airsim_y_distance < 2.0 && airsim_z_distance < 2.0)
	{
		return true;
	}
	else
		return false;
}
void Uav_trajplannimg::airsim_visualization(const ros::TimerEvent &te)
{
	gui_way.poses.push_back(airsim_mavros_position);
	gui_way.header.frame_id = "map";
	true_way_pub.publish(gui_way);
}

void Uav_trajplannimg::airsim_land()
{
	std::cout << "Press Enter to go home" << std::endl;
	std::cin.get();
	client.reset();

	std::cout << "Press Enter to disarm" << std::endl;
	std::cin.get();
	client.armDisarm(false);
}

// void Uav_trajplannimg::airsim_image_capture(std::vector<cv::Mat> &img_lr)
// {
// 	lr_depth.clear();
// 	airsim_img_request = vector<ImageRequest>{ImageRequest(1, ImageType::Scene), ImageRequest(2, ImageType::Scene)};
// 	airsim_img_response = client.simGetImages(airsim_img_request);
// 	ROS_INFO_STREAM( "# of images recieved: " << airsim_img_response.size());
// 	//std::cout << "# of images recieved: " << airsim_img_response.size() << std::endl;

// 	if (airsim_img_response.size() > 0)
// 	{
// 		for (ImageResponse &image_info : airsim_img_response)
// 		{
// 			//cv::Mat depth_img = cv::Mat(image_info.width, image_info.height, CV_32FC1, (float *)image_info.image_data_uint8.airsim_image_capturedata());
// 			cv::Mat depth_img = cv::Mat(image_info.image_data_uint8,true);
// 			lr_depth.push_back(depth_img.clone());

// 		    ROS_INFO_STREAM( "Image uint8 size: " << image_info.width << "*" << image_info.height);
// 			//std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;
// 		}
// 	}

// 	img_lr=lr_depth;
// }

// void Uav_trajplannimg::airsim_image_dowmload()
// {
//     //std::cout << "Press Enter to get FPV image" << std::endl; std::cin.get();
//     vector<ImageRequest> request = {ImageRequest(1,ImageType::Scene,false,true), ImageRequest(2, ImageType::Scene,false,true)};
//     const vector<ImageResponse> &response = client.simGetImages(request);
// 	int img_i=0;
//     std::cout << "# of images recieved: " << response.size() << std::endl;

//     if (response.size() > 0)
//     {
//         //std::cout << "Enter path with ending separator to save images (leave empty for no save)" << std::endl;
//         std::string path("/home/wuqi/airpic");
//         //std::getline(std::cin, path);

//         for (const ImageResponse &image_info : response)
//         {
//             std::cout << "Image uint8 size: " << image_info.width << "*" << image_info.height << std::endl;
//             //std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;

//             if (path != "")
//             {
//                 std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
// 				// if(img_i==0)
// 				// {
// 				// 	file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp)+"left");
// 				// }
// 				// else
// 				// {
// 				// 	file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp)+"right");
// 				// }

//                     std::ofstream file(file_path + ".png", std::ios::binary);
//                     file.write(reinterpret_cast<const char *>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
//                     file.close();
//             }

// 			img_i++;
//         }
//     }
//  }

/*******threads********/
void Uav_trajplannimg::fly_init_thread()
{
	// thread fly_init_thread1(&Uav_trajplannimg::fly_init, this);
	// fly_init_thread1.join();
	thread fly_init_thread2(&Uav_trajplannimg::airsim_init, this);

	fly_init_thread2.join();
}

void Uav_trajplannimg::fly_takeoff_thread(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_)
{
	// thread takeoff_thread1(&Uav_trajplannimg::fly_takeoff, this, octree_map_);
	// takeoff_thread1.join();
	thread takeoff_thread2(&Uav_trajplannimg::airsim_takeoff, this, octree_map_);

	takeoff_thread2.join();
}

void Uav_trajplannimg::fly_traj_plan_thread(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_)
{
	// thread traj_plan_thread1(&Uav_trajplannimg::fly_traj_plan, this, octree_map_);
	// traj_plan_thread1.join();
	thread traj_plan_thread2(&Uav_trajplannimg::airsim_traj_plan, this, octree_map_);
	traj_plan_thread2.join();
	thread execute_waypoints_thread(&Uav_trajplannimg::execute_waypoints, this);
	execute_waypoints_thread.join();
}

// void Uav_trajplannimg::airsim_image_capture_thread(std::vector<cv::Mat> &img_lr)
// {
// 	thread image_capture_thread(&Uav_trajplannimg::airsim_image_capture,this,ref(img_lr));
// 	image_capture_thread.join();

// }