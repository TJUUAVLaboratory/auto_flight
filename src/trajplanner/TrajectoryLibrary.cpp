/*
 * Trajectory library
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */

#include "ros_traj_plan/TrajectoryLibrary.hpp"

// Constructor that loads a trajectorys from a directory
TrajectoryLibrary::TrajectoryLibrary(double ground_safety_distance, int this_traj)
{
	ground_safety_distance_ = ground_safety_distance;
	this_traj_ = this_traj;
}

bool TrajectoryLibrary::LoadLibrary(std::string dirname, bool quiet)
{
	// if dirname does not end in "/", add a "/"

	if (dirname.back() != '/') //轨迹库磁盘路径
	{
		dirname.append("/");
	}

	// open the directory and find all the files that end in .csv
	DIR *dirp = NULL;
	dirp = opendir(dirname.c_str());
	struct dirent *dp;

	/*

		while (dp = readdir(dirp))  
{  
    std::cout<<dp->d_name<<std::endl; 
    //printf("%s\\n", entry->d_name); 
}  
   return false;
   */

	if (!dirp)
	{
		std::cerr << "ERROR: no such directory: " << dirname << std::endl;
		return false;
	}

	std::vector<Trajectory> temp_traj;

	int count = 0;

	while (dp = readdir(dirp))
	{
		std::string this_file = dp->d_name;

		if (this_file.length() > 4 && this_file.compare(this_file.length() - 6, 6, "-x.csv") == 0)
		{ //有时间看看这个函数是怎么定义的
			// found a .csv file
			// load a trajectory
			Trajectory this_traj(dirname + this_file.substr(0, this_file.length() - 6), quiet);

			temp_traj.push_back(this_traj);

			count++;
		}
	}

	// now we have loaded everything into memory, so sort
	for (int i = 0; i < (int)temp_traj.size(); i++)
	{

		bool flag = false;
		for (auto traj : temp_traj)
		{
			if (traj.GetTrajectoryNumber() == i)
			{
				traj_vec_.push_back(traj); //将Trajectory类型轨迹load到traj_vec_数组中  traj_vec_数组在哪定义的？头文件中的私有变量
				flag = true;			   //通过这个变换，保证轨迹的编号顺序，确定轨迹的优先级。
				break;
			}
		}
		if (flag == false)
		{
			std::cerr << "ERROR: missing trajectory #" << i << std::endl;

			return false;
		}
	}

	//(void)closedir(dirp);

	if (!quiet)
	{
		std::cout << "Loaded " << traj_vec_.size() << " trajectorie(s)" << std::endl;
	}

	if (traj_vec_.size() > 0)
	{
		return true;
	}
	return false;
}

bool TrajectoryLibrary::set_desstraight_traj(geometry_msgs::PoseStamped des_, const uavTrans &start_p)
{
	//Trajectory this_traj;
	int num_of_points = 9;
	Eigen::MatrixXd ep_matrix;
	double s_time = 0.0;
	uavTrans trans_xyz_yaw;
	bot_trans_copy(&trans_xyz_yaw, &start_p);
	float board_xyz[3];
	double world_xyz[3] =
		{des_.pose.position.x - trans_xyz_yaw.uav_position[0],
		 des_.pose.position.y - trans_xyz_yaw.uav_position[1],
		 des_.pose.position.z - trans_xyz_yaw.uav_position[2]};
	double rpy[3];
	bot_quat_to_roll_pitch_yaw(trans_xyz_yaw.uav_quat, rpy);
	rpy[0] = 0.0;
	rpy[1] = 0.0;
	rpy[2] = -rpy[2];
	bot_roll_pitch_yaw_to_quat(rpy, trans_xyz_yaw.uav_quat);
	rotation_quat(trans_xyz_yaw.uav_quat, world_xyz, board_xyz); //坐标系转化 世界坐标转换成机体坐标
	double ep_x = board_xyz[0];
	double ep_y = board_xyz[1];
	double ep_z = board_xyz[2];
	double ep_x2 = pow(board_xyz[0], 2.0);
	double ep_y2 = pow(board_xyz[1], 2.0);
	double ep_z2 = pow(board_xyz[2], 2.0);

	double ep = sqrt(ep_x2 + ep_y2 + ep_z2);
	double ep_vec[3];
	ep_vec[0] = ep_x / ep;
	ep_vec[1] = ep_y / ep;
	ep_vec[2] = ep_z / ep;
	ep_matrix.resize(num_of_points, 4);

	for (int p_i = 0; p_i < num_of_points; p_i++)
	{
		ep_matrix(p_i, 0) = s_time;
		s_time += 0.01;
		ep_matrix(p_i, 1) = ep_vec[0] * s_time * 50;
		ep_matrix(p_i, 2) = ep_vec[1] * s_time * 50;
		ep_matrix(p_i, 3) = ep_vec[2] * s_time * 50 ;
		// if (p_i == 6)
		// {
		// 	ep_matrix(p_i, 3) = ep_vec[2] * s_time * 50 + 0.06;
		// }

		//ep_matrix(p_i,4)=0.0;
		//ep_matrix(p_i,5)=0.0;
		//ep_matrix(p_i,6)=0.0;
	}

	Trajectory straight_traj(4, ep_matrix, 3, 0.01, 1.0);
	traj_vec_.at(4) = straight_traj;
	return true;
}

bool TrajectoryLibrary::airsim_set_desstraight_traj(geometry_msgs::PoseStamped des_, const uavTrans &start_p)
{
	//Trajectory this_traj;
	int num_of_points = 11;
	Eigen::MatrixXd ep_matrix;
	double s_time = 0.0;
	uavTrans trans_xyz_yaw;
	bot_trans_copy(&trans_xyz_yaw, &start_p);
	float board_xyz[3];
	double world_xyz[3] =
		{des_.pose.position.x - trans_xyz_yaw.uav_position[0],
		 des_.pose.position.y - trans_xyz_yaw.uav_position[1],
		 des_.pose.position.z - trans_xyz_yaw.uav_position[2]};
	double rpy[3];
	bot_quat_to_roll_pitch_yaw(trans_xyz_yaw.uav_quat, rpy);
	rpy[0] = 0.0;
	rpy[1] = 0.0;
	rpy[2] = -rpy[2];
	bot_roll_pitch_yaw_to_quat(rpy, trans_xyz_yaw.uav_quat);
	rotation_quat(trans_xyz_yaw.uav_quat, world_xyz, board_xyz); //坐标系转化 世界坐标转换成机体坐标
	double ep_x = board_xyz[0];
	double ep_y = board_xyz[1];
	double ep_z = board_xyz[2];
	double ep_x2 = pow(board_xyz[0], 2.0);
	double ep_y2 = pow(board_xyz[1], 2.0);
	double ep_z2 = pow(board_xyz[2], 2.0);

	double ep = sqrt(ep_x2 + ep_y2 + ep_z2);
	double ep_vec[3];
	ep_vec[0] = ep_x / ep;
	ep_vec[1] = ep_y / ep;
	ep_vec[2] = ep_z / ep;
	ep_matrix.resize(num_of_points, 4);

	for (int p_i = 0; p_i < num_of_points; p_i++)
	{
		ep_matrix(p_i, 0) = s_time;
		s_time += 0.01;
		ep_matrix(p_i, 1) = ep_vec[0] * s_time * 50;
		ep_matrix(p_i, 2) = ep_vec[1] * s_time * 50;
		ep_matrix(p_i, 3) = ep_vec[2] * s_time * 50;
		// if (p_i == 6)
		// {
		// 	ep_matrix(p_i, 3) = ep_vec[2] * s_time * 50 + 0.06;
		// }
	}

	Trajectory straight_traj(5, ep_matrix, 3, 0.01, 1.0);
	traj_vec_.at(5) = straight_traj;
	return true;
}

void TrajectoryLibrary::Print() const
{

	std::cout << "Time-varying trajectories" << std::endl
			  << "------------------------" << std::endl;

	for (int i = 0; i < GetNumberTrajectories(); i++)
	{
		traj_vec_.at(i).Print();
		std::cout << "num of points=" << traj_vec_.at(i).GetNumberOfPoints() << std::endl;
	}
}

Trajectory *TrajectoryLibrary::GetTrajectoryByNumber(int number)
{

	if (number >= GetNumberTrajectories())
	{
		std::cerr << "WARNING: Requested trajectory #" << number << " which is not in library (library size = " << GetNumberTrajectories() << ")." << std::endl;
		return nullptr;
	}
	else
	{
		return &traj_vec_.at(number);
	}
}

/**
* Finds the first Trajectory that is at least "threshold" distance away from any obstacle and the ground.
* In the case  that there is no such trajectory, returns the trajectory that is furthest from obstacles and
* the ground.
*
* @param octomap obstacle map
* @param body_to_local tranform for the aircraft that describes where we are in the map
* @param threshold minimum safe distance for the aircraft
* @param trajectory_out pointer that will be set to the best trajectory
* @param (optional) lcmgl if not NULL, will draw debug data

* const StereoOctomap &octomap, const BotTrans &body_to_local, double threshold, bot_lcmgl_t* lcmgl,

* @retval the distance to the closest obstacle or -1 if there are no obstacles
*/
std::tuple<int, double, Trajectory *, std::vector<geometry_msgs::PoseStamped>> TrajectoryLibrary::FindFarthestTrajectory(double threshold, const uavTrans body_to_local, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_, int preferred_traj[])
{

	Trajectory *farthest_traj = nullptr;
	bool if_sattisfy;
	std::vector<geometry_msgs::PoseStamped> waypoint_temp;
	std::vector<geometry_msgs::PoseStamped> waypoint;
	geometry_msgs::PoseStamped waypoint_;

	double traj_closest_dist = -1;
	int this_traj;

	// for each point in each trajectory, find the point that is closest in the octree 有优先级的设置
	for (int i = 0; i < 4; i++)
	{

		this_traj = i;

		waypoint_temp.clear();
		waypoint_temp.resize(400);
		//waypoint_temp.erase(waypoint_temp.begin(),waypoint_temp.end());

		if (preferred_traj[i] >= GetNumberTrajectories())
		{
			std::cerr << "WARNING: preferred trajectory number exceeds library size, ignoring it." << std::endl;
			preferred_traj[i] = -1;
		}

		if (preferred_traj[i] >= 0)
		{
			// there is a preferred trajectory active

			this_traj = preferred_traj[i];
		}
		//this_traj_ = this_traj;
		//std::cout << "Searching trajectory: " << this_traj << std::endl;

		double closest_obstacle_distance = -1;

		int number_of_points = traj_vec_.at(this_traj).GetNumberOfPoints();
		std::vector<double> point_distances(number_of_points);

		// check minumum altitude
		double min_altitude = traj_vec_.at(this_traj).GetMinimumAltitude() + body_to_local.uav_position[2];
		if (min_altitude < ground_safety_distance_)
		{
			// this trajectory would impact the ground
			closest_obstacle_distance = 0;
			//std::cout << "Trajectory " << this_traj << " would violate ground safety." << std::endl;
		}
		else
		{

		// for each trajectory, look at each point

		// use all availble processors
#pragma omp parallel
			{
#pragma omp for
				for (int j = 0; j < number_of_points; j++)
				{
					// now we are looking at a single point in a trajectorybot_lcmgl_t *lcmgl
					//ROS_INFO_STREAM("I am Thread " << omp_get_thread_num());

					float transformedPoint[3];

					double this_t = traj_vec_.at(this_traj).GetTimeAtIndex(j);
					/*
				Eigen::VectorXd state = traj_vec_.at(this_traj).GetState(this_t);
				transformedPoint[0] = state(0);
				transformedPoint[1] = state(1);
				transformedPoint[2] = state(2);
				*/
					traj_vec_.at(this_traj).GetXyzYawTransformedPoint(this_t, body_to_local, transformedPoint);
					//std::cout << "searching at (" << transformedPoint[0] << ", " << transformedPoint[1] << ", " << transformedPoint[2] << ")...";

					waypoint_.pose.position.x = transformedPoint[0];
					waypoint_.pose.position.y = transformedPoint[1];
					waypoint_.pose.position.z = transformedPoint[2];
					waypoint_temp.at(j) = waypoint_;
					//std::cout<<"发布航点:   "<<waypoint_temp.at(j)<<std::endl;
					//waypoint.push_back(waypoint_);
					point_distances.at(j) = NearestNeighbor(transformedPoint, octree_map_);

					//std::cout << "distance is: " << distance_to_point << std::endl;
				}

#pragma omp for
				for (int j = 0; j < number_of_points; j++)
				{
					double distance_to_point = point_distances.at(j);
					if (distance_to_point >= 0)
					{
						if (distance_to_point < closest_obstacle_distance || closest_obstacle_distance < 0)
						{
							closest_obstacle_distance = distance_to_point;
						}
					}
				}
			} //for循环中的语句并行执行
		}

		/*
	 for (int j = 0; j < number_of_points; j++) 
		    {   
		        waypoint.push_back(waypoint_temp.at(j));
				//std::cout<<"发布航点:"<< j<<"    "<<waypoint.at(j)<<std::endl;
		    }
	*/

		std::cout << "Trajectory " << this_traj << " has distance = " << closest_obstacle_distance << std::endl;

		if (traj_closest_dist == -1 || closest_obstacle_distance > traj_closest_dist)
		{

			this_traj_ = this_traj;
			traj_closest_dist = closest_obstacle_distance;
			farthest_traj = &traj_vec_.at(this_traj);

			waypoint.clear();
			for (int j = 0; j < number_of_points; j++)
			{
				waypoint.push_back(waypoint_temp.at(j));
				//std::cout<<"发布航点:"<< j<<"    "<<waypoint.at(j)<<std::endl;
			} //给路劲指针赋予轨迹地址  会随着for循环不断赋值

			if (traj_closest_dist > threshold || traj_closest_dist < 0)
			{ //traj_closest_dist < 0保证程序的封闭性
				// we are satisfied with this one, run it!

				std::cout << "Trajectory " << this_traj_ << " is good enough, running!" << std::endl;

				return std::tuple<int, double, Trajectory *, std::vector<geometry_msgs::PoseStamped>>(this_traj_, traj_closest_dist, farthest_traj, waypoint); //返回最短距离中最大的距离以及路径的地址,路劲的编号
																																							   //这里可以返回值，满足条件，提前结束循环
			}
		}
	}
	// if (this_traj_ == 5 || this_traj_ == 4) //|| this_traj_ ==  0
	// {
	// 	for (int j = 0; j < waypoint.size(); j++)
	// 	{
	// 		// waypoint.at(j).pose.position.x = body_to_local.uav_position[0];// 1.0;
	// 		// waypoint.at(j).pose.position.y = body_to_local.uav_position[1];
	// 		// waypoint.at(j).pose.position.z = body_to_local.uav_position[2] + 1.5;
	// 		waypoint.at(j).pose.position.z += 2.0;
	// 		//都不服个条件，向上飞1米！
	// 	}
	// }

	std::cout << "No trajectory is satisfied!!" << std::endl;

	return std::tuple<int, double, Trajectory *, std::vector<geometry_msgs::PoseStamped>>(this_traj_, traj_closest_dist, farthest_traj, waypoint);
}
