#include "ros_traj_plan/uavtrans.h"

void delay_msec(float msec)
{
	clock_t now = clock();
	while ((float)(clock() - now) * 1000 / CLOCKS_PER_SEC < msec)
		;
}

std::string goal_getTime()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );
    return tmp;
}

//pcl::PointCloud<pcl::PointXYZ>::Ptr building_cloud_;

pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *InsertPointsIntoOctree(pcl::PointCloud<pcl::PointXYZ> cloud_msg)
{
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *current_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(1.0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ = current_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	*current_cloud_ = cloud_msg;
	for (int i = 0; i < cloud_msg.points.size(); i++)
	{
		current_cloud_->push_back(cloud_msg.points[i]);
	}
	current_octree_->setInputCloud(current_cloud_);
	current_octree_->addPointsFromInputCloud();
	// apply this matrix to each point
	// for (int i = 0; i<cloud_msg.points.size (); i++) {

	// 	current_octree_->addPointToCloud(cloud_msg.points[i], current_cloud_);
	// }
	std::cout << "insert map successfully" << std::endl;
	return current_octree_;
}
/**
* Find the distance to the nearest neighbor of a point
*
* @param point the xyz point to search
*
* @retval distance to the nearest neighbor or -1 if no points found.
*/
double NearestNeighbor(float point[3], pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_)
{
	pcl::PointXYZ search_point;
	search_point.x = point[0];
	search_point.y = point[1];
	search_point.z = point[2];

	// init output parameters
	std::vector<int> point_out_indices(1);
	std::vector<float> k_sqr_distances(1);

	// ensure there is at least one point in the tree
	if (octree_map_->getLeafCount() < 1)
	{
		// no points in octree
		return -1;
	}

	int num_points_found = octree_map_->nearestKSearch(search_point, 1, point_out_indices, k_sqr_distances);

	//std::cout << "最近障碍物序号" << point_out_indices .at(0)<< std::endl;
	if (num_points_found < 1)
	{
		// no points found
		return -1;
	}
	else
	{

		return sqrt(k_sqr_distances.at(0));
	}
}

void board_to_world(const uavTrans *utrans, const double src[], float dst[3]) //地面坐标系加入点的坐标
{
	rotation_quat(utrans->uav_quat, src, dst); //坐标系转化 机体坐标转换成地面坐标
											   //std::cout<<"转换坐标（未加自身位置）\n"<<dst[0]<<"\n"<<dst[1]<<"\n"<<dst[2]<<std::endl;
	dst[0] = dst[0] + utrans->uav_position[0];
	dst[1] = dst[1] + utrans->uav_position[1];
	dst[2] = dst[2] + utrans->uav_position[2]; //加上无人机坐标
											   //std::cout<<"转换坐标（加自身位置）\n"<<dst[0]<<"\n"<<dst[1]<<"\n"<<dst[2]<<std::endl;
}

void rotation_quat(const double rot[4], const double v[], float r[3]) //旋转矩阵 坐标转换
{
	double ab = rot[0] * rot[1], ac = rot[0] * rot[2], ad = rot[0] * rot[3];
	double nbb = -rot[1] * rot[1], bc = rot[1] * rot[2], bd = rot[1] * rot[3];
	double ncc = -rot[2] * rot[2], cd = rot[2] * rot[3], ndd = -rot[3] * rot[3];

	r[0] = 2 * ((ncc + ndd) * v[0] + (bc - ad) * v[1] + (ac + bd) * v[2]) + v[0];
	r[1] = 2 * ((ad + bc) * v[0] + (nbb + ndd) * v[1] + (cd - ab) * v[2]) + v[1];
	r[2] = 2 * ((bd - ac) * v[0] + (ab + cd) * v[1] + (nbb + ncc) * v[2]) + v[2];
}

void bot_trans_copy(uavTrans *dest, const uavTrans *src)
{
	memcpy(dest, src, sizeof(uavTrans));
}

void bot_roll_pitch_yaw_to_quat(const double rpy[3], double q[4])
{
	double roll = rpy[0], pitch = rpy[1], yaw = rpy[2];

	double halfroll = roll / 2;
	double halfpitch = pitch / 2;
	double halfyaw = yaw / 2;

	double sin_r2 = sin(halfroll);
	double sin_p2 = sin(halfpitch);
	double sin_y2 = sin(halfyaw);

	double cos_r2 = cos(halfroll);
	double cos_p2 = cos(halfpitch);
	double cos_y2 = cos(halfyaw);

	q[0] = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2; //w
	q[1] = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2; //x
	q[2] = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2; //y
	q[3] = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2; //z
}

void bot_quat_to_roll_pitch_yaw(const double q[4], double rpy[3])
{
	double roll_a = 2 * (q[0] * q[1] + q[2] * q[3]);
	double roll_b = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
	rpy[0] = atan2(roll_a, roll_b);

	double pitch_sin = 2 * (q[0] * q[2] - q[3] * q[1]);
	rpy[1] = asin(pitch_sin);

	double yaw_a = 2 * (q[0] * q[3] + q[1] * q[2]);
	double yaw_b = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
	rpy[2] = atan2(yaw_a, yaw_b);
}

uavTrans set_trans_init()
{
	uavTrans trans_init =
		{
			{
				0,
				0,
				0,
				0,
			},
			{0, 0, 0}};
	return trans_init;
}

geometry_msgs::Pose set_start_point()
{
	geometry_msgs::Pose start_p_;
	start_p_.position.x = 0;
	start_p_.position.y = 0;
	start_p_.position.z = 0;
	start_p_.orientation.x = 0;
	start_p_.orientation.y = 0;
	start_p_.orientation.z = 0;
	start_p_.orientation.w = 0;
	return start_p_;
}
