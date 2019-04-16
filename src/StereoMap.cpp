#include <flight/StereoMap.h>

StereoMap::StereoMap() : map_resolution(0.3),
                         current_tree(map_resolution),
                         building_tree(map_resolution),
                         camera_factor(1)
{
    Octomap_pub = nh.advertise<octomap_msgs::Octomap>("/auto_flight//flight_octomap", 1);

    octomap_timer = nh.createTimer(ros::Duration(0.5), &StereoMap::octomap_visualization, this);

    pclmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/auto_flight/flight_pointcloud_map", 1);

    auto_flight_octomap_state_pub = nh.advertise<auto_flight::auto_flight_octomap_state>("/auto_flight/auto_flight_octomap_state", 1);

    current_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    building_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    current_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
    current_octree_->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)current_cloud_);

    building_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
    building_octree_->setInputCloud(building_cloud_);
}
void StereoMap::RemoveOldPoints(ros::Time ros_time, pcl::PointXYZ &curent_position)
{
    if (flag_firstStart == true)
    {
        current_octree_timestamp_ = ros_time;
        building_octree_timestamp_ = ros_time + ros::Duration(map_exist_time);
        flag_firstStart = false;
    }
    else if (current_octree_timestamp_ > ros_time)
    { //时间戳出错
        delete current_octree_;
        delete building_octree_;

        current_octree_timestamp_ = ros_time;
        building_octree_timestamp_ = ros_time + ros::Duration(map_exist_time);

        current_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        building_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        //重新建图
        current_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
        current_octree_->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)current_cloud_);

        building_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
        building_octree_->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)building_cloud_);

        ROS_INFO_STREAM("\nswapping octrees because jump back in time");
    }
    else if (current_octree_timestamp_ + ros::Duration(map_exist_time * 2) < ros_time)
    {
        //std::cout << "swapping octrees (old: " << (current_octree_timestamp_ - last_msg_time) / 1000000.0f << ", new: " << (building_octree_timestamp_ - last_msg_time) / 1000000.0f << ")" << std::endl;
        //新老地图交替
        // pcl::io::savePCDFile("./result.pcd", *current_cloud_);

        std::vector<int> point_out_indices(1);
        std::vector<float> k_sqr_distances(1);
        ROS_INFO("swaping octomap?");
        if (current_octree_->getLeafCount() < 1)
        {
            // no points in octree
            return;
        }

        int num_points_found = current_octree_->nearestKSearch(curent_position, 1, point_out_indices, k_sqr_distances);
        ROS_INFO_STREAM("octomap:num_points_found:" << num_points_found);
        closest_distance = sqrt(k_sqr_distances.at(0));
        ROS_INFO_STREAM("closest_distance in current_octree:" << closest_distance << "m");

        // if (building_octree_->getLeafCount() >= 1)
        // {
        //     int building_num_points_found = building_octree_->nearestKSearch(curent_position, 1, point_out_indices, k_sqr_distances);

        //     building_closest_distance = sqrt(k_sqr_distances.at(0));
        //     ROS_INFO_STREAM("closest_distance in build_octree:" << building_closest_distance << "m");
        // }

        if (closest_distance >= 4.0) //&& building_closest_distance >= 3.0
        {
            ROS_INFO("swaping octomap!");
            delete current_octree_;
            current_cloud_->clear();
            current_octree_ = building_octree_;
            current_cloud_ = building_cloud_;
            current_octree_timestamp_ = ros_time - ros::Duration(map_exist_time);
            building_octree_timestamp_ = ros_time;
            //构建新地图
            building_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            building_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
            building_octree_->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)building_cloud_);

            //可视化
            // octomap::OcTree current_tree = octomap::OcTree(building_tree);

            // octomap::OcTree building_tree(map_resolution);
        }
        else
        {
            ROS_INFO("waiting to swaping octomap!");
            // // voxel filter
            // pcl::VoxelGrid<PointT> voxel_filter;
            // voxel_filter.setLeafSize(0.01, 0.01, 0.01); // resolution
            // PointCloud::Ptr tmp(new PointCloud);
            // voxel_filter.setInputCloud(current_cloud_);
            // voxel_filter.filter(*tmp);
            // tmp->swap(*current_cloud_);
        }
    }
}

void StereoMap::InsertPointsIntoOctree(vector<Point3f> hitPointsWorld)
{
    if (hitPointsWorld.size() > 3)
    {
        PointCloud::Ptr current(new PointCloud);
        omp_set_num_threads(4);

        for (int i = 0; i < hitPointsWorld.size(); i++)
        {

            PointT p;
            // if (hitPointsWorld[i].z < 2.8 && hitPointsWorld[i].z > 1.7)
            //     continue;
            p.x = hitPointsWorld[i].x;
            p.y = hitPointsWorld[i].y;
            p.z = hitPointsWorld[i].z;
            current->points.push_back(p);
        }

        if (current->points.size() > 0)
        {
            //统计滤波器
            PointCloud::Ptr tmp(new PointCloud);
            pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
            statistical_filter.setMeanK(10);
            statistical_filter.setStddevMulThresh(1.0);
            statistical_filter.setInputCloud(current);
            statistical_filter.filter(*tmp);
            //#pragma omp parallel for
            for (int i = 0; i < tmp->points.size(); i++)
            {
                pcl::PointXYZ obstaclePoints(tmp->points[i].x, tmp->points[i].y, tmp->points[i].z);
                // current_cloud_->push_back(obstaclePoints);
                // building_cloud_->push_back(obstaclePoints);

                current_octree_->addPointToCloud(obstaclePoints, current_cloud_);
                building_octree_->addPointToCloud(obstaclePoints, building_cloud_);

                // current_tree.updateNode(octomap::point3d(obstaclePoints.x / camera_factor, obstaclePoints.y / camera_factor, obstaclePoints.z / camera_factor), true);
                // building_tree.updateNode(octomap::point3d(obstaclePoints.x / camera_factor, obstaclePoints.y / camera_factor, obstaclePoints.z / camera_factor), true);
            }
        }
    }
    else
    {
        ROS_INFO_STREAM("hitPointsWorld is empty");
    }
    ROS_INFO_STREAM("number of obstaclePoints in current cloud:" << current_cloud_->size());
    ROS_INFO_STREAM("number of obstaclePoints in building cloud:" << building_cloud_->size());
    current_octomap_exist_time = ros::Time::now().toSec() - building_octree_timestamp_.toSec();
    ROS_INFO_STREAM("current_octomap_exist_time:" << current_octomap_exist_time);
    auto_flight_octomap_state.header.stamp = ros::Time::now();
    auto_flight_octomap_state.current_octomap_size = current_cloud_->size();
    auto_flight_octomap_state.building_octomap_size = building_cloud_->size();
    auto_flight_octomap_state.current_octomap_exist_time = current_octomap_exist_time;
    auto_flight_octomap_state_pub.publish(auto_flight_octomap_state);
}

void StereoMap::stereoMapStatus()
{
}

void StereoMap::octomap_visualization(const ros::TimerEvent &te)
{
    // current_tree.updateInnerOccupancy();
    // building_tree.updateInnerOccupancy();

    // // prepare octomap msg
    // if (octomap_msgs::binaryMapToMsg(current_tree, msg_octomap))
    // {
    //     msg_octomap.binary = 1;
    //     msg_octomap.id = "OcTree";
    //     msg_octomap.resolution = map_resolution;
    //     msg_octomap.header.frame_id = "/map";
    //     msg_octomap.header.stamp = ros::Time::now();
    //     Octomap_pub.publish(msg_octomap);
    //     ROS_INFO("Octomap updated in RVIZ");
    // }
    // else
    // {
    //     ROS_ERROR("\n failedTransfer to octomap_msgs!");
    // }

    pcl::toROSMsg(*current_cloud_, obstacle_pointcloud_output);
    obstacle_pointcloud_output.header.frame_id = "map";
    pclmap_pub.publish(obstacle_pointcloud_output);
    //ROS_INFO("pclmap updated in RVIZ");
}
