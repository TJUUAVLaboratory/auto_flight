#include "auto_flight/destinate.h"
#include "ros_traj_plan/searchdistance.h"
#include <fstream>

class mavros_target_position
{
  public:
    mavros_target_position();
    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void local_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void home_position_cb(const mavros_msgs::HomePosition::ConstPtr &msg);
    void gps_target_position_cb(const mavros_msgs::WaypointList::ConstPtr &msg);

    void caculate_local_target();

    void call_service();

    std::string target_getTime()
    {
        time_t timep;
        time(&timep);
        char tmp[64];
        strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
        return tmp;
    }

  private:
    /* data */
    ros::NodeHandle tp_nh;
    ros::NodeHandle private_nh;

    ros::Subscriber gps_sub;   //!< current GPS
    ros::Subscriber local_sub; //!< current local ENU
    ros::Subscriber home_position_sub;

    ros::Subscriber gps_target_position_sub;

    ros::ServiceClient des_client;
    auto_flight::destinate des_srv;

    /* Stores current gps state. */
    //sensor_msgs::NavSatFix current_gps_msg;
    Eigen::Vector3d current_gps;       //!< geodetic coordinates LLA
    Eigen::Vector3d current_local_pos; //!< Current local position in ENU

    mavros_msgs::WaypointList::ConstPtr target_position_list;

    Eigen::Vector3d gps_target;
    Eigen::Vector3d local_target;

    bool target_flag, current_local_flag, current_gps_flag;

    std::string target_position_txt;
};

mavros_target_position::mavros_target_position() : private_nh("~"),
                                                   target_flag(false),
                                                   current_local_flag(false),
                                                   current_gps_flag(false)
{
    private_nh.param("destination_z", des_srv.request.z, 5.0);
    private_nh.param("target_position_txt", target_position_txt, std::string("target_position.txt"));
    des_srv.response.des_r = false;

    des_client = tp_nh.serviceClient<auto_flight::destinate>("set_destination");

    // subscriber for current gps state, mavros/global_position/global.
    gps_sub = tp_nh.subscribe("/mavros/global_position/global", 1, &mavros_target_position::gps_cb, this);
    // Subscribe for current local ENU pose.
    local_sub = tp_nh.subscribe("/mavros/local_position/pose", 1, &mavros_target_position::local_cb, this);

    gps_target_position_sub = tp_nh.subscribe("/mavros/mission/waypoints", 1, &mavros_target_position::gps_target_position_cb, this);

    // Subscribe for home_position.
    home_position_sub = tp_nh.subscribe("/mavros/home_position/home", 1, &mavros_target_position::home_position_cb, this);

    current_gps = Eigen::Vector3d::Zero();
    current_local_pos = Eigen::Vector3d::Zero();
    gps_target = Eigen::Vector3d::Zero();
    local_target = Eigen::Vector3d::Zero();
}

/**
	 * Current GPS coordinates
	 */
void mavros_target_position::gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    current_gps = Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude);
    current_gps_flag = true;
}

/**
	 * current local position in ENU
	 */
void mavros_target_position::local_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_local_pos = mavros::ftf::to_eigen(msg->pose.position);
    current_local_flag = true;
}

/**
	 *gps_target_position
	 */
void mavros_target_position::gps_target_position_cb(const mavros_msgs::WaypointList::ConstPtr &msg)
{
    target_position_list = msg;

    ROS_INFO_STREAM("waypoint size:" << target_position_list->waypoints.size());

    if (current_gps_flag == true && current_local_flag == true && target_position_list->waypoints.size() > 0) //
    {
        gps_target = Eigen::Vector3d(target_position_list->waypoints.at(0).x_lat,
                                     target_position_list->waypoints.at(0).y_long,
                                     target_position_list->waypoints.at(0).z_alt);

        caculate_local_target();

        std::cout << "current_gps：\n"
                  << current_gps.x() << std::endl;
        std::cout << current_gps.y() << std::endl;
        std::cout << current_gps.z() << std::endl;

        std::cout << "current_local：\n"
                  << current_local_pos.x() << std::endl;
        std::cout << current_local_pos.y() << std::endl;
        std::cout << current_local_pos.z() << std::endl;

        std::cout << "终点gps：\n"
                  << target_position_list->waypoints.at(0).x_lat << std::endl;
        std::cout << target_position_list->waypoints.at(0).y_long << std::endl;
        std::cout << target_position_list->waypoints.at(0).z_alt << std::endl;

        std::cout << "终点local：\n"
                  << des_srv.request.x << std::endl;
        std::cout << des_srv.request.y << std::endl;
        std::cout << des_srv.request.z << std::endl;

        target_flag = true;
    }
}

/**
	 *home_position
	 */
void mavros_target_position::home_position_cb(const mavros_msgs::HomePosition::ConstPtr &msg)
{
    // current_local_pos = mavros::ftf::to_eigen(msg->position);
    // current_local_flag = true;
    // current_gps = Eigen::Vector3d(msg->geo.latitude, msg->geo.longitude, msg->geo.altitude);
    // current_gps_flag = true;
}

void mavros_target_position::caculate_local_target()
{
    /**
		 * The idea is to convert the change in LLA(goal_gps-current_gps) to change in ENU
		 * 1- convert current/goal gps points to current/goal ECEF points
		 * 2- claculate offset in ECEF frame
		 * 3- converts ECEF offset to ENU offset given current gps LLA
		 * 4- adds ENU offset to current local ENU to that will be sent to FCU
		 */

    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

    // current gps -> curent ECEF
    Eigen::Vector3d current_ecef;
    earth.Forward(current_gps.x(), current_gps.y(), current_gps.z(),
                  current_ecef.x(), current_ecef.y(), current_ecef.z());

    // goal gps -> goal ECEF
    Eigen::Vector3d goal_ecef;
    earth.Forward(gps_target.x(), gps_target.y(), gps_target.z(),
                  goal_ecef.x(), goal_ecef.y(), goal_ecef.z());

    // get ENU offset from ECEF offset
    Eigen::Vector3d ecef_offset = goal_ecef - current_ecef;
    Eigen::Vector3d enu_offset = mavros::ftf::transform_frame_ecef_enu(ecef_offset, current_gps);

    local_target = current_local_pos + enu_offset;

    des_srv.request.x = local_target.x();
    des_srv.request.y = local_target.y();
}

void mavros_target_position::call_service()
{

    std::ofstream fout;
    while (ros::ok())
    {
        if (target_flag == true)
        {

            if (abs(des_srv.request.x - current_local_pos.x()) > 150 || abs(des_srv.request.y - current_local_pos.y()) > 150)
            {
                // fout.open(target_position_txt, std::ios::app); //创建一个txt的文件
                // std::string system_time = target_getTime();
                // fout << system_time << "  current position x:" << current_local_pos.x() << "  y:" << current_local_pos.y() << "  z:" << current_local_pos.z() << std::endl; //将变量的值写入文件
                // fout << system_time << "  target  position x:" << des_srv.request.x << "  y:" << des_srv.request.y << "  z:" << des_srv.request.z << std::endl;
                // fout << system_time << "  x or y out of range:" << std::endl
                //      << std::endl; //将变量的值写入文件
                // fout.close();      //关闭文件
                //continue;
            }
            else if (des_client.call(des_srv) && des_srv.response.des_r == 1)
            {
                ROS_INFO("setting target position");

                std::cout << "终点：\n"
                          << des_srv.request.x << std::endl;
                std::cout << des_srv.request.y << std::endl;
                std::cout << des_srv.request.z << std::endl;

                fout.open(target_position_txt, std::ios::app); //创建一个txt的文件
                std::string system_time = target_getTime();
                fout << system_time << "  current position x:" << current_local_pos.x() << "  y:" << current_local_pos.y() << "  z:" << current_local_pos.z() << std::endl; //将变量的值写入文件
                fout << system_time << "  target  position x:" << des_srv.request.x << "  y:" << des_srv.request.y << "  z:" << des_srv.request.z << std::endl
                     << std::endl; //将变量的值写入文件
                fout.close();      //关闭文件

                break;
            }
        }
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_target_position");

    mavros_target_position target_position;

    ROS_INFO_STREAM("setting target position");

    target_position.call_service();

    return 0;
}