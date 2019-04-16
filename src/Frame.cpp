#include <flight/Frame.h>

Frame::Frame(cv::Mat Q)
{
    // myslam::Config::setParameterFile(settingFile);
    matQ = Q;
    camera_factor = 1000;
    //obstacle_timer = nh.createTimer( ros::Duration(0.5), &Frame::obstacle_visualization, this );
    obstacle_point_publisher = nh.advertise<geometry_msgs::Point32>("/auto_flight/obstacle_points_camera", 1);

    mavros_pose_sub = nh.subscribe("/mavros/local_position/pose", 1, &Frame::get_mavros_pose, this);

    T = Eigen::Isometry3d::Identity();
    //cv::namedWindow("image window");
    flag_simoutanous = false;
    odom_flag = false;
    image_pose_num = 0;
}

sensor_msgs::ImagePtr Frame::imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t)
{
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image &imgMessage = *ptr;
    imgMessage.header.stamp = t;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1; //for endianness detection
    imgMessage.is_bigendian = !(*(char *)&num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);

    if (img.isContinuous())
        memcpy((char *)(&imgMessage.data[0]), img.data, size);
    else
    {
        uchar *opencvData = img.data;
        uchar *rosData = (uchar *)(&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++)
        {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }
    return ptr;
}

void Frame::get_mavros_pose(const geometry_msgs::PoseStamped &msg)
{
    // if (odom_flag == true)
    // {
    current_position.x = msg.pose.position.x;
    current_position.y = msg.pose.position.y;
    current_position.z = msg.pose.position.z;

    T_mavros = Eigen::Isometry3d::Identity();

    Eigen::Quaterniond q = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);

    Eigen::Vector3d translate(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    Eigen::Matrix3d R;
    R = q;
    //cout <<"translate:\n" <<translate <<endl;
    //cout <<"R:\n"<< R <<endl;
    //T.block(0,0,3,3) = R ;
    T_mavros.rotate(q);
    T_mavros.pretranslate(translate);
    //ROS_INFO_STREAM(setprecision(8) << fixed << "\nT = " << T_mavros.matrix());
    flag_simoutanous = true;
    // }

    odom_flag = false;
}

void Frame::getOdom(const nav_msgs::Odometry &msg)
{
    odom_flag = true;

    T = T_mavros;

    image_pose_num ++;
    // current_position.x = msg.pose.pose.position.x;
    // current_position.y = msg.pose.pose.position.y;
    // current_position.z = msg.pose.pose.position.z;
    // // ROS_INFO_STREAM(setprecision(6)<< fixed << "position = ("<<msg.pose.pose.position.y<<","<<msg.pose.pose.position.z<<","<<msg.pose.pose.position.x<<")");
    // // ROS_INFO_STREAM(setprecision(6)<< fixed << "orientation = ("<<msg.pose.pose.orientation.x<<","<<msg.pose.pose.orientation.y<<","<<msg.pose.pose.orientation.z<<","<<msg.pose.pose.orientation.w<<")");

    // ROS_INFO_STREAM(setprecision(6) << fixed << "camera_position = (x:" << msg.pose.pose.position.x << ", y:" << msg.pose.pose.position.y << ", z:" << msg.pose.pose.position.z << ")");
    // ROS_INFO_STREAM(setprecision(6) << fixed << "camera orientation = (x:" << msg.pose.pose.orientation.x << ", y:" << msg.pose.pose.orientation.y << ", z:" << msg.pose.pose.orientation.z << ", w:" << msg.pose.pose.orientation.w << ")");

    // T = Eigen::Isometry3d::Identity();

    // //Eigen::Quaterniond q = Eigen::Quaterniond(-msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, -msg.pose.pose.orientation.x, msg.pose.pose.orientation.w);
    // //aiesim
    // Eigen::Quaterniond q = Eigen::Quaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

    // camera_rpy = mavros::ftf::quaternion_to_rpy(q);
    // float camera_roll = camera_rpy(0);
    // float camera_pitch = camera_rpy(1);
    // float camera_yaw = camera_rpy(2);
    // ROS_INFO_STREAM("camera rpy: camera_roll:" << camera_roll * 180 / 3.1415
    //                                            << "d camera_pitch:" << camera_pitch * 180 / 3.1415
    //                                            << "d camera_yaw:" << camera_yaw * 180 / 3.1415 << "d");

    // // double camera_yaw = tf::getYaw(q);
    // // ROS_INFO_STREAM("camera yaw: " << camera_yaw);

    // Eigen::Vector3d translate(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

    // Eigen::Matrix3d R;
    // R = q;
    // //cout <<"translate:\n" <<translate <<endl;
    // //cout <<"R:\n"<< R <<endl;
    // //T.block(0,0,3,3) = R ;
    // T.rotate(q);
    // T.pretranslate(translate);
    // //ROS_INFO_STREAM(setprecision(8)<< fixed << "T = "<<T.matrix());
    // flag_simoutanous = true;
}

void Frame::visualizaFrame(cv::Mat &displayL_, vector<Point3i> pointVector2d, int blockSize)
{
    cv::Mat displayL;
    cv::cvtColor(displayL_, displayL, CV_GRAY2RGB);
    if (displayL.cols > 9 && displayL.rows > 0)
    {
#pragma omp parallel for
        for (unsigned int i = 0; i < pointVector2d.size(); i++)
        {
            int x2 = pointVector2d[i].x;
            int y2 = pointVector2d[i].y;

            rectangle(displayL, Point(x2, y2), Point(x2 + blockSize, y2 + blockSize), cv::Scalar(255, 0, 0));
            rectangle(displayL, Point(x2 + 1, y2 + 1), Point(x2 + blockSize - 1, y2 - 1 + blockSize), cv::Scalar(255, 0, 0));
        }

        // cv::imshow("image window", displayL);
        //  cv::waitKey(1);
        //imageResultleft.publish(imageToROSmsg(displayL,sensor_msgs::image_encodings::BGR8,a,ros::Time::now()));
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", displayL).toImageMsg(); //mono8
        imageResultleft.publish(msg);
    }
}

void Frame::obstacle_visualization(const ros::TimerEvent &te)
{
}

void Frame::pixelToCamera(std::vector<Point3f> hitPointsPixel)
{
    if (hitPointsPixel.size() > 0)
    {
        perspectiveTransform(hitPointsPixel, hitPointsCamera, matQ);

        // for(int i=0;i<hitPointsPixel.size();i++)
        // {
        //     cout <<" hitPointsPixel point:\n" <<hitPointsPixel[i] << endl;
        //     cout <<" pixelToCamera point:\n" << hitPointsCamera[i] << endl;
        // }
    }
    else
    {
        ROS_INFO_STREAM(" pixelToCamera is Empty");
    }
}
void Frame::cameraToWorld()
{
    if (hitPointsCamera.size() > 0)
    {
        //ROS_INFO_STREAM(setprecision(8)<< fixed << "T.inverse = "<<T.inverse().matrix());
        //#pragma omp parallel for
        for (int i = 0; i < hitPointsCamera.size(); i++)
        {
            Eigen::Vector3d v(-hitPointsCamera[i].z / camera_factor, hitPointsCamera[i].x / camera_factor, hitPointsCamera[i].y / camera_factor);
            //v = T.inverse()*v;

            obstacle_point.x = v(0, 0);
            obstacle_point.y = v(1, 0);
            obstacle_point.z = v(2, 0);
            obstacle_point_publisher.publish(obstacle_point);

            //Point3f temp(-v(0,0),-v(2,0),v(1,0)); //x轴旋转PI?
            //airsim
            Eigen::Vector3d t_v = T * v;
            Point3f temp(t_v(0, 0), t_v(1, 0), t_v(2, 0)); //x轴旋转PI?
            hitPointsWorld.push_back(temp);
        }
    }
    else
    {
        ROS_INFO_STREAM("pointsCamera is empty");
    }
    // return T.inverse() *hitPointsCamera;
}

void Frame::flashFrame()
{
    hitPointsCamera.clear();
    hitPointsWorld.clear();

    vector<Point3f>(hitPointsCamera).swap(hitPointsCamera);
    vector<Point3f>(hitPointsWorld).swap(hitPointsWorld);
    ROS_INFO_STREAM("flashFrame");
}

pcl::PointXYZ &Frame::get_current_position()
{
    return current_position;
}