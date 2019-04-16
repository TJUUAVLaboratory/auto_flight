#include "flight/ImgPro.h"
#include <omp.h>
//
//read Parameter from parameterfile
//
imgPro::imgPro(string strSettingPath)
{
    myslam::Config::setParameterFile(strSettingPath);

    Disparity_sub = nh.subscribe("/zed/Disparity", 1, &imgPro::call_Disparitymsg, this);
    piex_point_pub = nh.advertise<geometry_msgs::Point32>("/auto_flight/obstacle_points_piex", 1);

    image_num = 0;

    blockSize = myslam::Config::get<int>("blockSize");
    disparity = myslam::Config::get<int>("disparity");
    INVARIANCE_CHECK_HORZ_OFFSET_MAX = myslam::Config::get<int>("INVARIANCE_CHECK_HORZ_OFFSET_MAX");
    INVARIANCE_CHECK_HORZ_OFFSET_MIN = myslam::Config::get<int>("INVARIANCE_CHECK_HORZ_OFFSET_MIN");
    INVARIANCE_CHECK_VERT_OFFSET_INCREMENT = myslam::Config::get<int>("INVARIANCE_CHECK_VERT_OFFSET_INCREMENT");
    INVARIANCE_CHECK_VERT_OFFSET_MAX = myslam::Config::get<int>("INVARIANCE_CHECK_VERT_OFFSET_MAX");
    INVARIANCE_CHECK_VERT_OFFSET_MIN = myslam::Config::get<int>("INVARIANCE_CHECK_VERT_OFFSET_MIN");
    zero_disparity = myslam::Config::get<int>("zero_disparity");
    sobelLimit = myslam::Config::get<int>("sobelLimit");
    sadThreshold = myslam::Config::get<int>("sadThreshold");
    check_horizontal_invariance = true;

    double matQ_03, matQ_13, matQ_23, matQ_32;
    matQ_03 = myslam::Config::get<double>("matQ[0][3]");
    matQ_13 = myslam::Config::get<double>("matQ[1][3]");
    matQ_23 = myslam::Config::get<double>("matQ[2][3]");
    matQ_32 = myslam::Config::get<double>("matQ[3][2]");

    matQ = (Mat_<double>(4, 4) << 1, 0, 0, matQ_03, 0, 1, 0, matQ_13, 0, 0, 0, matQ_23, 0, 0, matQ_32, 0);

    cout << "matQ : " << matQ << endl;
    cout << "blockSize : " << blockSize << endl;
    cout << "disparity : " << disparity << endl;
    cout << "INVARIANCE_CHECK_HORZ_OFFSET_MAX : " << INVARIANCE_CHECK_HORZ_OFFSET_MAX << endl;
    cout << "INVARIANCE_CHECK_HORZ_OFFSET_MIN : " << INVARIANCE_CHECK_HORZ_OFFSET_MIN << endl;
    cout << "INVARIANCE_CHECK_VERT_OFFSET_INCREMENT : " << INVARIANCE_CHECK_VERT_OFFSET_INCREMENT << endl;
    cout << "INVARIANCE_CHECK_VERT_OFFSET_MAX : " << INVARIANCE_CHECK_VERT_OFFSET_MAX << endl;
    cout << "INVARIANCE_CHECK_VERT_OFFSET_MIN : " << INVARIANCE_CHECK_VERT_OFFSET_MIN << endl;
    cout << "zero_disparity : " << zero_disparity << endl;
    cout << "sobelLimit　: " << sobelLimit << endl;
    cout << "sadThreshold　: " << sadThreshold << endl;

    laplacianLeft = cv::Mat(image_height, image_width, CV_8UC1, Scalar(0));
    laplacianRight = cv::Mat(image_height, image_width, CV_8UC1, Scalar(0));
    imgLeft = cv::Mat(image_height, image_width, CV_8UC1, Scalar(0));
    imgRight = cv::Mat(image_height, image_width, CV_8UC1, Scalar(0));
    imgdepth = cv::Mat(image_height, image_width, CV_32FC1, Scalar(0));

    //检测视察的设置的上下误差范围(-2.0m~1.0m)
    up_error = (matQ.at<double>(2, 3) / (-1.0 * matQ.at<double>(3, 2))) / (matQ.at<double>(2, 3) / (-1.0 * matQ.at<double>(3, 2)) / (-1 * disparity) - 3000) + disparity;
    down_error = (matQ.at<double>(2, 3) / (-1.0 * matQ.at<double>(3, 2))) / (matQ.at<double>(2, 3) / (-1.0 * matQ.at<double>(3, 2)) / (-1 * disparity) + 1000) + disparity;
    std::cout << " up_error:" << up_error << std::endl;
    std::cout << " down_error:" << down_error << std::endl;
    depth2disp_ratio = matQ.at<double>(2, 3) / (-1.0 * matQ.at<double>(3, 2) * 1000);
    std::cout << " depth2disp_ratio:" << depth2disp_ratio << std::endl;

    cudaMallocManaged(&src_img1, image_height * image_width * sizeof(unsigned char));
    cudaMallocManaged(&src_img2, image_height * image_width * sizeof(unsigned char));
    cudaMallocManaged(&lapla_img1, image_height * image_width * sizeof(unsigned char));
    cudaMallocManaged(&lapla_img2, image_height * image_width * sizeof(unsigned char));

    cudaMallocManaged(&pixel_points, image_height * image_width * sizeof(int));
    cudaMallocManaged(&camera_points, image_height * image_width * sizeof(float));

    cudaMemset(pixel_points, 0, image_height * image_width * sizeof(int));
    cudaMemset(camera_points, 0, image_height * image_width * sizeof(float));

    // cudaError cuda_err = cudaMallocManaged(&dev_ptr, sizeof(*this));
    // if (cuda_err != cudaSuccess)
    //     throw CudaException("DeviceImage: cannot copy image parameters to device memory.", cuda_err);

    // cuda_err = cudaMemcpy(
    //     dev_ptr,
    //     this,
    //     sizeof(*this),
    //     cudaMemcpyHostToDevice);
    // if (cuda_err != cudaSuccess)
    //     throw CudaException("DeviceImage: cannot copy image parameters to device memory.", cuda_err);

    tmp_imgpro = new image_process(image_width, image_height);
    cudaMallocManaged(&dev_imgpro, sizeof(image_process));
    cudaMemcpy(dev_imgpro, tmp_imgpro, sizeof(image_process), cudaMemcpyHostToDevice);

    dev_imgpro->imgWidth = image_width;
    dev_imgpro->imgHeight = image_height;
    dev_imgpro->INVARIANCE_CHECK_HORZ_OFFSET_MIN = INVARIANCE_CHECK_HORZ_OFFSET_MIN;
    dev_imgpro->INVARIANCE_CHECK_HORZ_OFFSET_MAX = INVARIANCE_CHECK_HORZ_OFFSET_MAX;
    dev_imgpro->INVARIANCE_CHECK_VERT_OFFSET_INCREMENT = INVARIANCE_CHECK_VERT_OFFSET_INCREMENT;
    dev_imgpro->INVARIANCE_CHECK_VERT_OFFSET_MIN = INVARIANCE_CHECK_VERT_OFFSET_MIN;
    dev_imgpro->INVARIANCE_CHECK_VERT_OFFSET_MAX = INVARIANCE_CHECK_VERT_OFFSET_MAX;
    dev_imgpro->horizontalInvarianceMultiplier = horizontalInvarianceMultiplier;
    dev_imgpro->blockSize = blockSize;
    dev_imgpro->sadThreshold = sadThreshold;
    //dev_imgpro->sad = sad;
    dev_imgpro->zero_disparity = zero_disparity;
    dev_imgpro->sobelLimit = sobelLimit;
    dev_imgpro->disparity = disparity;
    dev_imgpro->check_horizontal_invariance = check_horizontal_invariance;
    dev_imgpro->up_error = up_error;
    dev_imgpro->down_error = down_error;
    dev_imgpro->depth2disp_ratio = matQ.at<double>(2, 3) / (-1.0 * matQ.at<double>(3, 2) * 1000);
    dev_imgpro->point_num = -1;
    dev_imgpro->sum_of_point_num = 0;
    dev_imgpro->sum_of_point = 0;

    //dev_imgpro->param_memocpy();
}

void imgPro::HitPoints_gpu()
{
}

void imgPro::get_camera_points_cuda(vector<Point3f> &localHitPoints)
{
    if (dev_imgpro->point_num >= 0)
    {
        for (int i = 0; i < dev_imgpro->point_num + 1; i++)
        {
            if (dev_imgpro->camera_points[i * 3 + 2] == 0.0 || dev_imgpro->camera_points[i * 3] == 0.0 || dev_imgpro->camera_points[i * 3 + 1] == 0.0)

                break;

            localHitPoints.push_back(Point3f(dev_imgpro->camera_points[i * 3], dev_imgpro->camera_points[i * 3 + 1],
                                             dev_imgpro->camera_points[i * 3 + 2]));
        }
    }
}

void imgPro::get_pixel_points_cuda(vector<Point3i> &pointVector2d)
{
    if (dev_imgpro->point_num >= 0)
    {
        for (int i = 0; i < dev_imgpro->point_num + 1; i++)
        {
            if (dev_imgpro->pixel_points[i * 3 + 2] == 0 || dev_imgpro->pixel_points[i * 3] == 0 || dev_imgpro->pixel_points[i * 3 + 1] == 0)

                break;

            pointVector2d.push_back(Point3i(dev_imgpro->pixel_points[i * 3], dev_imgpro->pixel_points[i * 3 + 1],
                                            dev_imgpro->pixel_points[i * 3 + 2]));
        }
    }
}

void imgPro::HitPoints(vector<Point3f> &localHitPoints, vector<Point3i> &pointVector2d)
{
    ROS_INFO_STREAM("image_num:" << image_num);
    //HitPoints_gpu();

    // dev_imgpro->set_raw_image_data(imgLeft, imgRight);

    // Disparity_array_current = Disparity_array;
    // dev_imgpro->set_depth_image_data(Disparity_array_current);

    //gpu

    image_process_gpu(dev_imgpro);

    ROS_INFO_STREAM("feature points num(cuda):" << dev_imgpro->point_num);
    if (dev_imgpro->point_num >= 0)
    {
        // cudaMemcpy(camera_points, dev_imgpro->camera_points, (dev_imgpro->point_num + 1) * 3  * sizeof(int),cudaMemcpyDeviceToHost);
        // cudaMemcpy(pixel_points, dev_imgpro->pixel_points, (dev_imgpro->point_num + 1) * 3  * sizeof(float),cudaMemcpyDeviceToHost);
        int true_num = -1;
        ROS_INFO_STREAM("insert points");
        for (int i = 0; i < dev_imgpro->point_num + 1; i++)
        {
            // cout << "i:" << i << "  x:" << dev_imgpro->camera_points[i * 3] << " y:"
            //      << dev_imgpro->camera_points[i * 3 + 1] << " z:" << dev_imgpro->camera_points[i * 3 + 2] << endl;
            if (dev_imgpro->pixel_points[i * 3 + 2] == 0)
                continue;

            if (dev_imgpro->camera_points[i * 3 + 2] == 0.0)
                continue;
            ;
            piex_obstacle_point.x = dev_imgpro->pixel_points[i * 3];
            piex_obstacle_point.y = dev_imgpro->pixel_points[i * 3 + 1];
            piex_obstacle_point.z = dev_imgpro->pixel_points[i * 3 + 2];
            piex_point_pub.publish(piex_obstacle_point);

            pointVector2d.push_back(Point3i(dev_imgpro->pixel_points[i * 3], dev_imgpro->pixel_points[i * 3 + 1],
                                            dev_imgpro->pixel_points[i * 3 + 2]));

            localHitPoints.push_back(Point3f(dev_imgpro->camera_points[i * 3], dev_imgpro->camera_points[i * 3 + 1],
                                             dev_imgpro->camera_points[i * 3 + 2]));
            true_num++;
        }

        ROS_INFO_STREAM("feature points num(cuda) true:" << true_num);

        dev_imgpro->sum_of_point_num = dev_imgpro->sum_of_point_num + true_num + 1;
        dev_imgpro->sum_of_point = dev_imgpro->sum_of_point + dev_imgpro->point_num + 1;

        //  memset(pixel_points, 0, (dev_imgpro->point_num + 1) * 3  * sizeof(int));  // image_height * image_width * sizeof(int));
        //  memset(camera_points, 0, (dev_imgpro->point_num + 1) * 3 * sizeof(float)); //image_height * image_width * sizeof(float));
        // memset(dev_imgpro->camera_points, 0, (dev_imgpro->point_num + 1) * 3);  // image_height * image_width * sizeof(int));
        // memset(dev_imgpro->camera_points, 0, (dev_imgpro->point_num + 1) * 3); //image_height * image_width * sizeof(float));
    }

    ROS_INFO_STREAM("dev_imgpro->sum_of_point:" << dev_imgpro->sum_of_point);
    ROS_INFO_STREAM("dev_imgpro->sum_of_point(true):" << dev_imgpro->sum_of_point_num);

    float probability = (float)dev_imgpro->sum_of_point_num / (float)dev_imgpro->sum_of_point;

    ROS_INFO_STREAM("true num probability:" << probability);

    dev_imgpro->point_num = -1;
    ROS_INFO_STREAM("feature points num(cuda) flash:" << dev_imgpro->point_num);

    //cpu
    /*
    ROS_INFO_STREAM("start extracting feature points");

    if (imgRight.cols > 0 && imgRight.rows > 0)
    {

        Disparity_array_current = Disparity_array;
        int rows = imgRight.rows;
        int cols = imgLeft.cols;
        // vector<uchar> pointColors;
        int hitCounter = 0;
        int stopi = rows - blockSize;
        int startJ = 0;
        int stopJ = cols - (disparity + blockSize);
        if (disparity < 0)
        {
            startJ = -disparity;
            stopJ = cols - blockSize;
        }
        omp_set_num_threads(4); //stopi / blockSize + 1

#pragma omp parallel for
        for (int i = 0; i < stopi; i += blockSize)
        {
            //ROS_INFO_STREAM("I am Thread " << omp_get_thread_num() << " i:" << i);
            //#pragma omp parallel for
            for (int j = startJ; j < stopJ; j += blockSize)
            {
                // if(i == 0)
                // {
                //     ROS_INFO_STREAM("I am Thread " << omp_get_thread_num() << " j:" << j);
                // }
                int sad = getSAD(imgLeft, imgRight, laplacianLeft, laplacianRight, j, i);
                //ROS_INFO_STREAM("sad time1!");
                if (sad < sadThreshold && sad >= 0)
                {
                    // if (check_horizontal_invariance && checkHorizontalInvariance(imgLeft, imgRight, laplacianLeft, laplacianRight, j, i) == false)
                    // {
                    //ROS_INFO_STREAM("sad time2!");
                    all_count += 1;
                    //if (check_Disparity(j + blockSize / 2.0, i + blockSize / 2.0, Disparity_array_current) == true)
                    if (check_Depth(j + blockSize / 2.0, i + blockSize / 2.0) == true)
                    {
                        true_count += 1;
                        //unsigned char pxL = imgLeft.at<unsigned char>(i, j);
                        // pointColors.push_back(pxL);
                        //hitCounter++;
                        piex_obstacle_point.x = j;
                        piex_obstacle_point.y = i;
                        piex_obstacle_point.z = sad;
                        piex_point_pub.publish(piex_obstacle_point);
#pragma omp critical
                        {
                            localHitPoints.push_back(Point3f(j + blockSize / 2.0, i + blockSize / 2.0, true_disparity));
                            pointVector2d.push_back(Point3i(j, i, sad));
                        }
                    }
                    //}
                }
            }
        }
    }
    if (all_count != 0.0)
    {
        true_probobility = true_count / all_count;
        ROS_INFO_STREAM("depth true_count:" << true_count);
        ROS_INFO_STREAM("depth all_count:" << all_count);
        ROS_INFO_STREAM("depth true_probobility:" << true_probobility);
    }

    ROS_INFO_STREAM("finish extracting feature points");
     */
    //all_count = 0.0;
    //true_count = 0.0;
}
bool imgPro::checkHorizontalInvariance(Mat &leftImage, Mat &rightImage, Mat &sobelL, Mat &sobelR, int pxX, int pxY)
{
    int startX = pxX;
    int startY = pxY;
    int endX = pxX + blockSize - 1;
    int endY = pxY + blockSize - 1;

    if (startX + zero_disparity + INVARIANCE_CHECK_HORZ_OFFSET_MIN < 0 || endX + zero_disparity + INVARIANCE_CHECK_HORZ_OFFSET_MAX > rightImage.cols)
    {
        return true;
    }
    if (startY + INVARIANCE_CHECK_VERT_OFFSET_MIN < 0 || endY + INVARIANCE_CHECK_VERT_OFFSET_MAX > rightImage.rows)
    {
        return true;
    }

    int leftVal = 0;
    int right_val_array[400];
    int sad_array[400];
    int sobel_array[400];

    for (int i = 0; i < 400; i++)
    {
        right_val_array[i] = 0;
        sad_array[i] = 0;
        sobel_array[i] = 0;
    }
    int counter = 0;
    for (int i = startY; i <= endY; i++)
    {
        for (int j = startX; j <= endX; j++)
        {
            unsigned char pxL = leftImage.at<unsigned char>(i, j);
            unsigned char pxR_array[400], sR_array[400];
            counter = 0;
            for (int vert_offset = INVARIANCE_CHECK_VERT_OFFSET_MIN; vert_offset <= INVARIANCE_CHECK_VERT_OFFSET_MAX;
                 vert_offset += INVARIANCE_CHECK_VERT_OFFSET_INCREMENT)
            {
                for (int horz_offset = INVARIANCE_CHECK_HORZ_OFFSET_MIN; horz_offset <= INVARIANCE_CHECK_HORZ_OFFSET_MAX;
                     horz_offset++)
                {
                    pxR_array[counter] = rightImage.at<unsigned char>(i + vert_offset, j + zero_disparity + horz_offset);
                    sR_array[counter] = sobelR.at<unsigned char>(i + vert_offset, j + zero_disparity + horz_offset);
                    right_val_array[counter] += sR_array[counter];
                    sad_array[counter] += abs(pxL - pxR_array[counter]);
                    counter++;
                }
            }
            unsigned char sL = sobelL.at<unsigned char>(i, j);
            leftVal += sL;
        }
    }
    for (int i = 0; i < counter; i++)
    {
        sobel_array[i] = leftVal + right_val_array[i];
        if (right_val_array[i] >= sobelLimit && 333 * 1 * (float)sad_array[i] / ((float)sobel_array[i]) < sadThreshold)
        {                //counter = 8*8/2*2,每一个里面都是整个时视差窗口内12*12的总和，如果某一个counter内的值纹理丰富，则该点匹配，应当去掉
            return true; //是水平线，返回true
        }
    }
    return false;
}

bool imgPro::check_Disparity(int pxX, int pxY, std_msgs::Float32MultiArray &Disparity_array_)
{
    true_disparity = Disparity_array_.data.at(pxY * image_width + pxX - 1) * image_width;
    float dis_error = true_disparity - (-1 * disparity);

    if (dis_error >= down_error) //&& dis_error <= up_error
    {
        return true;
    }
    else
    {
        return false;
    }

    //return true;
}

bool imgPro::check_Depth(int pxX, int pxY)
{
    // cout << "x:" << pxX << "  y:" << pxY << endl;
    // cout << "深度:" << imgdepth.at<float>(pxY, pxX) << endl;
    float true_depth = imgdepth.at<float>(pxY, pxX);

    if (true_depth < 6.0 && true_depth > 3.0)
    {
        true_disparity = depth2disp_ratio / true_depth; //matQ.at<double>(2, 3) / (-1.0 * matQ.at<double>(3, 2) * true_depth * 1000);
        cout << "视差:" << true_disparity << endl;
        return true;
    }

    cout << "视差错误" << endl;
    return false;
}

int imgPro::getSAD(Mat &leftImage, Mat &rightImage, Mat &laplacianL, Mat &laplacianR, int pxX, int pxY)
{
    int startX = pxX;
    int startY = pxY;
    int endX = pxX + blockSize - 1;
    int endY = pxY + blockSize - 1;

    //if(endX)

    int leftVal = 0, rightVal = 0;
    int sad = 0;

    for (int i = startY; i <= endY; i++)
    {
        unsigned char *this_rowL = leftImage.ptr<unsigned char>(i);
        unsigned char *this_rowR = rightImage.ptr<unsigned char>(i);
        unsigned char *this_row_laplacianL = laplacianL.ptr<unsigned char>(i);
        unsigned char *this_row_laplacianR = laplacianR.ptr<unsigned char>(i);
        for (int j = startX; j <= endX; j++)
        {
            unsigned char sL = this_row_laplacianL[j];
            unsigned char sR = this_row_laplacianR[j + disparity];

            leftVal += sL;
            rightVal += sR;

            unsigned char pxL = this_rowL[j];
            unsigned char pxR = this_rowR[j + disparity];

            sad += abs(pxL - pxR);
        }
    }
    int laplacian_value = leftVal + rightVal;
    if (leftVal < sobelLimit && rightVal < sobelLimit)
    {
        //ROS_INFO_STREAM("don't have enough texture:"<<leftVal);
        return -1;
    }
    //ROS_INFO_STREAM("have enough texture:"<<leftVal);
    return 333 * (float)sad / (float)laplacian_value;
}

void imgPro::getImgRight(const sensor_msgs::ImageConstPtr &msg)
{
    flag = 2;

    // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_ASSERT(cv_ptr->image.channels() == 3 || cv_ptr->image.channels() == 1);

    if (cv_ptr->image.channels() == 3)
    {

        cvtColor(cv_ptr->image, imgRight, CV_RGB2GRAY);
        //ROS_INFO_STREAM("three channels");
    }
    else if (cv_ptr->image.channels() == 1)
    {
        cv_ptr->image.copyTo(imgRight);
    }
    //memcpy(dev_imgpro->src_img2, imgRight.data, image_height * image_width * sizeof(unsigned char));
    //memcpy(src_img2, imgRight.data, image_height * image_width * sizeof(unsigned char));
    // imshow("image",imgRight);
    //  image_pub_right.publish(cv_ptr->toImageMsg());
    //ROS_INFO_STREAM("get right image\r");
    //std::cout<<"get right image\r";
}
void imgPro::getImgLeft(const sensor_msgs::ImageConstPtr &msg)
{
    flag = 1;

    // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_ASSERT(cv_ptr->image.channels() == 3 || cv_ptr->image.channels() == 1);

    if (cv_ptr->image.channels() == 3)
    {

        cvtColor(cv_ptr->image, imgLeft, CV_RGB2GRAY);
    }
    else if (cv_ptr->image.channels() == 1)
    {
        cv_ptr->image.copyTo(imgLeft);
    }

    //memcpy(dev_imgpro->src_img1, imgLeft.data, image_height * image_width * sizeof(unsigned char));
    //memcpy(src_img1, imgLeft.data, image_height * image_width * sizeof(unsigned char));
    //  image_pub_left.publish(cv_ptr->toImageMsg());
    //ROS_INFO_STREAM("get left image\r");
    //std::cout<<"get left image\r";
}

void imgPro::getImgDepth(const sensor_msgs::ImageConstPtr &msg)
{

    // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    //memcpy((float*)imgdepth.data, (float *)&msg->data[0],  image_height * image_width * sizeof(float));

    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_ASSERT(cv_ptr->image.channels() == 3 || cv_ptr->image.channels() == 1);

    if (cv_ptr->image.channels() == 3)
    {

        cvtColor(cv_ptr->image, imgdepth, CV_RGB2GRAY);
    }
    else if (cv_ptr->image.channels() == 1)
    {
        cv_ptr->image.copyTo(imgdepth);
    }

    //memcpy(src_img2, imgRight.data, image_height * image_width * sizeof(unsigned char));
    //memcpy(src_img1, imgLeft.data, image_height * image_width * sizeof(unsigned char));
    memcpy(dev_imgpro->src_img2, imgRight.data, image_height * image_width * sizeof(unsigned char));
    memcpy(dev_imgpro->src_img1, imgLeft.data, image_height * image_width * sizeof(unsigned char));
    memcpy((float *)dev_imgpro->zed_depth_img, imgdepth.data, image_height * image_width * sizeof(float)); //(float *)&msg->data[0],

    //  imshow("image",imgdepth);
    //   cv::waitKey(0);

    //ROS_INFO_STREAM("get depth image\r");
    //std::cout<<"get depth image\r";
    image_num++;

    //std::cout << "Disparity float value:" << imgdepth.at<float>(107, 330) << std::endl;
}

void imgPro::LaplacianPro()
{
    // cv::Laplacian(imgLeft, laplacianLeft, -1, 3, 1, 0, BORDER_DEFAULT);
    // cv::Laplacian(imgRight, laplacianRight, -1, 3, 1, 0, BORDER_DEFAULT);

    // memcpy(src_img1, imgLeft.data, image_height * image_width * sizeof(unsigned char));
    // //cudaMemcpyHostToDevice);

    // memcpy(src_img2, imgRight.data, image_height * image_width * sizeof(unsigned char));
    // //cudaMemcpyHostToDevice);
    laplacian_uma_gpu(src_img1, lapla_img1, laplacianLeft, image_height, image_width);
    laplacian_uma_gpu(src_img2, lapla_img2, laplacianRight, image_height, image_width);
    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", laplacianLeft).toImageMsg(); //mono8
    // image_pub_laplacianLeft.publish(msg);

    ROS_INFO_STREAM("laplacian operator!");
}

void imgPro::call_Disparitymsg(const std_msgs::Float32MultiArray &msg)
{
    Disparity_array = msg;
    memcpy((float *)dev_imgpro->depth_img, (float *)&Disparity_array.data[0], image_height * image_width * sizeof(float));
    //std::cout << "Disparity float value:" << Disparity_array.data.at(40000) << std::endl;
}

//边缘检测CPU函数
void imgPro::laplacian_cpu(Mat &srcImg, Mat &dstImg, int imgHeight, int imgWidth)
{
    int Gx = 0;
    int Gy = 0;
    int gl = 0;
    omp_set_num_threads(4);

#pragma omp parallel for
    for (int i = 1; i < imgHeight - 1; i++)
    {
        uchar *dataUp = srcImg.ptr<uchar>(i - 1);
        uchar *data = srcImg.ptr<uchar>(i);
        uchar *dataDown = srcImg.ptr<uchar>(i + 1);
        uchar *out = dstImg.ptr<uchar>(i);
        for (int j = 1; j < imgWidth - 1; j++)
        {
            //     Gx = (dataUp[j + 1] + 2 * data[j + 1] + dataDown[j + 1]) -
            //          (dataUp[j - 1] + 2 * data[j - 1] + dataDown[j - 1]);
            //     Gy = (dataUp[j - 1] + 2 * dataUp[j] + dataUp[j + 1]) -
            //          (dataDown[j - 1] + 2 * dataDown[j] + dataDown[j + 1]);
            //     out[j] = (abs(Gx) + abs(Gy)) / 2;

            gl = 4 * data[j] -
                 data[j + 1] -
                 data[j - 1] -
                 dataUp[j] -
                 dataDown[j];
            out[j] = abs(gl);
        }
    }
}
