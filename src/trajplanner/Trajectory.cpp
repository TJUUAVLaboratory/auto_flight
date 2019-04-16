/*
 * Trajectory class representing a trajectory in state space
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013-2015
 *
 */

#include "ros_traj_plan/Trajectory.hpp"

Trajectory::Trajectory()
{ //构造函数
    trajectory_number_ = -1;
    dimension_ = 0;
    udimension_ = 0;
    filename_prefix_ = "";
    dt_ = 0;
}
//Constructor that for straight trajectory
Trajectory::Trajectory(int trajectory_number, Eigen::MatrixXd matrix, int dimension, double dt, double min_altitude)
{
    xpoints_ = matrix;
    trajectory_number_ = trajectory_number;
    dimension_ = dimension;
    dt_ = dt;
    min_altitude_ = min_altitude;
}
// Constructor that loads a trajectory from a file
Trajectory::Trajectory(std::string filename_prefix, bool quiet) : Trajectory()
{
    LoadTrajectory(filename_prefix, quiet);
}

void Trajectory::LoadTrajectory(std::string filename_prefix, bool quiet)
{
    // open the file
    std::vector<std::vector<std::string>> strs;

    if (!quiet)
    {
        std::cout << "Loading trajectory: " << std::endl
                  << "\t" << filename_prefix << std::endl;
    }

    std::string traj_number_str = filename_prefix.substr(filename_prefix.length() - 5, 5); //提取文件名的后5个字符
    trajectory_number_ = std::stoi(traj_number_str);                                       //将字符转换成数值

    LoadMatrixFromCSV(filename_prefix + "-x.csv", xpoints_, quiet);
    // LoadMatrixFromCSV(filename_prefix + "-u.csv", upoints_, quiet);
    // LoadMatrixFromCSV(filename_prefix + "-controller.csv", kpoints_, quiet);
    // LoadMatrixFromCSV(filename_prefix + "-affine.csv", affine_points_, quiet);

    filename_prefix_ = filename_prefix;

    dimension_ = xpoints_.cols() - 1; // minus 1 because of time index
    udimension_ = upoints_.cols() - 1;
    /*
 if (kpoints_.cols() - 1 != dimension_ * udimension_) {
        std::cerr << "Error: expected to have " << dimension_ << "*" << udimension_ << "+1 = " << dimension_ * udimension_ + 1 << " columns in " << filename_prefix << "-controller.csv but found " << kpoints_.cols() << std::endl;
        exit(1);
    }

    if (affine_points_.cols() - 1 != udimension_) {
        std::cerr << "Error: expected to have " << udimension_ << "+1 = " << udimension_ + 1 << " columns in " << filename_prefix << "-affine.csv but found " << affine_points_.cols() << std::endl;
        exit(1);
    }

    if (upoints_.rows() != kpoints_.rows() || upoints_.rows() != affine_points_.rows()) {
        std::cerr << "Error: inconsistent number of rows in CSV files: " << std::endl
            << "\t" << filename_prefix << "-x: " << xpoints_.rows() << std::endl
            << "\t" << filename_prefix << "-u: " << upoints_.rows() << std::endl
            << "\t" << filename_prefix << "-controller: " << kpoints_.rows() << std::endl
            << "\t" << filename_prefix << "-affine: " << affine_points_.rows() << std::endl;

            exit(1);
    }
*/

    // set the minimum altitude
    uavTrans trans_initial;
    trans_initial = set_trans_init();
    bool first_run = true;

    for (int i = 0; i < GetNumberOfPoints(); i++)
    {

        double this_t = GetTimeAtIndex(i);
        float xyz[3];

        GetXyzYawTransformedPoint(this_t, trans_initial, xyz);

        if (xyz[2] < min_altitude_ || first_run == true)
        {
            min_altitude_ = xyz[2];
            first_run = false;
        }
    }
}

void Trajectory::LoadMatrixFromCSV(const std::string &filename, Eigen::MatrixXd &matrix, bool quiet)
{

    if (!quiet)
    {
        std::cout << "Loading " << filename << std::endl;
    }

    int number_of_lines = GetNumberOfLines(filename); //矩阵行数
    int row_num = 0;

    int i = 0;
    //                                   file, delimiter, first_line_is_header?
    CsvParser *csvparser = CsvParser_new(filename.c_str(), ",", true);
    CsvRow *header;
    CsvRow *row;

    header = CsvParser_getHeader(csvparser);
    if (header == NULL)
    {
        printf("%s\n", CsvParser_getErrorMessage(csvparser));
        return;
    }

    // note: do not remove the getFields(header) call as it has
    // side effects we need even if we don't use the headers

    //char **headerFields = CsvParser_getFields(header);
    CsvParser_getFields(header);
    for (i = 0; i < CsvParser_getNumFields(header); i++)
    {
        //printf("TITLE: %s\n", headerFields[i]);
    } //矩阵列数

    matrix.resize(number_of_lines - 1, i); // minus 1 for header, i = number of columns     //去掉表头

    while ((row = CsvParser_getRow(csvparser)))
    { //不停循环，直到达到最后一行
        //printf("NEW LINE:\n");
        char **rowFields = CsvParser_getFields(row);
        for (i = 0; i < CsvParser_getNumFields(row); i++)
        {

            matrix(row_num, i) = atof(rowFields[i]);

            //printf("FIELD: %20.20f\n", atof(rowFields[i]));
        }
        CsvParser_destroy_row(row);

        if (row_num == 1)
        {
            dt_ = matrix(1, 0) - matrix(0, 0);
        }
        else if (row_num > 1)
        {
            if (matrix(row_num, 0) - matrix(row_num - 1, 0) - dt_ > 5 * std::numeric_limits<double>::epsilon())
            {
                std::cerr << "Error: non-constant dt. Expected dt = " << dt_ << " but got matrix[" << row_num << "][0] - matrix[" << row_num - 1 << "][0] = " << matrix(row_num, 0) - matrix(row_num - 1, 0) << " (residual = " << (matrix(row_num, 0) - matrix(row_num - 1, 0) - dt_) << std::endl;

                std::cout << matrix << std::endl;
                exit(1);
            }
        }

        row_num++;
    }
    CsvParser_destroy(csvparser);
}

int Trajectory::GetNumberOfLines(std::string filename) const
{
    int number_of_lines = 0;
    std::string line;
    std::ifstream myfile(filename);

    while (getline(myfile, line))
    {
        ++number_of_lines;
    }

    return number_of_lines;
}

Eigen::VectorXd Trajectory::GetState(double t) const
{
    int index = GetIndexAtTime(t);
    Eigen::VectorXd row_vec = xpoints_.row(index);

    return row_vec.tail(xpoints_.cols() - 1); // remove time
}
/*Eigen::VectorXd Trajectory::GetUCommand(double t) const {
    int index = GetIndexAtTime(t);

    Eigen::VectorXd row_vec = upoints_.row(index);

    return row_vec.tail(upoints_.cols() - 1); // remove time
}*/

/**
 * Assuming a constant dt, we can compute the index of a point
 * based on its time.
 *
 * @param t time to find index of
 * @param (optional) use_rollout set to true to use time bounds from rollout instead of xpoints
 * @retval index of nearest point
 */
int Trajectory::GetIndexAtTime(double t) const
{

    // round t to the nearest dt_

    double t0, tf;

    t0 = xpoints_(0, 0);
    tf = xpoints_(xpoints_.rows() - 1, 0); //去掉标头

    if (t <= t0)
    {
        return 0;
    }
    else if (t >= tf)
    {

        return xpoints_.rows() - 1;
    }
    //std::cout << "after" << std::endl;
    // otherwise, we are somewhere in the bounds of the trajectory
    int num_dts = std::floor(t / dt_);
    float remainder = std::remainder(t, dt_);
    //std::cout << "t = " << t << " remainder = " << remainder << " num_dts = " << num_dts << std::endl;
    if (remainder > 0.5f * dt_)
    {
        num_dts++;
    }

    int starting_dts = t0 / dt_;

    return num_dts + starting_dts;
}

/**
 * Unpacks the gain matrix for a specific time t.
 *
 * This could be pre-computed if it becomes a performance bottleneck.
 *
 * @param t time along the trajectory
 *
 * @retval gain matrix at that time with dimension: state_dimension x u_dimension


 Eigen::MatrixXd Trajectory::GetGainMatrix(double t) const {
 int index = GetIndexAtTime(t);

 Eigen::VectorXd k_row = kpoints_.row(index);


 Eigen::MatrixXd k_mat(udimension_, dimension_);

 for (int i = 0; i < udimension_; i++) {

 // +1 because the column index is time
 int start_pos = i * dimension_ + 1;

 k_mat.row(i) = k_row.segment(start_pos, dimension_);
 }

 return k_mat;


 }

 */

void Trajectory::Print() const
{
    std::cout << "------------ Trajectory print -------------" << std::endl;
    std::cout << "Filename: " << filename_prefix_ << std::endl;
    std::cout << "Trajectory number: " << trajectory_number_ << std::endl;
    std::cout << "Dimension: " << dimension_ << std::endl;
    //    std::cout << "u-dimension: " << udimension_ << std::endl;

    std::cout << " t\t   x\t    y\t   z\t  " << std::endl; //roll\t pitch\t yaw

    std::cout << xpoints_ << std::endl;
    /* std::cout << "------------- u points ----------------" << std::endl;

    std::cout << " t\t u1\t u2\t u3" << std::endl;

    std::cout << upoints_ << std::endl;

    std::cout << "------------- k points ----------------" << std::endl;

    std::cout << kpoints_ << std::endl;

    std::cout << "------------- affine points ----------------" << std::endl;

    std::cout << affine_points_ << std::endl;*/
}

//* Returns a point along the trajectory transformed with xyz and yaw.  Ignores pitch and roll.

void Trajectory::GetXyzYawTransformedPoint(double t, const uavTrans &transform, float xyz[]) const
{
    // apply the transformation from the global frame: orgin = (0,0,0)
    // to the local frame point

    Eigen::VectorXd state = GetState(t);

    double original_point[3];
    original_point[0] = state(0);
    original_point[1] = state(1);
    original_point[2] = state(2);

    // remove roll and pitch from the transform
    uavTrans trans_xyz_yaw;

    bot_trans_copy(&trans_xyz_yaw, &transform);

    double rpy[3];
    bot_quat_to_roll_pitch_yaw(trans_xyz_yaw.uav_quat, rpy);
    //std::cout<<"rpy角度\n"<<rpy[0]*180/3.1415<<"\n"<<rpy[1]*180/3.1415<<"\n"<<rpy[2]*180/3.1415<<std::endl;

    rpy[0] = 0;
    rpy[1] = 0;

    bot_roll_pitch_yaw_to_quat(rpy, trans_xyz_yaw.uav_quat);

    board_to_world(&trans_xyz_yaw, original_point, xyz);
    //std::cout<<"转换坐标（加自身位置）\n"<<xyz[0]<<"\n"<<xyz[1]<<"\n"<<xyz[2]<<std::endl;
}

/**
 * Searches along the remainder of the trajectory, assuming it executes exactly from the
 * current position.
 *
 * Returns the distance to the closest obstacle over that search.  Used for determining if
 * replanning is required
 *
 * @param octomap Obstacle map
 * @param bodyToLocal Current position of the aircraft
 * @param current_t Time along the trajectory
 *
 * @retval Distance to the closest obstacle along the remainder of the trajectory
 
 */

double Trajectory::ClosestObstacleInRemainderOfTrajectory(double current_t, uavTrans &body_to_local, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *octree_map_, double min_altitude_allowed) const
{

    // for each point remaining in the trajectory
    int number_of_points = 9; //GetNumberOfPoints() - 6;

    int starting_index = 0;                                //GetIndexAtTime(current_t);
    std::vector<double> point_distances(number_of_points); // TODO: potentially inefficient
    double closest_obstacle_distance = -1;

    for (int i = starting_index; i < number_of_points; i++)
    {
        // for each point in the trajectory

        // subtract the current position (ie move the trajectory to where we are)
        float transformed_point[3];

        //直接利用未进行坐标系转化的位置坐标
        double this_t = GetTimeAtIndex(i);
        /*
        Eigen::VectorXd state = GetState(this_t);
		transformed_point[0] = state(0);
		transformed_point[1] = state(1);
		transformed_point[2] = state(2);
        */

        GetXyzYawTransformedPoint(this_t, body_to_local, transformed_point);

        // check if there is an obstacle nearby
        point_distances.at(i) = NearestNeighbor(transformed_point, octree_map_);
    }

    for (int i = starting_index; i < number_of_points; i++)
    {
        double distance_to_point = point_distances.at(i);
        if (distance_to_point >= 0)
        {
            if (distance_to_point < closest_obstacle_distance || closest_obstacle_distance < 0)
            {
                closest_obstacle_distance = distance_to_point;
            }
        }
    }

    // check minumum altitude
    double min_altitude_traj = GetMinimumAltitude() + body_to_local.uav_position[2];
    if (min_altitude_traj < min_altitude_allowed)
    {
        // this trajectory would impact the ground
        closest_obstacle_distance = 0; //这条路劲不符合条件
    }                                  // else if (min_altitude_traj < closest_obstacle_distance || closest_obstacle_distance < 0) {
                                       //   closest_obstacle_distance = min_altitude_traj;
                                       //}

    return closest_obstacle_distance;
}