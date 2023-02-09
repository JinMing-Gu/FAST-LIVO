
#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H
#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <fast_livo/States.h>
#include <geometry_msgs/Vector3.h>

/// *************Preconfiguration

#define MAX_INI_COUNT (200)

const bool time_list(PointType &x, PointType &y); //{return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
    void push_update_state(double offs_t, StatesGroup state);
    void set_extrinsic(const Eigen::Vector3d &transl, const Eigen::Matrix3d &rot);
    void set_extrinsic(const Eigen::Vector3d &transl);
    void set_extrinsic(const MD(4, 4) & T);
    void set_gyr_cov_scale(const Eigen::Vector3d &scaler);
    void set_acc_cov_scale(const Eigen::Vector3d &scaler);
    void set_gyr_bias_cov(const Eigen::Vector3d &b_g);
    void set_acc_bias_cov(const Eigen::Vector3d &b_a);
    void Process(const LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
    void Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
    void UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

    ros::NodeHandle nh;
    ofstream fout_imu;
    Eigen::Vector3d cov_acc;
    Eigen::Vector3d cov_gyr;
    Eigen::Vector3d cov_acc_scale;
    Eigen::Vector3d cov_gyr_scale;
    Eigen::Vector3d cov_bias_gyr;
    Eigen::Vector3d cov_bias_acc;
    double first_lidar_time;

private:
    void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);
    void Forward(const MeasureGroup &meas, StatesGroup &state_inout, double pcl_beg_time, double end_time);
    void Backward(const LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

    PointCloudXYZI::Ptr cur_pcl_un_;
    sensor_msgs::ImuConstPtr last_imu_;
    deque<sensor_msgs::ImuConstPtr> v_imu_;
    vector<Pose6D> IMUpose;
    vector<Eigen::Matrix3d> v_rot_pcl_;
    Eigen::Matrix3d Lid_rot_to_IMU;
    Eigen::Vector3d Lid_offset_to_IMU;
    Eigen::Vector3d mean_acc;
    Eigen::Vector3d mean_gyr;
    Eigen::Vector3d angvel_last;
    Eigen::Vector3d acc_s_last;
    Eigen::Vector3d last_acc;
    Eigen::Vector3d last_ang;
    double start_timestamp_;
    double last_lidar_end_time_;
    int init_iter_num = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};
#endif
