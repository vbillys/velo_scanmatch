#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "transform.h"
#include "eigen_conversions/eigen_msg.h"

class OdomCalculator
{
  public:
    explicit OdomCalculator();
    ~OdomCalculator(){};
    OdomCalculator(const OdomCalculator&) = delete;
    OdomCalculator& operator=(const OdomCalculator&) = delete;

    bool Process(const double &imu_val, const int &enc_l_val, const int &enc_r_val, const ::ros::Time &curr_ros_time);
    const nav_msgs::Odometry& GetOdometryMsg()
    {
	return odom_imu_;
    }
    void setEncoderOnly(const bool & flag)
    {
	encoder_only_ = flag;
    }
    const cartographer::transform::Rigid3d & GetRigid3d();

    void ProcessImuOdom(const sensor_msgs::Imu &imu, const int &enc_l_val, const int &enc_r_val, const ::ros::Time &curr_ros_time);
    const cartographer::transform::Rigid3d & GetImuOdom();

    static inline ::Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion &msg) {
        ::Eigen::Quaterniond result;
        ::tf::quaternionMsgToEigen(msg, result);
        return result;
}

  private:
    int delta_l_enc_;
    int delta_r_enc_;
    int l_enc_prev_;
    int r_enc_prev_;
    bool first_time_;
    double pose_Yaw_;
    double pose_X_;
    double pose_Y_;
    ::ros::Time pose_stamp_;
    nav_msgs::Odometry odom_imu_;
    cartographer::transform::Rigid3d odom_as_rigid3d_;
    double tick_distance_;
    double wtw_distance_;
    bool encoder_only_;

    ::Eigen::Quaterniond current_imu_ori_;

    cartographer::transform::Rigid3d imuodom_as_rigid3d_;
};


