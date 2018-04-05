#include "odom_calc.h"
#include "tf/tf.h"

#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"

inline static ::Eigen::Quaterniond fixOriEigen(const ::Eigen::Quaterniond &input){
  ::Eigen::Quaterniond correction;
  //auto tf_correction = tf::createQuaternionFromYaw(M_PI/2);
  //auto tf_correction = tf::createQuaternionFromRPY(M_PI,0,M_PI/2);
  auto tf_correction = tf::createQuaternionFromRPY(M_PI,0,0);
  ::tf::quaternionTFToEigen(tf_correction, correction);
  return correction * input;
}


inline static ::Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion &msg){
  ::Eigen::Quaterniond result;
  ::tf::quaternionMsgToEigen(msg, result);
  return result;
}

OdomCalculator::OdomCalculator():
  delta_l_enc_(0),
  delta_r_enc_(0),
  r_enc_prev_(0),
  l_enc_prev_(0),
  first_time_(true),
  pose_X_(0),
  pose_Y_(0),
  pose_Yaw_(0),
  tick_distance_(0.00206),
  wtw_distance_(1.56),
  encoder_only_(false),
  imuodom_as_rigid3d_ (cartographer::transform::Rigid3d())
{
}

#define FILTER_THRESHOLD 5000 //1500
bool OdomCalculator::Process(const double &imu_val, const int &enc_l_val, const int &enc_r_val, const ::ros::Time &curr_ros_time)
{
  if (first_time_){
    // cache value only 
    l_enc_prev_ = enc_l_val;
    r_enc_prev_ = enc_r_val;
    first_time_ = false;
    return false;
  }
  int l_enc = enc_l_val;
  int r_enc = enc_r_val;
  int delta_l_enc=l_enc-l_enc_prev_;
  int delta_r_enc=r_enc_prev_-r_enc; //Right encoder is in the reverse direction
  // filtering
  if (abs(delta_l_enc) > FILTER_THRESHOLD) {
    printf("delta_l_enc jumped!!! %d\n", delta_l_enc);
    ROS_ERROR("delta_l_enc jumped!!! %d\n", delta_l_enc);
    delta_l_enc = 0;
    delta_r_enc = 0;
  }

  if (abs(delta_r_enc) > FILTER_THRESHOLD) {
    printf("delta_r_enc jumped!!! %d\n", delta_r_enc);
    ROS_ERROR("delta_r_enc jumped!!! %d\n", delta_r_enc);
    delta_r_enc = 0;
    delta_l_enc = 0;
  }
  l_enc_prev_=l_enc;
  r_enc_prev_=r_enc;

  // convert from encoder ticks to distance
  double l_wd = delta_l_enc * tick_distance_;
  double r_wd = delta_r_enc * tick_distance_;

  // we use Imu for delta_theta
  // Imu z-axis is pointing downwards
  double imu_delta_thet, delta_thet, delta_x, delta_y;
  if (encoder_only_)
  {
      delta_thet  = (r_wd - l_wd)/wtw_distance_;
  }
  else
  {
      imu_delta_thet         = - imu_val * 0.02;
  }
  //double delta_thet	= (r_wd - l_wd)/wtw_distance_;
  double delta_v		= (l_wd + r_wd)/2.0;
  if (encoder_only_)
  {
      delta_x		= delta_v * cos(pose_Yaw_ + delta_thet/2.0);
      delta_y		= delta_v * sin(pose_Yaw_ + delta_thet/2.0);
      LOG(INFO) << "delta_x: " << delta_x << " delta_y: " << delta_y;
  }
  else
  {
      delta_x		= delta_v * cos(pose_Yaw_ + imu_delta_thet/2.0);
      delta_y		= delta_v * sin(pose_Yaw_ + imu_delta_thet/2.0);
  }
  //double delta_x		= delta_v * cos(pose_Yaw_ + delta_thet/2.0);
  //double delta_y		= delta_v * sin(pose_Yaw_ + delta_thet/2.0);

  if (false == encoder_only_)
  {
      pose_Yaw_ += imu_delta_thet;
  }
  else
  {
      pose_Yaw_ += delta_thet;
  }
  //pose_Yaw_ += delta_thet;
  pose_X_   += delta_x;
  pose_Y_   += delta_y;
  LOG(INFO) << "Pose_X: " << pose_X_ << " Pose_Y: " << pose_Y_;

  pose_stamp_ = curr_ros_time;

  odom_imu_.header.stamp =  curr_ros_time ;
  odom_imu_.header.frame_id = "/odom_imu";

  //setimu_ the position
  odom_imu_.pose.pose.position.x = pose_X_;
  odom_imu_.pose.pose.position.y = pose_Y_;
  odom_imu_.pose.pose.position.z = 0.0;
  ::geometry_msgs::Quaternion imu_odom_quat = ::tf::createQuaternionMsgFromYaw(pose_Yaw_);
  //ROS_INFO("imu_pose_Yaw: %.6f", imu_pose_Yaw);
  odom_imu_.pose.pose.orientation = imu_odom_quat;
  return true;
}

void OdomCalculator::ProcessImuOdom(const sensor_msgs::Imu &imu, const int &enc_l_val, const int &enc_r_val, const ::ros::Time &curr_ros_time)
{
    if (first_time_){
	// cache value only 
	l_enc_prev_ = enc_l_val;
	r_enc_prev_ = enc_r_val;
	first_time_ = false;
	//current_imu_ori_ = fixOriEigen(toEigen(imu.orientation));
	current_imu_ori_ = toEigen(imu.orientation);
	ROS_INFO("imu in calc: %f %f %f %f %f",  imu.header.stamp.toSec(), imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	//LOG(INFO) << current_imu_ori_.x() << current_imu_ori_.y() << current_imu_ori_.z() << current_imu_ori_.w();
	return;
    }

    ::Eigen::Quaterniond last_imu_ori ( current_imu_ori_);
    //current_imu_ori_ = fixOriEigen(toEigen(imu.orientation));
    current_imu_ori_ = toEigen(imu.orientation);
    //LOG(INFO) << last_imu_ori.x() << last_imu_ori.y() << last_imu_ori.z() << last_imu_ori.w();
    //LOG(INFO) << current_imu_ori_.x() << current_imu_ori_.y() << current_imu_ori_.z() << current_imu_ori_.w();
    if (std::isnan(current_imu_ori_.x()) || std::isnan(current_imu_ori_.y()) || std::isnan(current_imu_ori_.z()) || std::isnan(current_imu_ori_.w()) )
    {
	LOG(FATAL) << current_imu_ori_.x() << current_imu_ori_.y() << current_imu_ori_.z() << current_imu_ori_.w();
    }
    auto delta_q = last_imu_ori.inverse() * current_imu_ori_;
    //LOG(INFO) << delta_q.x() << " " << delta_q.y() << " " << delta_q.z() << " " << delta_q.w();

    int l_enc = enc_l_val;
    int r_enc = enc_r_val;
    int delta_l_enc=l_enc-l_enc_prev_;
    int delta_r_enc=r_enc_prev_-r_enc; //Right encoder is in the reverse direction

    // filtering
    if (abs(delta_l_enc) > FILTER_THRESHOLD) {
	printf("delta_l_enc jumped!!! %d\n", delta_l_enc);
	ROS_ERROR("delta_l_enc jumped!!! %d\n", delta_l_enc);
	delta_l_enc = 0;
	delta_r_enc = 0;
    }

    if (abs(delta_r_enc) > FILTER_THRESHOLD) {
	printf("delta_r_enc jumped!!! %d\n", delta_r_enc);
	ROS_ERROR("delta_r_enc jumped!!! %d\n", delta_r_enc);
	delta_r_enc = 0;
	delta_l_enc = 0;
    }
    l_enc_prev_=l_enc;
    r_enc_prev_=r_enc;

    // convert from encoder ticks to distance
    double l_wd = delta_l_enc * tick_distance_;
    double r_wd = delta_r_enc * tick_distance_;

    //// we use Imu for delta_theta
    //// Imu z-axis is pointing downwards
    //double imu_delta_thet, delta_thet, delta_x, delta_y;
    double delta_v = (l_wd + r_wd) / 2;
    cartographer::transform::Rigid3d delta_T (::Eigen::Vector3d(delta_v, 0,0), delta_q);
    //LOG(INFO) << delta_T;
    // update result
    imuodom_as_rigid3d_ = imuodom_as_rigid3d_ * delta_T;
}

const cartographer::transform::Rigid3d & OdomCalculator::GetRigid3d()
{
    odom_as_rigid3d_= cartographer::transform::Rigid3d(cartographer::transform::Rigid3d::Vector(pose_X_, pose_Y_, 0), cartographer::transform::RollPitchYaw(0,0,pose_Yaw_));
    return odom_as_rigid3d_;
}

const cartographer::transform::Rigid3d & OdomCalculator::GetImuOdom()
{
    return imuodom_as_rigid3d_;
}

