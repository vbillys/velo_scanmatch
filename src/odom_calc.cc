#include "odom_calc.h"
#include "tf/tf.h"

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
  encoder_only_(false)
{
}

#define FILTER_THRESHOLD 5000 //1500
void OdomCalculator::Process(const double &imu_val, const int &enc_l_val, const int &enc_r_val, const ::ros::Time &curr_ros_time)
{
  if (first_time_){
    // cache value only 
    l_enc_prev_ = enc_l_val;
    r_enc_prev_ = enc_r_val;
    first_time_ = false;
    return;
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
      double delta_x		= delta_v * cos(pose_Yaw_ + delta_thet/2.0);
      double delta_y		= delta_v * sin(pose_Yaw_ + delta_thet/2.0);
  }
  else
  {
      double delta_x		= delta_v * cos(pose_Yaw_ + imu_delta_thet/2.0);
      double delta_y		= delta_v * sin(pose_Yaw_ + imu_delta_thet/2.0);
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
}

const cartographer::transform::Rigid3d & OdomCalculator::GetRigid3d()
{
    odom_as_rigid3d_= cartographer::transform::Rigid3d(cartographer::transform::Rigid3d::Vector(pose_X_, pose_Y_, 0), cartographer::transform::RollPitchYaw(0,0,pose_Yaw_));
    return odom_as_rigid3d_;
}

