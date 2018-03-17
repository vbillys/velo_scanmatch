#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "trajectory.pb.h"
#include "proto_stream.h"
#include "time_conversion.h"
#include "transform.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/NavSatFix.h"

#include "odom_calc.h"

DEFINE_string(bag_filenames, "",
    "Bags to process, must be in the same order as the trajectories "
    "in 'pose_graph_filename'.");

DEFINE_string(odom_stream_filename, "",
    "File to output to contain the timestamped odometry data.");

DEFINE_bool(process_raw, false, "Include 'advanced' using raw encoder and imu data");
DEFINE_bool(gps_mode, false, "Using GNSS data to extract");

using namespace std;

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";

  ros::init(argc, argv, "extractOdom");
  ros::NodeHandle nh, pnh("~");

  std::string pc_topic;
  pnh.param<std::string>("odom_topic"  , pc_topic, "/imuodom");
  std::string gnss_topic;
  pnh.param<std::string>("gnss_topic"  , gnss_topic, "/an_device/NavSatFix_metric");

  // for raw processing
  std::string encoder_left_topic, encoder_right_topic, imu_topic;
  pnh.param<std::string>("encoder_left_topic"  , encoder_left_topic, "/encoder_left");
  pnh.param<std::string>("encoder_right_topic"  , encoder_right_topic, "/encoder_right");
  pnh.param<std::string>("imu_topic"  , imu_topic, "/an_device/Imu");
  double wtw_distance, tick_distance;
  pnh.param<double>("wtw_distance" , wtw_distance , 1.56 );
  pnh.param<double>("tick_distance" , tick_distance , 0.00206);

  double sensor_pose[6];
  pnh.param<double>("sensor_pose_x" , sensor_pose[0] , 0.0);
  pnh.param<double>("sensor_pose_y" , sensor_pose[1] , 0.0);
  pnh.param<double>("sensor_pose_z" , sensor_pose[2] , 0.0);
  pnh.param<double>("sensor_pose_roll" , sensor_pose[3] , 0.0);
  pnh.param<double>("sensor_pose_pitch" , sensor_pose[4] , 0.0);
  pnh.param<double>("sensor_pose_yaw" , sensor_pose[5] , 0.0);
  cartographer::transform::Rigid3d sensor_pose_rigid3d = cartographer::transform::Rigid3d(cartographer::transform::Rigid3d::Vector(sensor_pose[0], sensor_pose[1], sensor_pose[2]), cartographer::transform::RollPitchYaw(sensor_pose[3],sensor_pose[4],sensor_pose[5]));

  enum StringCode {
    eImu,
    eEncoderLeft,
    eEncoderRight,
    eGnss
  };
  std::map<std::string, StringCode > s_mapStringToStringCode =
  {
      { imu_topic, eImu },
      { encoder_left_topic, eEncoderLeft },
      { encoder_right_topic, eEncoderRight },
      { gnss_topic, eGnss }
  };

  rosbag::Bag bag;
  bag.open(FLAGS_bag_filenames, rosbag::bagmode::Read);
  ROS_INFO("Using bag file: %s", FLAGS_bag_filenames.c_str());

  std::vector<std::string> topics;
  if (FLAGS_gps_mode)
  {
      topics.push_back(gnss_topic);
      topics.push_back(imu_topic);
  }
  else if (FLAGS_process_raw)
  {
      topics.push_back(encoder_left_topic);
      topics.push_back(encoder_right_topic);
      topics.push_back(imu_topic);
  }
  else
  {
      topics.push_back(pc_topic);
  }
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  cartographer::mapping::proto::Trajectory g_traj;
  std::vector<cartographer::transform::proto::Rigid3d> proto_rigid3ds;

  if (FLAGS_gps_mode)
  {
      bool gps_harvest = false;
      bool gnss_sampled = false;
      bool imu_sampled = false;
      ::ros::Time last_stamp_imu , last_stamp_gnss;

      sensor_msgs::Imu::Ptr imu_msg;
      sensor_msgs::NavSatFix::Ptr gnss_msg;

      for (const rosbag::MessageInstance& message : view) {
	  StringCode topic_key = s_mapStringToStringCode[message.getTopic()];

	  switch (topic_key)
	  {
	      case eImu:
		  imu_msg = message.instantiate<sensor_msgs::Imu>();
		  imu_sampled = true;
		  last_stamp_imu = message.getTime();
		  //ROS_INFO("imu: %f",  imu_msg->header.stamp.toSec());
		  //ROS_INFO("imu: %f",  message.getTime().toSec());
		  break;
	      case eGnss:
		  gnss_msg = message.instantiate<sensor_msgs::NavSatFix>();
		  gnss_sampled = true;
		  last_stamp_gnss = message.getTime();
		  //ROS_INFO("gnss: %f",  gnss_msg->header.stamp.toSec());
		  //ROS_INFO("gnss: %f",  message.getTime().toSec());
		  break;
	      default:
		  break;
	  }
//#if 0
	  //we do simple check, if both gnss and imu are inited, we check time diff
	  // if time diff too big, we drop the older data
	  if (imu_sampled && gnss_sampled)
	  {
	      if (false == gps_harvest)
	      {
		  if (last_stamp_gnss.toSec() - last_stamp_imu.toSec() >= 0.005) //assuming 100 Hz
		  {
		      imu_sampled = false;
		  }
		  else if (last_stamp_imu.toSec() - last_stamp_gnss.toSec() >= 0.005) //assuming 100 Hz
		  {
		      gnss_sampled = false;
		  }
		  else 
		      gps_harvest= true;
	      }

	      if (gps_harvest)
	      {
		  auto new_node = g_traj.add_node();
		  new_node->set_timestamp(cartographer::common::ToUniversal(cartographer_ros::FromRos(gnss_msg->header.stamp)));

		  //proto_rigid3ds.push_back(cartographer::transform::ToProto(cartographer::transform::Rigid3d(sensor_pose_rigid3d.inverse() * cartographer::transform::Rigid3d::Vector(gnss_msg->longitude, gnss_msg->latitude, gnss_msg->altitude), cartographer::transform::Rigid3d::Quaternion(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z)) * sensor_pose_rigid3d ));
		  proto_rigid3ds.push_back(cartographer::transform::ToProto(cartographer::transform::Rigid3d(cartographer::transform::Rigid3d::Vector(), cartographer::transform::RollPitchYaw(M_PI,0,0)) * cartographer::transform::Rigid3d(cartographer::transform::Rigid3d::Vector(gnss_msg->longitude, gnss_msg->latitude, gnss_msg->altitude), cartographer::transform::Rigid3d::Quaternion(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z)) * sensor_pose_rigid3d ));
		  cartographer::transform::proto::Rigid3d *t_proto_rigid3d = new cartographer::transform::proto::Rigid3d(proto_rigid3ds.back());
		  new_node->set_allocated_pose(t_proto_rigid3d);
		  ROS_INFO("odometry node size: %d %.6f", g_traj.node_size(), gnss_msg->header.stamp.toSec());
		  
		  imu_sampled = false;
		  gnss_sampled = false;
	      }
	  }
//#endif
      }
  }
  else if (FLAGS_process_raw)
  {
      bool imu_initialized = false;
      bool encoder_harvest = false;
      bool encoder_right_sampled = false;
      bool encoder_left_sampled = false;
      double last_imu_angular_z_vel;
      int encoder_left_data, encoder_right_data;
      ::ros::Time last_stamp_encoder_left, last_stamp_encoder_right;

      int delta_l_enc = 0;
      int delta_r_enc = 0;
      int l_enc_prev = 0;
      int r_enc_prev = 0;
      bool first_time = true;
      double pose_Yaw = 0;
      double pose_X = 0;
      double pose_Y = 0;

      OdomCalculator odom_calc;
      odom_calc.setEncoderOnly(true);

      for (const rosbag::MessageInstance& message : view) {
	  StringCode topic_key = s_mapStringToStringCode[message.getTopic()];

	  sensor_msgs::Imu::Ptr imu_msg;
	  std_msgs::Int32::Ptr encoder_left_msg;
	  std_msgs::Int32::Ptr encoder_right_msg;

	  switch (topic_key)
	  {
	      case eImu:
		  imu_msg = message.instantiate<sensor_msgs::Imu>();
		  last_imu_angular_z_vel = imu_msg->angular_velocity.z;
		  imu_initialized = true;
		  //ROS_INFO("imu: %f",  imu_msg->header.stamp.toSec());
		  break;
	      case eEncoderLeft:
		  encoder_left_msg = message.instantiate<std_msgs::Int32>();
		  encoder_left_data = encoder_left_msg->data;
		  last_stamp_encoder_left = message.getTime();
		  encoder_left_sampled = true;
		  //ROS_INFO("left enc: %d %f", encoder_left_msg->data, message.getTime().toSec());
		break;
	      case eEncoderRight:
		  encoder_right_msg = message.instantiate<std_msgs::Int32>();
		  encoder_right_data = encoder_right_msg->data;
		  last_stamp_encoder_right = message.getTime();
		  encoder_right_sampled = true;
		  //ROS_INFO("right enc: %d %f", encoder_right_msg->data, message.getTime().toSec());
		  break;
	      default:
		  break;
	  }
	  //we do simple check, if both encoders are inited, we check time diff
	  // if time diff too big, we drop the older encoder data
	  if (encoder_right_sampled && encoder_left_sampled)
	  {
	      if (false == encoder_harvest)
	      {
		  if (last_stamp_encoder_left.toSec() - last_stamp_encoder_right.toSec() >= 0.005) //assuming 100 Hz
		  {
			encoder_right_sampled = false;
		  }
		  else if (last_stamp_encoder_right.toSec() - last_stamp_encoder_left.toSec() >= 0.005) //assuming 100 Hz
		  {
			encoder_left_sampled = false;
		  }
		  else 
			encoder_harvest = true;
	      }

	      if (encoder_harvest)
	      {
		  if (odom_calc.Process(last_imu_angular_z_vel, encoder_left_data, encoder_right_data, last_stamp_encoder_left))
		  {
		      auto new_node = g_traj.add_node();
		      new_node->set_timestamp(cartographer::common::ToUniversal(cartographer_ros::FromRos(last_stamp_encoder_left)));

		      auto t_odom_pose = sensor_pose_rigid3d.inverse() * odom_calc.GetRigid3d() * sensor_pose_rigid3d; 
		      proto_rigid3ds.push_back(cartographer::transform::ToProto(t_odom_pose));
		      cartographer::transform::proto::Rigid3d *t_proto_rigid3d = new cartographer::transform::proto::Rigid3d(proto_rigid3ds.back());
		      new_node->set_allocated_pose(t_proto_rigid3d);

		      LOG(INFO) << t_odom_pose;
		      ROS_INFO("odometry node size: %d %.6f", g_traj.node_size(), last_stamp_encoder_left.toSec());

		  }

		  encoder_right_sampled = false;
		  encoder_left_sampled = false;
	      }
	  }
      }
  }
  else
  {
      for (const rosbag::MessageInstance& message : view) {
	  nav_msgs::Odometry::ConstPtr odom_msg = message.instantiate<nav_msgs::Odometry>();

	  auto new_node = g_traj.add_node();
	  new_node->set_timestamp(cartographer::common::ToUniversal(cartographer_ros::FromRos(odom_msg->header.stamp)));

	  proto_rigid3ds.push_back(cartographer::transform::ToProto(cartographer::transform::Rigid3d(sensor_pose_rigid3d.inverse() * cartographer::transform::Rigid3d::Vector(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z), cartographer::transform::Rigid3d::Quaternion(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)) * sensor_pose_rigid3d ));
	  cartographer::transform::proto::Rigid3d *t_proto_rigid3d = new cartographer::transform::proto::Rigid3d(proto_rigid3ds.back());
	  new_node->set_allocated_pose(t_proto_rigid3d);
	  ROS_INFO("odometry node size: %d %.6f", g_traj.node_size(), odom_msg->header.stamp.toSec());
      }
  }

  if (!FLAGS_odom_stream_filename.empty()){
      cartographer::io::ProtoStreamWriter writer(FLAGS_odom_stream_filename);
      writer.WriteProto(g_traj);
      writer.Close();
      ROS_INFO("trajectory saved as: %s", FLAGS_odom_stream_filename.c_str());
  }
}
