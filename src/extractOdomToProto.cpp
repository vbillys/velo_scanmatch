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

DEFINE_string(bag_filenames, "",
    "Bags to process, must be in the same order as the trajectories "
    "in 'pose_graph_filename'.");

DEFINE_string(odom_stream_filename, "",
    "File to output to contain the timestamped odometry data.");

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";

  ros::init(argc, argv, "extractOdom");
  ros::NodeHandle nh, pnh("~");

  std::string pc_topic;
  pnh.param<std::string>("odom_topic"  , pc_topic, "/imuodom");

  rosbag::Bag bag;
  bag.open(FLAGS_bag_filenames, rosbag::bagmode::Read);
  ROS_INFO("Using bag file: %s", FLAGS_bag_filenames.c_str());

  std::vector<std::string> topics;
  topics.push_back(pc_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  cartographer::mapping::proto::Trajectory g_traj;
  std::vector<cartographer::transform::proto::Rigid3d> proto_rigid3ds;

  for (const rosbag::MessageInstance& message : view) {
    nav_msgs::Odometry::ConstPtr odom_msg = message.instantiate<nav_msgs::Odometry>();

    auto new_node = g_traj.add_node();
    new_node->set_timestamp(cartographer::common::ToUniversal(cartographer_ros::FromRos(odom_msg->header.stamp)));

    proto_rigid3ds.push_back(cartographer::transform::ToProto(cartographer::transform::Rigid3d(cartographer::transform::Rigid3d::Vector(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z), cartographer::transform::Rigid3d::Quaternion(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z))));
    cartographer::transform::proto::Rigid3d *t_proto_rigid3d = new cartographer::transform::proto::Rigid3d(proto_rigid3ds.back());
    new_node->set_allocated_pose(t_proto_rigid3d);
    ROS_INFO("odometry node size: %d %.6f", g_traj.node_size(), odom_msg->header.stamp.toSec());
  }

  if (!FLAGS_odom_stream_filename.empty()){
      cartographer::io::ProtoStreamWriter writer(FLAGS_odom_stream_filename);
      writer.WriteProto(g_traj);
      writer.Close();
      ROS_INFO("trajectory saved as: %s", FLAGS_odom_stream_filename.c_str());
  }
}
