/**@file	/home/agoy/Documents/workspace/velo_scanmatch/src/trajProcess.cc
 * @author	agoy
 * @version	704
 * @date
 * 	Created:	21st Feb 2018
 * 	Last Update:	21st Feb 2018
 */

/*===========================================================================*/
/*===============================[ «section» ]===============================*/
/*===========================================================================*/

//#include "0"


/*===========================================================================*/
/*===============================[ «section» ]===============================*/
/*===========================================================================*/


#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "proto_stream.h"
#include "trajectory.pb.h"

#include "time_conversion.h"

#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/PointCloud2.h"

#include "transform.h"
#include "transform_interpolation_buffer.h"

#include "tf/transform_datatypes.h"

#include "point_types.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

DEFINE_string(traj_filename, "",
    "Proto stream file containing the pose graph.");
DEFINE_string(bag_filenames, "",
    "Bags to process, must be in the same order as the trajectories "
    "in 'pose_graph_filename'.");

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_traj_filename.empty())
    << "-traj_filename is missing.";
  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";

  ros::init(argc, argv, "trajProcess");
  ros::NodeHandle nh, pnh("~");
  bool b_opt_publish_pc;
  pnh.param("publish_pc", b_opt_publish_pc, false);

  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>("total_pc", 1, true);

  ROS_INFO("Using traj file: %s", FLAGS_traj_filename.c_str());
  cartographer::io::ProtoStreamReader reader(FLAGS_traj_filename);
  cartographer::mapping::proto::Trajectory proto_traj;
  CHECK(reader.ReadProto(&proto_traj));
  ROS_INFO("nodes contained %d", proto_traj.node_size());

  for (auto i : proto_traj.node())
  {
    ROS_INFO("%.10f", cartographer_ros::ToRos(cartographer::common::FromUniversal(i.timestamp())).toSec());
    //LOG(INFO) << cartographer::transform::ToRigid3(i.pose());
  }

  const cartographer::transform::TransformInterpolationBuffer
    transform_interpolation_buffer(proto_traj);
  rosbag::Bag bag;
  bag.open(FLAGS_bag_filenames, rosbag::bagmode::Read);
  ROS_INFO("Using bag file: %s", FLAGS_bag_filenames.c_str());

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_left/velodyne_points"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::vector<tf::Pose> odometry_poses;
  std::vector<VPointCloud> pcl_pointclouds;

  for (const rosbag::MessageInstance& message : view) {
    sensor_msgs::PointCloud2::ConstPtr pc2_msg = message.instantiate<sensor_msgs::PointCloud2>();
    VPointCloud pcl_cloud;
    pcl::fromROSMsg(*pc2_msg, pcl_cloud);

    // TODO: Put additional spacing condition here...
    if(transform_interpolation_buffer.Has(cartographer_ros::FromRos(pc2_msg->header.stamp)))
    {
      PointCloud pcl_cloud_noNaN;
      std::vector<int> indices;
      PointCloud t_ppcl_cloud;
      pcl::copyPointCloud(pcl_cloud, t_ppcl_cloud);
      pcl::removeNaNFromPointCloud(t_ppcl_cloud, pcl_cloud_noNaN, indices);
      VPointCloud pcl_vcloud_noNaN;
      pcl::copyPointCloud(pcl_cloud, indices, pcl_vcloud_noNaN);
      pcl_pointclouds.push_back( pcl_vcloud_noNaN);

      const cartographer::transform::Rigid3d tracking_to_map =
	transform_interpolation_buffer.Lookup(cartographer_ros::FromRos(pc2_msg->header.stamp));
      // TO DO plug in transform param here...
      //const cartographer::transform::Rigid3d sensor_to_tracking;
      //const cartographer::transform::Rigid3f sensor_to_map =
	//(tracking_to_map * sensor_to_tracking).cast<float>();
      odometry_poses.push_back(tf::Transform( tf::Quaternion(tracking_to_map.rotation().x(),tracking_to_map.rotation().y(),tracking_to_map.rotation().z(),tracking_to_map.rotation().w()), tf::Vector3(tracking_to_map.translation().x(),tracking_to_map.translation().y(),tracking_to_map.translation().z())));
      LOG(INFO) << tracking_to_map;
    }

    // test ring info
    //int no_of_points = 0;
    //for (VPointCloud::iterator p=pcl_cloud.begin(); p!= pcl_cloud.end(); p++)
    //{
       //if (p->ring == 7) no_of_points++;
    //}
    //ROS_INFO("no_of_points: %d", no_of_points);

    ROS_INFO("Time message: %.10f %s points:%lu", pc2_msg->header.stamp.toSec(),
      transform_interpolation_buffer.Has(cartographer_ros::FromRos(pc2_msg->header.stamp))?"has":"not", pcl_cloud.size());
  }
  bag.close();


  int index = 0; PointCloud total_pointcloud;
  for (auto curr_pc : pcl_pointclouds)
  {
     PointCloud transformed_pointcloud;
     PointCloud t_pc; pcl::copyPointCloud(curr_pc, t_pc);
     pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, tf::Pose(odometry_poses[index]) * tf::Pose(tf::Quaternion(), tf::Vector3()));
     total_pointcloud += transformed_pointcloud;
     index++;
  }

  ROS_INFO("Clouds size: %lu, odometry_poses size: %lu, total_points: %lu", pcl_pointclouds.size(), odometry_poses.size(), total_pointcloud.size());

  //total_pointcloud = PointCloud(pcl_pointclouds[0]);
  //pcl::copyPointCloud ( pcl_pointclouds[0], total_pointcloud);
  total_pointcloud.header.frame_id = "map";
  total_pointcloud.header.stamp    = ros::Time::now().toNSec();


  if (b_opt_publish_pc)
    pc_pub.publish(total_pointcloud);
  ros::spin();
}
