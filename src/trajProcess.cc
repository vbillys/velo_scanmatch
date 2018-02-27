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

class Accumulator
{
  public:
    std::vector<VPointCloud> * GetVectorClouds(){ return &pcl_pointclouds_;};
    std::vector<tf::Pose> * GetOdometryPoses(){ return &odometry_poses_;};
    PointCloud  * GetTotalPointCloud() { return &total_pointcloud_;};
    VPointCloud * GetTotalVPointCloud(){ return &total_vpointcloud_;};
    void AccumulateWithTransform(const tf::Transform & transform);
    void AccumulateIRWithTransform(const tf::Transform & transform);
  private:
    std::vector<tf::Pose> odometry_poses_;
    std::vector<VPointCloud> pcl_pointclouds_;
    PointCloud total_pointcloud_;
    VPointCloud total_vpointcloud_;
    const tf::Transform correction_after_  = tf::Transform( tf::createQuaternionFromRPY(1.570795,0,1.570795));
    const tf::Transform correction_before_ = tf::Transform( tf::createQuaternionFromRPY(1.570795,0,1.570795)).inverse();
};

void Accumulator::AccumulateWithTransform(const tf::Transform & transform)
{
  int index = 0; total_pointcloud_.clear();
  for (auto curr_pc : pcl_pointclouds_)
  {
    PointCloud transformed_pointcloud;
    PointCloud t_pc; pcl::copyPointCloud(curr_pc, t_pc);
    pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, odometry_poses_[index]);
    //pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, tf::Pose(odometry_poses[index]) * tf::Pose(tf::Quaternion(), tf::Vector3()));
    // use below if need to correct...
    //pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, correction_after * odometry_poses[index] * correction_before );
    total_pointcloud_ += transformed_pointcloud;
    index++;
  }

  ROS_INFO("Clouds size: %lu, odometry_poses size: %lu, total_points: %lu", pcl_pointclouds_.size(), odometry_poses_.size(), total_pointcloud_.size());
}

void Accumulator::AccumulateIRWithTransform(const tf::Transform & transform)
{
  int index = 0; total_vpointcloud_.clear();
  for (auto curr_pc : pcl_pointclouds_)
  {
    PointCloud transformed_pointcloud;
    PointCloud t_pc; pcl::copyPointCloud(curr_pc, t_pc);
    pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, odometry_poses_[index]);

    VPointCloud transformed_vpointcloud(curr_pc);
    pcl::copyPointCloud(transformed_pointcloud, transformed_vpointcloud);

    //pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, tf::Pose(odometry_poses[index]) * tf::Pose(tf::Quaternion(), tf::Vector3()));
    // use below if need to correct...
    //pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, correction_after * odometry_poses[index] * correction_before );
    total_vpointcloud_ += transformed_vpointcloud;
    index++;
  }
}

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_traj_filename.empty())
    << "-traj_filename is missing.";
  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";

  ros::init(argc, argv, "trajProcess");
  ros::NodeHandle nh, pnh("~");
  bool b_opt_publish_pc, b_opt_calibrate;
  std::string pc_topic;
  pnh.param("publish_pc", b_opt_publish_pc, false);
  pnh.param<std::string>("pc_topic"  , pc_topic, "/velodyne_right/velodyne_points");
  pnh.param("calibrate", b_opt_calibrate, false);

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
  topics.push_back(pc_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::vector<tf::Pose> odometry_poses;
  std::vector<VPointCloud> pcl_pointclouds;
  Accumulator accumulator;

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
      //pcl_pointclouds.push_back( pcl_vcloud_noNaN);
      accumulator.GetVectorClouds()->push_back( pcl_vcloud_noNaN);

      const cartographer::transform::Rigid3d tracking_to_map =
	transform_interpolation_buffer.Lookup(cartographer_ros::FromRos(pc2_msg->header.stamp));
      // TO DO plug in transform param here...
      //const cartographer::transform::Rigid3d sensor_to_tracking;
      //const cartographer::transform::Rigid3f sensor_to_map =
	//(tracking_to_map * sensor_to_tracking).cast<float>();
      //odometry_poses.push_back(tf::Transform( tf::Quaternion(tracking_to_map.rotation().x(),tracking_to_map.rotation().y(),tracking_to_map.rotation().z(),tracking_to_map.rotation().w()), tf::Vector3(tracking_to_map.translation().x(),tracking_to_map.translation().y(),tracking_to_map.translation().z())));
      accumulator.GetOdometryPoses()->push_back( tf::Transform( tf::Quaternion(tracking_to_map.rotation().x(),tracking_to_map.rotation().y(),tracking_to_map.rotation().z(),tracking_to_map.rotation().w()), tf::Vector3(tracking_to_map.translation().x(),tracking_to_map.translation().y(),tracking_to_map.translation().z())));
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

#if 0
  //const tf::Transform correction_after  = tf::Transform( tf::createQuaternionFromRPY(1.570795,0,1.570795)).inverse();
  const tf::Transform correction_after  = tf::Transform( tf::createQuaternionFromRPY(1.570795,0,1.570795));
  const tf::Transform correction_before = tf::Transform( tf::createQuaternionFromRPY(1.570795,0,1.570795)).inverse();

  int index = 0; PointCloud total_pointcloud;
  for (auto curr_pc : pcl_pointclouds)
  {
     PointCloud transformed_pointcloud;
     PointCloud t_pc; pcl::copyPointCloud(curr_pc, t_pc);
     pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, odometry_poses[index]);
     //pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, tf::Pose(odometry_poses[index]) * tf::Pose(tf::Quaternion(), tf::Vector3()));
     // use below if need to correct...
     //pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, correction_after * odometry_poses[index] * correction_before );
     total_pointcloud += transformed_pointcloud;
     index++;
  }

  ROS_INFO("Clouds size: %lu, odometry_poses size: %lu, total_points: %lu", pcl_pointclouds.size(), odometry_poses.size(), total_pointcloud.size());
#endif

  //total_pointcloud = PointCloud(pcl_pointclouds[0]);
  //pcl::copyPointCloud ( pcl_pointclouds[0], total_pointcloud);

  //total_pointcloud.header.frame_id = "map";
  //total_pointcloud.header.stamp    = ros::Time::now().toNSec();


  if (b_opt_publish_pc)
  {
    //accumulator.AccumulateWithTransform(tf::Transform());
    //accumulator.GetTotalPointCloud()->header.frame_id = "map";
    //accumulator.GetTotalPointCloud()->header.stamp    = ros::Time::now().toNSec();
    //pc_pub.publish(*accumulator.GetTotalPointCloud());
    accumulator.AccumulateIRWithTransform(tf::Transform());
    accumulator.GetTotalVPointCloud()->header.frame_id = "map";
    accumulator.GetTotalVPointCloud()->header.stamp    = ros::Time::now().toNSec();
    pc_pub.publish(*accumulator.GetTotalVPointCloud());
    ros::spin();
  }
  else if (b_opt_calibrate)
  {
  }
}
