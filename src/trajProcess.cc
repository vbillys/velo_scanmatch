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

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>

typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

DEFINE_string(traj_filename, "",
    "Proto stream file containing the pose graph.");
DEFINE_string(bag_filenames, "",
    "Bags to process, must be in the same order as the trajectories "
    "in 'pose_graph_filename'.");

class Accumulator
{
  public:
    Accumulator(){
      tree_ = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ> ());
      total_vpointcloud_ = VPointCloud::Ptr(new VPointCloud());
      total_pointcloud_  =  PointCloud::Ptr(new  PointCloud());
    }
    std::vector<VPointCloud> * GetVectorClouds(){ return &pcl_pointclouds_;};
    std::vector<tf::Pose> * GetOdometryPoses(){ return &odometry_poses_;};
    PointCloud::Ptr   GetTotalPointCloud() { return total_pointcloud_;};
    VPointCloud::Ptr  GetTotalVPointCloud(){ return total_vpointcloud_;};
    void AccumulateWithTransform(const tf::Transform & transform);
    void AccumulateIRWithTransform(const tf::Transform & transform);
    float J_calc_wTf(const tf::Transform & transform);
    float J_calc_wRPY_degree(const double & roll, const double & pitch, const double & yaw);
    float J_calc_wtrans_m ( const double & x, const double & y, const double & z);
    float J_calc_wEuler( const double & x, const double & y, const double & z, const double & roll, const double & pitch, const double & yaw);
  private:
    std::vector<tf::Pose> odometry_poses_;
    std::vector<VPointCloud> pcl_pointclouds_;
    PointCloud::Ptr  total_pointcloud_;
    VPointCloud::Ptr total_vpointcloud_;
    const tf::Transform correction_after_  = tf::Transform( tf::createQuaternionFromRPY(1.570795,0,1.570795));
    const tf::Transform correction_before_ = tf::Transform( tf::createQuaternionFromRPY(1.570795,0,1.570795)).inverse();

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_;

};

class JExecutor
{
  public:
    JExecutor(Accumulator* accumulator) : accumulator_(accumulator), filewriter_(std::ofstream("calibration_J_log.txt", std::ios::out | std::ios::binary))
    {};
    float J_calc_wEuler_andLog ( const double & x, const double & y, const double & z, const double & roll, const double & pitch, const double & yaw);
  private:
    Accumulator * accumulator_;
    std::ofstream filewriter_;
};

float JExecutor::J_calc_wEuler_andLog ( const double & x, const double & y, const double & z, const double & roll, const double & pitch, const double & yaw)
{
  float J = accumulator_->J_calc_wTf(tf::Transform(tf::createQuaternionFromRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180), tf::Vector3(x,y,z)));
  //if (!filewriter_) filewriter_ = new std::ofstream("calibration_J_log.txt", std::ios::out | std::ios::binary);
  if (filewriter_.bad()) LOG(WARNING) << "something wrong with io!! canceling write...";
  else
  {
    int n=0; char buffer[200];
    n += sprintf(buffer, "%07.5f %07.5f %07.5f %07.5f %07.5f %07.5f %07.5f\n", x, y, z, roll, pitch, yaw, J);
    filewriter_.write(buffer, n);
    filewriter_.flush();
    ROS_INFO("logged: %s %d", buffer, n);
  }
  return J;
}

float Accumulator::J_calc_wEuler( const double & x, const double & y, const double & z, const double & roll, const double & pitch, const double & yaw)
{
  return J_calc_wTf(tf::Transform(tf::createQuaternionFromRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180), tf::Vector3(x,y,z)));
}

float Accumulator::J_calc_wRPY_degree ( const double & roll, const double & pitch, const double & yaw)
{
  return J_calc_wTf(tf::Transform(tf::createQuaternionFromRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180)));
}

float Accumulator::J_calc_wtrans_m ( const double & x, const double & y, const double & z)
{
  return J_calc_wTf(tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(x,y,z)));
}

float Accumulator::J_calc_wTf(const tf::Transform & transform)
{
  constexpr int neighbooring_for_normal_est = 20; //100; //40;
  constexpr int neighbooring_for_min_density = 10; //2; //20;
  constexpr int beam_no_max_diff = 4; //5; //3; //8; //3;
  constexpr float max_distance_between_two_points = 0.36; //0.2; //0.8; //0.2;
  constexpr int skip_points = 16; //8; //16;
  //constexpr int max_relations_per_i = 3;
  //std::cout<<"normal calcualtion begin"<<std::endl;
  //ne_.setRadiusSearch (0.1);
  //ne_.compute (*cloud_normals);
  //std::cout<<"normal calcualtion end"<<std::endl;

  double begin_time = ros::Time::now().toSec();

  AccumulateIRWithTransform(transform);
  //VPointCloud::Ptr total_vpointcloud_ptr(&total_vpointcloud_);
  //PointCloud::Ptr total_pointcloud_ptr(&total_pointcloud_);
  ne_.setInputCloud (total_pointcloud_);
  ne_.setSearchMethod (tree_);
  ne_.setKSearch (20);
  kdtree.setInputCloud(total_pointcloud_);
  std::vector<float> Js (total_pointcloud_->size());
  std::vector<std::vector<int>>  takens (total_pointcloud_->size());
  

  omp_lock_t writelock;
  omp_init_lock(&writelock);

#pragma omp parallel for num_threads(3)
  //for(int i=0;i<total_pointcloud_->points.size() && ros::ok();i+=16)
  for(int i=0;i<total_pointcloud_->points.size();i+=skip_points)
  {   
    Js[i] = 0;
    //LOG_EVERY_N(INFO, 1000) << "points processed: " << i;
    //LOG_EVERY_N(INFO, 10) << "points processed: " << i;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::vector<float> normal(3); float curv;
    std::vector<int> indices;
    std::vector<float> dists;
    kdtree.nearestKSearch(total_pointcloud_->points[i], neighbooring_for_normal_est , indices, dists);
    ne_.computePointNormal(*total_pointcloud_, indices, normal[0], normal[1], normal[2], curv);
    if (pcl_isfinite(normal[0]) && pcl_isfinite(normal[1]) && pcl_isfinite(normal[2]))
    {}else continue;
    // check if normal is valid first before anything else

    int number_neighbor=kdtree.radiusSearch(total_pointcloud_->points[i], max_distance_between_two_points ,pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if (number_neighbor >= neighbooring_for_min_density )
    {
      int current_beam =total_vpointcloud_->points[i].ring;
      //int neighbor_beam=total_vpointcloud_->points[ pointIdxRadiusSearch[j] ].ring;
      // we form subclouds for choosing nearest points seen by other points' beam
      for (int beam_j = std::max(0,current_beam-beam_no_max_diff); beam_j<=std::min(15,current_beam+beam_no_max_diff); beam_j++)
      {

	    //for (int j=1;j<number_neighbor;++j)
	    //int j_count = 0;
	    // WARNING: -1 means no this beam closest view...
	    int nearest_beam_index= -1;
	    // WARNING assuming big distanc will never reach...
	    float closest_dist_squared = 1e11;
	    std::vector<float> point(3), nearest_vec(3);
	    for (int j=number_neighbor-1;j>=1;--j)
	    {
	      int neighbor_beam=total_vpointcloud_->points[ pointIdxRadiusSearch[j] ].ring;

	      if (beam_j == neighbor_beam )
	      {
		int neighbor_idx = pointIdxRadiusSearch[j];
		point[0]=total_pointcloud_->points[i].x-total_pointcloud_->points[neighbor_idx].x;
		point[1]=total_pointcloud_->points[i].y-total_pointcloud_->points[neighbor_idx].y;
		point[2]=total_pointcloud_->points[i].z-total_pointcloud_->points[neighbor_idx].z;
		float t_dist_squared = point[0] * point[0] + point[1] * point[1] + point[2] * point[2];

		if (closest_dist_squared > t_dist_squared )
		{
		  closest_dist_squared = t_dist_squared;
		  nearest_beam_index = pointIdxRadiusSearch[j];
		  nearest_vec[0] = point[0];nearest_vec[1] = point[1];nearest_vec[2] = point[2];
		}
	      }
	    }
	    //if (j_count >= max_relations_per_i) break;

	    //int current_beam =total_vpointcloud_->points[i].ring;
	    //int neighbor_beam=total_vpointcloud_->points[ pointIdxRadiusSearch[j] ].ring;
	    //if(abs(current_beam-neighbor_beam)<beam_no_max_diff )
	    //{
	    //int neighbor_idx = pointIdxRadiusSearch[j];

	    // check if this relation has been computed before
	    // if not we do computation and set it to computed
	    //omp_set_lock(&writelock);
	    //bool computed_before = false;
	    //for ( auto& t : takens [neighbor_idx ]) {if (t==i) {computed_before = true; break;}}
	    //if (false == computed_before) takens[neighbor_idx ].push_back(i);
	    //omp_unset_lock(&writelock);
	    //if (computed_before) continue;

	    //std::vector<float> point(3);
	    //point[0]=total_pointcloud_->points[i].x-total_pointcloud_->points[neighbor_idx].x;
	    //point[1]=total_pointcloud_->points[i].y-total_pointcloud_->points[neighbor_idx].y;
	    //point[2]=total_pointcloud_->points[i].z-total_pointcloud_->points[neighbor_idx].z;

	    if (nearest_beam_index != -1)
	    {
	      //std::vector<float> normal(3); float curv;
	      ////float nx, ny, nz;
	      //std::vector<int> indices;
	      //std::vector<float> dists;
	      //int number_neighbor=kdtree.nearestKSearch(total_pointcloud_->points[i], neighbooring_for_normal_est , indices, dists);
	      ////ne_.computePointNormal(total_pointcloud_, pointIdxRadiusSearch, normal[0], normal[1], normal[2], curv);
	      //ne_.computePointNormal(*total_pointcloud_, indices, normal[0], normal[1], normal[2], curv);
	      //normal[0]=cloud_normals->points[i].normal_x;
	      //normal[1]=cloud_normals->points[i].normal_y;
	      //normal[2]=cloud_normals->points[i].normal_z;
	      //LOG(INFO) << point[0] << " " << normal[0] << " " << point[1] << " " << normal[1] << " " << point[2] << " " << normal[2];
	      //
	      //if (pcl_isfinite(normal[0]) && pcl_isfinite(normal[1]) && pcl_isfinite(normal[2]))
	      //{
		//float mag = point[0] * normal[0] + point[1] * normal[1] + point[2] * normal[2];
		float mag = nearest_vec[0] * normal[0] + nearest_vec[1] * normal[1] + nearest_vec[2] * normal[2];
		Js[i] += mag * mag;
		//j_count++;
	      //}
	    
	    //else 
	      //continue;
	    //LOG(INFO) << mag;
	    //break;
	    }
        //}
      }
    }
  }

    
  omp_destroy_lock(&writelock);

  float J = 0;
  for (auto& tJ : Js)
        J += tJ;

  LOG(INFO) << "Elapse in this iter: " << ros::Time::now().toSec() - begin_time << " cost: " << J;

  return J;
}

void Accumulator::AccumulateWithTransform(const tf::Transform & transform)
{
  int index = 0; total_pointcloud_->clear();
  for (auto curr_pc : pcl_pointclouds_)
  {
    PointCloud transformed_pointcloud;
    PointCloud t_pc; pcl::copyPointCloud(curr_pc, t_pc);
    pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, odometry_poses_[index] * transform);
    //pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, tf::Pose(odometry_poses[index]) * tf::Pose(tf::Quaternion(), tf::Vector3()));
    // use below if need to correct...
    //pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, correction_after * odometry_poses[index] * correction_before );
    *total_pointcloud_ += transformed_pointcloud;
    index++;
  }

  ROS_INFO("Clouds size: %lu, odometry_poses size: %lu, total_points: %lu", pcl_pointclouds_.size(), odometry_poses_.size(), total_pointcloud_->size());
}

void Accumulator::AccumulateIRWithTransform(const tf::Transform & transform)
{
  int index = 0; total_vpointcloud_->clear(); total_pointcloud_->clear();
  for (auto curr_pc : pcl_pointclouds_)
  {
    PointCloud transformed_pointcloud;
    PointCloud t_pc; pcl::copyPointCloud(curr_pc, t_pc);
    pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, odometry_poses_[index] * transform);

    VPointCloud transformed_vpointcloud(curr_pc);
    pcl::copyPointCloud(transformed_pointcloud, transformed_vpointcloud);

    //pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, tf::Pose(odometry_poses[index]) * tf::Pose(tf::Quaternion(), tf::Vector3()));
    // use below if need to correct...
    //pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, correction_after * odometry_poses[index] * correction_before );
    *total_vpointcloud_ += transformed_vpointcloud;
    *total_pointcloud_  += transformed_pointcloud;
    index++;
  }
  ROS_INFO("Clouds size: %lu, odometry_poses size: %lu, total_points: %lu", pcl_pointclouds_.size(), odometry_poses_.size(), total_pointcloud_->size());
}

//void Accumulator::CopyVCloudToCloud()
//{
  //total_pointcloud_.clear();
  //pcl::copyPointCloud(total_vpointcloud_, total_pointcloud_);
//}

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
  //pcl::VoxelGrid<VPointCloud> sor;

  for (const rosbag::MessageInstance& message : view) {
    sensor_msgs::PointCloud2::ConstPtr pc2_msg = message.instantiate<sensor_msgs::PointCloud2>();
    VPointCloud pcl_cloud;
    pcl::fromROSMsg(*pc2_msg, pcl_cloud);

    // TODO: Put additional spacing condition here...
    if(transform_interpolation_buffer.Has(cartographer_ros::FromRos(pc2_msg->header.stamp)))
    {
      PointCloud pcl_cloud_noNaN;
      std::vector<int> indices;
      PointCloud::Ptr t_ppcl_cloud(new PointCloud());
      pcl::copyPointCloud(pcl_cloud, *t_ppcl_cloud);
      pcl::removeNaNFromPointCloud(*t_ppcl_cloud, pcl_cloud_noNaN, indices);
      VPointCloud pcl_vcloud_noNaN;
      pcl::copyPointCloud(pcl_cloud, indices, pcl_vcloud_noNaN);
      //pcl_pointclouds.push_back( pcl_vcloud_noNaN);

      // passthorugh
      t_ppcl_cloud->clear();
      pcl::copyPointCloud(pcl_vcloud_noNaN, *t_ppcl_cloud);
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (t_ppcl_cloud);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (0.6, 10000.0);
      indices.clear();
      pass.filter (indices);
      VPointCloud pcl_vcloud_noNaN_passthrough;
      pcl::copyPointCloud(pcl_vcloud_noNaN, indices, pcl_vcloud_noNaN_passthrough);

      //accumulator.GetVectorClouds()->push_back( pcl_vcloud_noNaN);
      accumulator.GetVectorClouds()->push_back( pcl_vcloud_noNaN_passthrough);

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
    //accumulator.AccumulateIRWithTransform(tf::transform());
    accumulator.AccumulateIRWithTransform(tf::Transform(tf::createQuaternionFromRPY(0, 5*M_PI/180, 0)));
    accumulator.GetTotalVPointCloud()->header.frame_id = "map";
    //accumulator.GetTotalVPointCloud()->header.stamp    = ros::Time::now().toNSec();
    pc_pub.publish(*accumulator.GetTotalVPointCloud());
    ros::spin();
  }
  else if (b_opt_calibrate)
  {
    omp_set_dynamic(0);
    JExecutor jexecutor(&accumulator);
    double begin_time = ros::Time::now().toSec();
    LOG(INFO) << "Begin at:" << begin_time;
    for (int k=0; k<20 && ros::ok() ; k++)
      //jexecutor.J_calc_wEuler_andLog(0,-1.98,0,0, k-10, 0);
      //jexecutor.J_calc_wEuler_andLog(0,0,0,0, static_cast<double>(k*0.25-2.5), 0);
      jexecutor.J_calc_wEuler_andLog(0,-1.98,0,0, static_cast<double>(7.5+k*0.25), 0);
      //jexecutor.J_calc_wEuler_andLog(0,-1.98,0,0, static_cast<double>(9.5+k*0.25), 0);
    double end_time = ros::Time::now().toSec();
    LOG(INFO) << "End at:" << end_time;
    LOG(INFO) << "Time elapsed: " << end_time - begin_time;
  }
}
