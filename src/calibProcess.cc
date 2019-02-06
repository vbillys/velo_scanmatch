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
#include <pcl/search/flann_search.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <ecl/threads.hpp>

typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::WallTime start_, end_;

// ros::NodeHandle pnh("~");

//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//pcl::KdTreeFLANN<pcl::PointXYZ> ref_kdtree;
//pcl::search::FlannSearch<pcl::PointXYZ> kdtree;
//pcl::search::FlannSearch<pcl::PointXYZ> ref_kdtree;

// pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> kdtree (0.5);
// pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> ref_kdtree (0.5);

using ecl::Mutex;


DEFINE_string(traj_filename, "",
    "Proto stream file containing the pose graph.");
DEFINE_string(bag_filenames, "",
    "Bags to process, must be in the same order as the trajectories "
    "in 'pose_graph_filename'.");

class Accumulator
{
  public:
    Accumulator() : use_ref_(false){
      tree_ = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ> ());
      total_vpointcloud_ = VPointCloud::Ptr(new VPointCloud());
      total_pointcloud_  =  PointCloud::Ptr(new  PointCloud());
      ref_total_vpointcloud_ = VPointCloud::Ptr(new VPointCloud());
      ref_total_pointcloud_  =  PointCloud::Ptr(new  PointCloud());

    }
    std::vector<VPointCloud> * GetVectorClouds(){ return &pcl_pointclouds_;};
    std::vector<tf::Pose> * GetOdometryPoses(){ return &odometry_poses_;};
    PointCloud::Ptr   GetTotalPointCloud() { return total_pointcloud_;};
    VPointCloud::Ptr  GetTotalVPointCloud(){ return total_vpointcloud_;};

    std::vector<VPointCloud> * GetRefVectorClouds(){ return &ref_pcl_pointclouds_;};
    std::vector<tf::Pose> * GetRefOdometryPoses(){ return &ref_odometry_poses_;};
    PointCloud::Ptr   GetRefTotalPointCloud() { return ref_total_pointcloud_;};
    VPointCloud::Ptr  GetRefTotalVPointCloud(){ return ref_total_vpointcloud_;};

    void AccumulateWithTransform(const tf::Transform & transform);
    void AccumulateIRWithTransform(const tf::Transform & transform);
    float J_calc_wTf(const tf::Transform & transform);
    float J_calc_wRPY_degree(const double & roll, const double & pitch, const double & yaw);
    float J_calc_wtrans_m ( const double & x, const double & y, const double & z);
    float J_calc_wEuler( const double & x, const double & y, const double & z, const double & roll, const double & pitch, const double & yaw);

    void setUseRef(const bool & use_ref){use_ref_ = use_ref;};
    void InsertScanWithTransform(VPointCloud::Ptr vpointcloud_ptr, const tf::Transform & transform);
    void AccumulateScansTransform(const tf::Transform & transform);
    float CalcJForRingSepScans();

  private:
    const int neighbooring_for_normal_est = 20;

  private:
    static std::vector<VPointCloud::Ptr> RingSepScan(VPointCloud::Ptr vpointcloud_ptr);
    static VPointCloud::Ptr CleanVPointCloud(VPointCloud::Ptr vpointcloud_ptr);
    void CalcKdtreeForAccumRingScans();
    bool FindNearestPointInBeamScans(int i, const pcl::PointXYZ & point_in, pcl::PointXYZ & point_out, std::vector<int> & indices);
    float ComputeProjCost(int beam, const pcl::PointXYZ & pk, const pcl::PointXYZ & nearest_p, const std::vector<int> & indices);

  private:
    std::vector<tf::Pose> odometry_poses_;
    std::vector<VPointCloud> pcl_pointclouds_;
    PointCloud::Ptr  total_pointcloud_;
    VPointCloud::Ptr total_vpointcloud_;

    std::vector<std::vector<VPointCloud::Ptr>> ring_sep_scans_;
    // std::vector<VPointCloud::Ptr> accum_ring_rep_irscans_;
    std::vector<PointCloud::Ptr> accum_ring_rep_scans_;
    // std::vector<pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>::Ptr> accum_ring_rep_scans_ne_;
    std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr> accum_ring_rep_scans_kdtree_;

    std::vector<tf::Pose> ref_odometry_poses_;
    std::vector<VPointCloud> ref_pcl_pointclouds_;
    PointCloud::Ptr  ref_total_pointcloud_;
    VPointCloud::Ptr ref_total_vpointcloud_;

    const tf::Transform correction_after_  = tf::Transform( tf::createQuaternionFromRPY(1.570795,0,1.570795));
    const tf::Transform correction_before_ = tf::Transform( tf::createQuaternionFromRPY(1.570795,0,1.570795)).inverse();

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
    //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_;

    bool use_ref_;
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

  accumulator_->AccumulateScansTransform(tf::Transform(tf::createQuaternionFromRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180), tf::Vector3(x,y,z)));

  float J = accumulator_->CalcJForRingSepScans();

  // float J = accumulator_->J_calc_wTf(tf::Transform(tf::createQuaternionFromRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180), tf::Vector3(x,y,z)));
  //if (!filewriter_) filewriter_ = new std::ofstream("calibration_J_log.txt", std::ios::out | std::ios::binary);
  if (filewriter_.bad()) LOG(WARNING) << "something wrong with io!! canceling write...";
  else
  {
    int n=0; char buffer[200];
    n += sprintf(buffer, "%07.5f %07.5f %07.5f %07.5f %07.5f %07.5f %07.5f\n", x, y, z, roll, pitch, yaw, J);
    filewriter_.write(buffer, n);
    filewriter_.flush();
    //ROS_INFO("logged: %s %d", buffer, n);
  }
  return J;
}

void Accumulator::InsertScanWithTransform(VPointCloud::Ptr vpointcloud_ptr, const tf::Transform & transform) {
    odometry_poses_.push_back(transform);
    ring_sep_scans_.push_back(RingSepScan(CleanVPointCloud(vpointcloud_ptr)));
}

VPointCloud::Ptr Accumulator::CleanVPointCloud(VPointCloud::Ptr vpointcloud_ptr){
  PointCloud pcl_cloud_noNaN;
  std::vector<int> indices;
  PointCloud::Ptr t_ppcl_cloud(new PointCloud());
  pcl::copyPointCloud(*vpointcloud_ptr, *t_ppcl_cloud);
  pcl::removeNaNFromPointCloud(*t_ppcl_cloud, pcl_cloud_noNaN, indices);
  VPointCloud pcl_vcloud_noNaN;
  pcl::copyPointCloud(*vpointcloud_ptr, indices, pcl_vcloud_noNaN);
  // pcl_pointclouds.push_back( pcl_vcloud_noNaN);

  // passthorugh
  t_ppcl_cloud->clear();
  pcl::copyPointCloud(pcl_vcloud_noNaN, *t_ppcl_cloud);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(t_ppcl_cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.6, 10000.0);
  indices.clear();
  pass.filter(indices);
  VPointCloud::Ptr pcl_vcloud_noNaN_passthrough(new VPointCloud());
  pcl::copyPointCloud(pcl_vcloud_noNaN, indices, *pcl_vcloud_noNaN_passthrough);
  return pcl_vcloud_noNaN_passthrough;
}

std::vector<VPointCloud::Ptr> Accumulator::RingSepScan(VPointCloud::Ptr vpointcloud_ptr) {
    std::vector<VPointCloud::Ptr> result;
    // allocate 16 rings
    for (int i=0; i<16; i++) {
        result.push_back(VPointCloud::Ptr(new VPointCloud()));
    }
    for(const auto& p:*vpointcloud_ptr) {
        result.at(p.ring)->push_back(p);
    }
    return result;
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

void Accumulator::CalcKdtreeForAccumRingScans() {
    for (int i=0; i<16; i++) {
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr _kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        _kdtree->setInputCloud(accum_ring_rep_scans_.at(i)); 
        accum_ring_rep_scans_kdtree_.push_back(_kdtree);
    }
}

bool Accumulator::FindNearestPointInBeamScans(int i, const pcl::PointXYZ & point_in, pcl::PointXYZ & point_out, std::vector<int> & indices) {
    constexpr float max_distance_between_two_points = 0.2; //0.5; //0.2; //0.36; //0.2; //0.8; //0.2;
    
    // std::vector<int> indices(neighbooring_for_normal_est);
    std::vector<float> dists(neighbooring_for_normal_est);
    accum_ring_rep_scans_kdtree_.at(i)->nearestKSearch(point_in, neighbooring_for_normal_est , indices, dists);
    if (dists[0] < max_distance_between_two_points) {
        point_out = accum_ring_rep_scans_.at(i)->points[indices[0]];
        return true;
    } else {
        return false;
    }
}

float Accumulator::ComputeProjCost(int beam, const pcl::PointXYZ & pk, const pcl::PointXYZ & nearest_p, const std::vector<int> & indices) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    std::vector<float> normal(3);
    float curv;
    ne.computePointNormal(*(accum_ring_rep_scans_.at(beam)), indices, normal[0], normal[1], normal[2], curv);
    if (pcl_isfinite(normal[0]) && pcl_isfinite(normal[1]) && pcl_isfinite(normal[2])) {
      float mag = (pk.x - nearest_p.x) * normal[0] +
                  (pk.y - nearest_p.y) * normal[1] +
                  (pk.z - nearest_p.z) * normal[2];
      return mag * mag;
    } else {
      return 0;
    }
}

float Accumulator::CalcJForRingSepScans() {

    // init kdtrees first
    CalcKdtreeForAccumRingScans();

    float result = 0;
    constexpr int N = 2;
    for (int bi = 0; bi < 16; bi++) {
        for (int bj = bi-N; bj < bi+N; bj++) {
            int _bj = bj < 0 ? 0 : bj;
            _bj = _bj > 15 ? 15 : _bj;
            for (const auto& pk : *(accum_ring_rep_scans_.at(_bj))) {
                pcl::PointXYZ nearest_p;
                std::vector<int> indices;
                if (FindNearestPointInBeamScans(bi, pk, nearest_p, indices)) {
                    result += ComputeProjCost(bi, pk, nearest_p, indices);
                }
            }
        }
    }
    return result;
}

float Accumulator::J_calc_wTf(const tf::Transform & transform)
{
  constexpr int neighbooring_for_normal_est = 20; //100; //40;
  constexpr int neighbooring_for_min_density = 10; //2; //20;
  constexpr int beam_no_max_diff = 4; //5; //3; //8; //3;
  constexpr float max_distance_between_two_points = 0.2; //0.5; //0.2; //0.36; //0.2; //0.8; //0.2;
  // int skip_points;
  // pnh.param("skip_points",skip_points,16);
  constexpr int skip_points = 16; //512; //4; //16; //32; //16; //8; //16;
  constexpr int max_relations_per_i = 1;
  constexpr float max_dist_squared = max_distance_between_two_points * max_distance_between_two_points;
  //std::cout<<"normal calcualtion begin"<<std::endl;
  //ne_.setRadiusSearch (0.1);
  //ne_.compute (*cloud_normals);
  //std::cout<<"normal calcualtion end"<<std::endl;

  Mutex mutex;

  double begin_time = ros::Time::now().toSec();

  AccumulateIRWithTransform(transform);
  //VPointCloud::Ptr total_vpointcloud_ptr(&total_vpointcloud_);
  //PointCloud::Ptr total_pointcloud_ptr(&total_pointcloud_);
  //ne_.setInputCloud (total_pointcloud_);
  //ne_.setSearchMethod (tree_);
  //ne_.setKSearch (20);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::KdTreeFLANN<pcl::PointXYZ> ref_kdtree;

  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> kdtree (0.25);
  // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> ref_kdtree (0.25);

  // kdtree.setInputCloud(total_pointcloud_); kdtree.deleteTree(); kdtree.addPointsFromInputCloud();
  //if (use_ref_) { ref_kdtree.setInputCloud(ref_total_pointcloud_); ref_kdtree.deleteTree(); ref_kdtree.addPointsFromInputCloud(); }
  kdtree.setInputCloud(total_pointcloud_); 
  if (use_ref_) { ref_kdtree.setInputCloud(ref_total_pointcloud_); }

  std::vector<float> Js (total_pointcloud_->size(),0.0);
  std::vector<std::vector<int>>  takens (total_pointcloud_->size());
  // Js=0;

  // omp_lock_t writelock;
  // omp_init_lock(&writelock);

  if (use_ref_) 
  {
      #pragma omp parallel for num_threads(8)
      for(int i=0;i<total_pointcloud_->points.size();i+=skip_points)
      {
        Js[i] = 0;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        std::vector<float> normal(3); float curv;
        std::vector<int> indices(neighbooring_for_normal_est);
        std::vector<float> dists(neighbooring_for_normal_est);
        kdtree.nearestKSearch(total_pointcloud_->points[i], neighbooring_for_normal_est , indices, dists);
        ne_.computePointNormal(*total_pointcloud_, indices, normal[0], normal[1], normal[2], curv);
        if (pcl_isfinite(normal[0]) && pcl_isfinite(normal[1]) && pcl_isfinite(normal[2]))
        {}else continue;
        // check if normal is valid first before anything else

        int number_neighbor=ref_kdtree.radiusSearch(total_pointcloud_->points[i], max_distance_between_two_points ,pointIdxRadiusSearch, pointRadiusSquaredDistance);
        if (number_neighbor >= neighbooring_for_min_density )
        {
          int j_count = 0;
          //for (int j=number_neighbor-1;j>=1;--j)
          for (int j=1;j<number_neighbor;++j)
          {
            if (j_count > max_relations_per_i) break;

            int neighbor_idx = pointIdxRadiusSearch[j];

            std::vector<float> point(3);
            point[0]=total_pointcloud_->points[i].x-ref_total_pointcloud_->points[neighbor_idx].x;
            point[1]=total_pointcloud_->points[i].y-ref_total_pointcloud_->points[neighbor_idx].y;
            point[2]=total_pointcloud_->points[i].z-ref_total_pointcloud_->points[neighbor_idx].z;

            float mag = point[0] * normal[0] + point[1] * normal[1] + point[2] * normal[2];
            //float mag = point[0] * point[0] + point[1] * point[1] + point[2] * point[2];
            Js[i] += mag * mag;

            j_count++;
          }
        }
      }
  }
  else
  {
      ROS_INFO_STREAM("total_pc size:"<<" "<<total_pointcloud_->points.size());
      #pragma omp parallel for num_threads(12)
      //for(int i=0;i<total_pointcloud_->points.size() && ros::ok();i+=16)
      for(int i=0;i<total_pointcloud_->points.size();i+=skip_points)
      {
        // ROS_INFO("inside for loop");
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_;
        Js[i] = 0;
        //LOG_EVERY_N(INFO, 1000) << "points processed: " << i;
        //LOG_EVERY_N(INFO, 10) << "points processed: " << i;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        std::vector<float> normal(3); float curv;
        std::vector<int> indices(neighbooring_for_normal_est);
        std::vector<float> dists(neighbooring_for_normal_est);

        start_ = ros::WallTime::now();
        kdtree.nearestKSearch(total_pointcloud_->points[i], neighbooring_for_normal_est , indices, dists);
        end_ = ros::WallTime::now();
        double execution_time = (end_ - start_).toNSec() * 1e-6;
        // ROS_WARN("Exectution time for KNN search1 (ms):%f ",execution_time);
        
        // ROS_INFO("20 nearest neighbor:%d",indices);
        //mutex.lock();
        start_ = ros::WallTime::now();
        ne_.computePointNormal(*total_pointcloud_, indices, normal[0], normal[1], normal[2], curv);
        end_ = ros::WallTime::now();
        execution_time = (end_ - start_).toNSec() * 1e-6;
        // ROS_WARN("Exectution time for normal calc (ms):%f ",execution_time);
        //mutex.unlock();
        // if(i<20)
        // {
        // ROS_INFO("normal:%f",normal[1]);
        // }
        
        if (pcl_isfinite(normal[0]) && pcl_isfinite(normal[1]) && pcl_isfinite(normal[2]))
        {}else continue;
        // check if normal is valid first before anything else
        start_ = ros::WallTime::now();
        int number_neighbor=kdtree.radiusSearch(total_pointcloud_->points[i], max_distance_between_two_points ,pointIdxRadiusSearch, pointRadiusSquaredDistance);
        end_ = ros::WallTime::now();
        execution_time = (end_ - start_).toNSec() * 1e-6;
        // ROS_WARN("Exectution time for KNN search2 (ms):%f ",execution_time);
        // if(i<20)
        //   {
        //   for(int disp_count=0;disp_count<10;++disp_count)
        //   {
        //     ROS_INFO("radius search result x: %f",total_vpointcloud_->points[pointIdxRadiusSearch[disp_count]].x);
        //     ROS_INFO("radius search result y: %f",total_vpointcloud_->points[pointIdxRadiusSearch[disp_count]].y);
        //     ROS_INFO("radius search result z: %f",total_vpointcloud_->points[pointIdxRadiusSearch[disp_count]].z);
        //   }
        //   }
        

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

              // if (beam_j == neighbor_beam )
              // {
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
              // }
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

            //if (nearest_beam_index != -1 && closest_dist_squared < max_dist_squared)
            if (nearest_beam_index != -1 && closest_dist_squared < max_dist_squared)
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
        // mutex.unlock();
      }
  }


  // omp_destroy_lock(&writelock);

  float J = 0;
  for (auto& tJ : Js)
        J += tJ;

  //LOG(INFO) << "Elapse in this iter: " << ros::Time::now().toSec() - begin_time << " cost: " << J;

  return J;
}

void Accumulator::AccumulateScansTransform(const tf::Transform & transform) {
    // clear/renew first
    accum_ring_rep_scans_.clear();
    for (int i=0; i<16; i++) {
        accum_ring_rep_scans_.push_back(PointCloud::Ptr(new PointCloud()));
    }
    int index = 0;
    for (const auto& ringed_scans : ring_sep_scans_) {
        for (int i=0; i<16; i++) {
          PointCloud t_pc;
          PointCloud transformed_pointcloud;
          pcl::copyPointCloud(*(ringed_scans.at(i)), t_pc);
          pcl_ros::transformPointCloud(t_pc, transformed_pointcloud,
                                       odometry_poses_[index] * transform);
          *(accum_ring_rep_scans_.at(i)) += transformed_pointcloud;
        }
        ++index;
    }
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

  //ROS_INFO("Clouds size: %lu, odometry_poses size: %lu, total_points: %lu", pcl_pointclouds_.size(), odometry_poses_.size(), total_pointcloud_->size());
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
  //ROS_INFO("Clouds size: %lu, odometry_poses size: %lu, total_points: %lu", pcl_pointclouds_.size(), odometry_poses_.size(), total_pointcloud_->size());

  if (use_ref_){
      index = 0; ref_total_vpointcloud_->clear(); ref_total_pointcloud_->clear();
      for (auto curr_pc : ref_pcl_pointclouds_)
      {
	PointCloud transformed_pointcloud;
	PointCloud t_pc; pcl::copyPointCloud(curr_pc, t_pc);
	pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, ref_odometry_poses_[index]);

	VPointCloud transformed_vpointcloud(curr_pc);
	pcl::copyPointCloud(transformed_pointcloud, transformed_vpointcloud);

	//pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, tf::Pose(odometry_poses[index]) * tf::Pose(tf::Quaternion(), tf::Vector3()));
	// use below if need to correct...
	//pcl_ros::transformPointCloud(t_pc, transformed_pointcloud, correction_after * odometry_poses[index] * correction_before );
	*ref_total_vpointcloud_ += transformed_vpointcloud;
	*ref_total_pointcloud_  += transformed_pointcloud;
	index++;
      }
      //ROS_INFO("Clouds size: %lu, odometry_poses size: %lu, total_points: %lu", ref_pcl_pointclouds_.size(), ref_odometry_poses_.size(), ref_total_pointcloud_->size());
  }
}

//void Accumulator::CopyVCloudToCloud()
//{
  //total_pointcloud_.clear();
  //pcl::copyPointCloud(total_vpointcloud_, total_pointcloud_);
//}

int main(int argc, char** argv) 
{
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_traj_filename.empty())
    << "-traj_filename is missing.";
  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";

  ros::init(argc, argv, "trajProcess");
  ros::NodeHandle nh, pnh("~");
  bool b_opt_publish_pc, b_opt_calibrate, b_opt_use_ref;
  std::string pc_topic, ref_topic;
  int mode;
  double init_x, init_y, init_z, init_roll, init_pitch, init_yaw, search_resolution_xyz, half_width_search_xyz,msg_start_num, msg_end_num, search_resolution_rpy, half_width_search_rpy, search_resolution, half_width_search;
  pnh.param("publish_pc", b_opt_publish_pc, false);
  pnh.param("use_ref", b_opt_use_ref, false);
  pnh.param<std::string>("pc_topic"  , pc_topic, "/velodyne_right/velodyne_points");
  pnh.param("calibrate", b_opt_calibrate, false);
  pnh.param<std::string>("ref_topic"  , ref_topic, "/velodyne_left/velodyne_points");
  pnh.param("mode",mode,1);
  pnh.param("init_x",init_x,2.735);
  pnh.param("init_y",init_y,0.0);
  pnh.param("init_z",init_z,-1.376);
  pnh.param("init_roll",init_roll,0.0);
  pnh.param("init_pitch",init_pitch,0.0);
  pnh.param("init_yaw",init_yaw,-180.0);

  pnh.param("search_resolution",search_resolution,0.1);
  pnh.param("half_width_search",half_width_search,3.0);

  pnh.param("search_resolution_xyz",search_resolution_xyz,0.1);
  pnh.param("half_width_search_xyz",half_width_search_xyz,3.0);

  pnh.param("search_resolution_rpy",search_resolution_rpy,0.1);
  pnh.param("half_width_search_rpy",half_width_search_rpy,3.0);



  pnh.param("starting_mssg_number",msg_start_num,200.0);
  pnh.param("ending_mssg_number",msg_end_num,600.0);

  std::ofstream filewriter(std::ofstream("calibration_J_log_results.txt", std::ios::out | std::ios::binary));
  // init_x-=1.415;
  // init_z-=1.698;
  // init_yaw*= -1;

  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>("total_pc", 1, true);
  ros::Publisher ref_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("total_ref_pc", 1, true);

  ROS_INFO("Using traj file: %s", FLAGS_traj_filename.c_str());
  cartographer::io::ProtoStreamReader reader(FLAGS_traj_filename);
  cartographer::mapping::proto::Trajectory proto_traj;
  CHECK(reader.ReadProto(&proto_traj));
  // ROS_INFO("nodes contained %d", proto_traj.node_size());

  for (auto i : proto_traj.node())
  {
    //ROS_INFO("%.10f", cartographer_ros::ToRos(cartographer::common::FromUniversal(i.timestamp())).toSec());
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

  //std::vector<tf::Pose> odometry_poses;
  //std::vector<VPointCloud> pcl_pointclouds;
  Accumulator accumulator;
  accumulator.setUseRef(b_opt_use_ref);
  //pcl::VoxelGrid<VPointCloud> sor;
  double num_points_counter=0;
  int num_scans_in_bag = 0;

  for (const rosbag::MessageInstance &message : view) {
    sensor_msgs::PointCloud2::ConstPtr pc2_msg =
        message.instantiate<sensor_msgs::PointCloud2>();
    if (transform_interpolation_buffer.Has(
            cartographer_ros::FromRos(pc2_msg->header.stamp))) {
      const cartographer::transform::Rigid3d tracking_to_map =
          transform_interpolation_buffer.Lookup(
              cartographer_ros::FromRos(pc2_msg->header.stamp));
      // accumulator.GetOdometryPoses()->push_back(tf::Transform(
      //     tf::Quaternion(
      //         tracking_to_map.rotation().x(), tracking_to_map.rotation().y(),
      //         tracking_to_map.rotation().z(), tracking_to_map.rotation().w()),
      //     tf::Vector3(tracking_to_map.translation().x(),
      //                 tracking_to_map.translation().y(),
      //                 tracking_to_map.translation().z())));
      ++num_points_counter;
      VPointCloud::Ptr pcl_cloud(new VPointCloud());
      pcl::fromROSMsg(*pc2_msg, *pcl_cloud);
      accumulator.InsertScanWithTransform(pcl_cloud, tf::Transform(
          tf::Quaternion(
              tracking_to_map.rotation().x(), tracking_to_map.rotation().y(),
              tracking_to_map.rotation().z(), tracking_to_map.rotation().w()),
          tf::Vector3(tracking_to_map.translation().x(),
                      tracking_to_map.translation().y(),
                      tracking_to_map.translation().z())));
    }
    ++num_scans_in_bag;
  }
  ROS_INFO_STREAM("No. of scans that's interpolated: " << num_points_counter << " out of " << num_scans_in_bag);

  JExecutor jexecutor(&accumulator);

  float min_J_k;
  float min_J_k_x;
  float min_J_k_y;
  float min_J_k_z;
  float min_J_k_roll;
  float min_J_k_pitch;
  float min_J_k_yaw;
  float min_J; //=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z,
               //init_roll, init_pitch, init_yaw);
  float J_k = 0;
  float J_k_x = 0;
  float J_k_y = 0;
  float J_k_z = 0;
  float J_k_roll = 0;
  float J_k_pitch = 0;
  float J_k_yaw = 0;

  float final_val_x;
  float final_val_y;
  float final_val_z;
  float final_val_roll;
  float final_val_pitch;
  float final_val_yaw;
  float final_val;

  {
    double sum = 0;
    int load_count = 0;
    // for(int i=0;i<5;++i)
    // {
    min_J_k = 10000000;
    J_k = 0;
    for (int k = 0;
         k < (((half_width_search * 2) / search_resolution) + 1) && ros::ok();
         k++) {
      load_count += 1;
      J_k = jexecutor.J_calc_wEuler_andLog(
          init_x - half_width_search + k * search_resolution, init_y, init_z,
          init_roll, init_pitch, init_yaw);
      ROS_INFO("Processed:%d/%f", load_count,
               (1) * (((half_width_search * 2) / search_resolution) + 1));
      if (J_k < min_J_k) {
        final_val = init_x - half_width_search + k * search_resolution;
        // ROS_WARN("Optimized
        // value=%f",init_x-half_width_search+k*search_resolution);
        min_J_k = J_k;
      }
    }
    // ROS_WARN("Optimized value x=%f",final_val+1.415);

    //   sum+=final_val;
    // }
    ROS_WARN("Optimized value x=%f", final_val);
  }
  exit(-1);

  for (const rosbag::MessageInstance &message : view) {
    // if(num_points_counter>msg_end_num)
    if (false) {
      break;
    }

    sensor_msgs::PointCloud2::ConstPtr pc2_msg =
        message.instantiate<sensor_msgs::PointCloud2>();
    VPointCloud pcl_cloud;
    pcl::fromROSMsg(*pc2_msg, pcl_cloud);

    // TODO: Put additional spacing condition here...
    if (transform_interpolation_buffer.Has(
            cartographer_ros::FromRos(pc2_msg->header.stamp))) {
      // if(num_points_counter>msg_start_num)
      if (true) {
        PointCloud pcl_cloud_noNaN;
        std::vector<int> indices;
        PointCloud::Ptr t_ppcl_cloud(new PointCloud());
        pcl::copyPointCloud(pcl_cloud, *t_ppcl_cloud);
        pcl::removeNaNFromPointCloud(*t_ppcl_cloud, pcl_cloud_noNaN, indices);
        VPointCloud pcl_vcloud_noNaN;
        pcl::copyPointCloud(pcl_cloud, indices, pcl_vcloud_noNaN);
        // pcl_pointclouds.push_back( pcl_vcloud_noNaN);

        // passthorugh
        t_ppcl_cloud->clear();
        pcl::copyPointCloud(pcl_vcloud_noNaN, *t_ppcl_cloud);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(t_ppcl_cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.6, 10000.0);
        indices.clear();
        pass.filter(indices);
        VPointCloud pcl_vcloud_noNaN_passthrough;
        pcl::copyPointCloud(pcl_vcloud_noNaN, indices,
                            pcl_vcloud_noNaN_passthrough);

        accumulator.GetVectorClouds()->push_back(pcl_vcloud_noNaN);
        // accumulator.GetVectorClouds()->push_back(
        // pcl_vcloud_noNaN_passthrough);

        const cartographer::transform::Rigid3d tracking_to_map =
            transform_interpolation_buffer.Lookup(
                cartographer_ros::FromRos(pc2_msg->header.stamp));
        // TO DO plug in transform param here...
        // const cartographer::transform::Rigid3d sensor_to_tracking;
        // const cartographer::transform::Rigid3f sensor_to_map =
        //(tracking_to_map * sensor_to_tracking).cast<float>();
        // odometry_poses.push_back(tf::Transform(
        // tf::Quaternion(tracking_to_map.rotation().x(),tracking_to_map.rotation().y(),tracking_to_map.rotation().z(),tracking_to_map.rotation().w()),
        // tf::Vector3(tracking_to_map.translation().x(),tracking_to_map.translation().y(),tracking_to_map.translation().z())));
        accumulator.GetOdometryPoses()->push_back(tf::Transform(
            tf::Quaternion(
                tracking_to_map.rotation().x(), tracking_to_map.rotation().y(),
                tracking_to_map.rotation().z(), tracking_to_map.rotation().w()),
            tf::Vector3(tracking_to_map.translation().x(),
                        tracking_to_map.translation().y(),
                        tracking_to_map.translation().z())));
        LOG(INFO) << tracking_to_map;
      }
      ++num_points_counter;
    }

    //ROS_INFO("Time message: %.10f %s points:%lu", pc2_msg->header.stamp.toSec(),
	//transform_interpolation_buffer.Has(cartographer_ros::FromRos(pc2_msg->header.stamp))?"has":"not", pcl_cloud.size());
  }


    if (b_opt_use_ref) {

      std::vector<std::string> ref_topics;
      ref_topics.push_back(ref_topic);
      rosbag::View ref_view(bag, rosbag::TopicQuery(ref_topics));

      for (const rosbag::MessageInstance& message : ref_view) {
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

	  accumulator.GetVectorClouds()->push_back( pcl_vcloud_noNaN);
	  //accumulator.GetRefVectorClouds()->push_back( pcl_vcloud_noNaN_passthrough);

	  const cartographer::transform::Rigid3d tracking_to_map =
	    transform_interpolation_buffer.Lookup(cartographer_ros::FromRos(pc2_msg->header.stamp));
	  // TO DO plug in transform param here...
	  //const cartographer::transform::Rigid3d sensor_to_tracking;
	  //const cartographer::transform::Rigid3f sensor_to_map =
	  //(tracking_to_map * sensor_to_tracking).cast<float>();
	  //odometry_poses.push_back(tf::Transform( tf::Quaternion(tracking_to_map.rotation().x(),tracking_to_map.rotation().y(),tracking_to_map.rotation().z(),tracking_to_map.rotation().w()), tf::Vector3(tracking_to_map.translation().x(),tracking_to_map.translation().y(),tracking_to_map.translation().z())));
	  accumulator.GetRefOdometryPoses()->push_back( tf::Transform( tf::Quaternion(tracking_to_map.rotation().x(),tracking_to_map.rotation().y(),tracking_to_map.rotation().z(),tracking_to_map.rotation().w()), tf::Vector3(tracking_to_map.translation().x(),tracking_to_map.translation().y(),tracking_to_map.translation().z())));
	  LOG(INFO) << tracking_to_map;
	}
	//ROS_INFO("Time message: %.10f %s points:%lu", pc2_msg->header.stamp.toSec(),
	   // transform_interpolation_buffer.Has(cartographer_ros::FromRos(pc2_msg->header.stamp))?"has":"not", pcl_cloud.size());
      }
    }
    bag.close();

    // test ring info
    //int no_of_points = 0;
    //for (VPointCloud::iterator p=pcl_cloud.begin(); p!= pcl_cloud.end(); p++)
    //{
       //if (p->ring == 7) no_of_points++;
    //}
    //ROS_INFO("no_of_points: %d", no_of_points);


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
    //accumulator.AccumulateIRWithTransform(tf::Transform());
    //accumulator.AccumulateIRWithTransform(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0,0,0)));
    //accumulator.AccumulateIRWithTransform(tf::Transform(tf::createQuaternionFromRPY(0, 0, M_PI), tf::Vector3(2.735,0,-1.376)));
    accumulator.AccumulateIRWithTransform(tf::Transform(tf::createQuaternionFromRPY(init_roll*M_PI/180, init_pitch*M_PI/180, init_yaw*M_PI/180), tf::Vector3(init_x,init_y,init_z)));
    //accumulator.AccumulateIRWithTransform(tf::Transform(tf::createQuaternionFromRPY(0, 9.75*M_PI/180, 0), tf::Vector3(0,-2.455,0)));
    //accumulator.AccumulateIRWithTransform(tf::Transform(tf::createQuaternionFromRPY(0.25*M_PI/180, 9.75*M_PI/180, 3.5*M_PI/180), tf::Vector3(0.25,-1.98,0.175)));
    accumulator.GetTotalVPointCloud()->header.frame_id = "map";
    //accumulator.GetTotalVPointCloud()->header.stamp    = ros::Time::now().toNSec();
    pc_pub.publish(*accumulator.GetTotalVPointCloud());
    if (b_opt_use_ref){
	accumulator.GetRefTotalVPointCloud()->header.frame_id = "map";
	ref_pc_pub.publish(*accumulator.GetRefTotalVPointCloud());
    }
    ros::spin();
  }
  else if (b_opt_calibrate)
  {
    // omp_set_dynamic(0);
    JExecutor jexecutor(&accumulator);
    double begin_time = ros::Time::now().toSec();
    LOG(INFO) << "Begin at:" << begin_time;
    float min_J_k;
    float min_J_k_x;
    float min_J_k_y;
    float min_J_k_z;
    float min_J_k_roll;
    float min_J_k_pitch;
    float min_J_k_yaw;
    float min_J;//=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z, init_roll, init_pitch, init_yaw);
    float J_k=0;
    float J_k_x=0;
    float J_k_y=0;
    float J_k_z=0;
    float J_k_roll=0;
    float J_k_pitch=0;
    float J_k_yaw=0;

    float final_val_x;
    float final_val_y;
    float final_val_z;
    float final_val_roll;
    float final_val_pitch;
    float final_val_yaw;
    float final_val;
    switch (mode)
    {
      case 1 :
              {double sum=0;
              int load_count=0;
              // for(int i=0;i<5;++i)
              // {
                min_J_k=10000000;
                J_k=0;
                for (int k=0; k<(((half_width_search*2)/search_resolution)+1) && ros::ok() ; k++)
                {
                  load_count+=1;
                  J_k=jexecutor.J_calc_wEuler_andLog(init_x-half_width_search+k*search_resolution,init_y,init_z, init_roll, init_pitch, init_yaw);
                  ROS_INFO("Processing:%d/%f",load_count,(1)*(((half_width_search*2)/search_resolution)+1));
                  if (J_k<min_J_k)
                  {
                    final_val=init_x-half_width_search+k*search_resolution;
                    //ROS_WARN("Optimized value=%f",init_x-half_width_search+k*search_resolution);
                    min_J_k=J_k;
                  }
                }
                //ROS_WARN("Optimized value x=%f",final_val+1.415);
                
              //   sum+=final_val;
              // }
              ROS_WARN("Optimized value x=%f",final_val);
              }break;

      case 2 :
              {double sum=0;
              int load_count=0;
              // for(int i=0;i<5;++i)
              // {
                min_J_k=10000000;
                J_k=0;
                for (int k=0; k<(((half_width_search*2)/search_resolution)+1) && ros::ok() ; k++)
                {
                  load_count+=1;
                  J_k=jexecutor.J_calc_wEuler_andLog(init_x,init_y-half_width_search+k*search_resolution,init_z, init_roll, init_pitch, init_yaw);
                  ROS_INFO("Processing:%d/%f",load_count,(1)*(((half_width_search*2)/search_resolution)+1));
                  if (J_k<min_J_k)
                  {
                    final_val=init_y-half_width_search+k*search_resolution;
                    // ROS_WARN("Optimized value=%f",init_y-half_width_search+k*search_resolution);
                    min_J_k=J_k;
                  }                  
                }
                //ROS_WARN("Optimized value y=%f",final_val);
              //   sum+=final_val;
              // }
              ROS_WARN("Optimized value y=%f",final_val);
              }break;

      case 3 :
              {double sum=0;
              int load_count=0;
              // for (int i=0;i<5;++i)
              // {
                min_J_k=10000000;
                J_k=0;
                for (int k=0; k<(((half_width_search*2)/search_resolution)+1) && ros::ok() ; k++)
                {
                  load_count+=1;
                  J_k=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z-half_width_search+k*search_resolution, init_roll, init_pitch, init_yaw);
                  ROS_INFO("Processing:%d/%f",load_count,(1)*(((half_width_search*2)/search_resolution)+1));
                  if (J_k<min_J_k)
                  {
                    final_val=init_z-half_width_search+k*search_resolution;
                    // ROS_WARN("Optimized value=%f",init_z-half_width_search+k*search_resolution);
                    min_J_k=J_k;
                  }
                }
                //ROS_WARN("Optimized value z=%f",final_val+1.698);
              //   sum+=final_val;
              // }
              ROS_WARN("Optimized value z=%f",final_val);
              }break;

      case 4 :
              {double sum=0;
              int load_count=0;
              // for (int i=0;i<5;++i)
              // {
                min_J_k=10000000;
                J_k=0;
                for (int k=0; k<(((half_width_search*2)/search_resolution)+1) && ros::ok() ; k++)
                {
                  load_count+=1;
                  J_k=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z, init_roll-half_width_search+k*search_resolution, init_pitch, init_yaw);
                  ROS_INFO("Processing:%d/%f",load_count,(1)*(((half_width_search*2)/search_resolution)+1));
                  if (J_k<min_J_k)
                  {
                    final_val=init_roll-half_width_search+k*search_resolution;
                    // ROS_WARN("Optimized value=%f",init_roll-half_width_search+k*search_resolution);
                    min_J_k=J_k;
                  }                  
                }
                //ROS_WARN("Optimized value roll=%f",final_val);
                // sum+=final_val;
              // }
              ROS_WARN("Optimized value roll=%f",final_val);
              }break;

      case 5 :
              {double sum=0;
              int load_count=0;
              // for (int i=0;i<5;++i)
              // {
                min_J_k=10000000;
                J_k=0;
                for (int k=0; k<(((half_width_search*2)/search_resolution)+1) && ros::ok() ; k++)
                {
                  load_count+=1;
                  J_k=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z, init_roll, init_pitch-half_width_search+k*search_resolution, init_yaw);
                  ROS_INFO("Processing:%d/%f",load_count,(1)*(((half_width_search*2)/search_resolution)+1));
                  if (J_k<min_J_k)
                  {
                    final_val=init_pitch-half_width_search+k*search_resolution;
                    // ROS_WARN("Optimized value=%f",init_pitch-half_width_search+k*search_resolution);
                    min_J_k=J_k;
                  }                  
                }
                //ROS_WARN("Optimized value pitch=%f",final_val);
                // sum+=final_val;
              // }
              ROS_WARN("Optimized value pitch=%f",final_val);
              }break;

      case 6 :
              {double sum=0;
              int load_count=0;
              //for (int i=0;i<5;++i)
              // {
                
                min_J_k=10000000;
                J_k=0;
                for (int k=0; k<(((half_width_search*2)/search_resolution)+1) && ros::ok() ; k++)
                {
                  load_count+=1;
                  // J_k=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z, init_roll, init_pitch, init_yaw-half_width_search+k*search_resolution);
                  //ROS_WARN("Optimized value=%f",init_yaw-half_width_search+k*search_resolution);
                  J_k=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z, init_roll, init_pitch, init_yaw-half_width_search+k*search_resolution);
                  ROS_INFO("Processing:%d/%f",load_count,(1)*(((half_width_search*2)/search_resolution)+1));
                  //ROS_WARN("weight=%f",J_k);
                  if (J_k<min_J_k)
                  {
                    final_val=init_yaw-half_width_search+k*search_resolution;
                    ROS_WARN("Optimized value=%f",init_yaw-half_width_search+k*search_resolution);
                    min_J_k=J_k;
                  }                  
                }
                //ROS_WARN("Optimized value yaw=%f",final_val*-1);
                // sum+=final_val;
              // }
              //ROS_WARN("Optimized value yaw=%f",(sum/5)*-1);
              ROS_WARN("Optimized value yaw=%f",(final_val)*1);
              }break;

      case 7 :
              {
                float delta=1;
                float num_partcle=500;
                float yaw_center=init_yaw;
                float roll_center=init_roll;
                float pitch_center=init_pitch;
                int count=0;
                final_val_yaw=init_yaw;
                final_val_pitch=init_pitch;
                final_val_roll=init_roll;

                float prev_final_val_roll=init_roll;
                float prev_final_val_pitch=init_pitch;
                float prev_final_val_yaw=init_yaw;
                float yaw_search;
                float roll_search;
                float pitch_search;
                float rand_num;
                int count_inner=0;
                min_J=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z, init_roll, init_pitch, init_yaw);
                while(count<200 && count_inner<50)
                {
                  //yaw_search=yaw_center+random(-1,1)*half_width_search;
                  for (int particle_index=1;particle_index<num_partcle;++particle_index)
                  {
                    if(count_inner>50)
                    {break;} 
                    rand_num=(rand()%100)*0.01;
                    //ROS_INFO("random number:%f",rand_num); 
                    //yaw_search=yaw_center-half_width_search+(rand_num*half_width_search*2);
                    roll_search=roll_center-(particle_index*search_resolution);
                    pitch_search=pitch_center-(particle_index*search_resolution);
                    yaw_search=yaw_center-(particle_index*search_resolution);

                    ROS_INFO("searching at:%f",yaw_search);  
                    J_k=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z, roll_search, pitch_search, yaw_search);
                    
                    if(abs(J_k-min_J)<0.1 || J_k<min_J)
                    {
                      //roll_center=roll_search;
                      //pitch_center=pitch_search;

                      prev_final_val_roll=final_val_roll;
                      prev_final_val_pitch=final_val_pitch;
                      prev_final_val_yaw=final_val_yaw;

                      roll_search=(roll_search+prev_final_val_roll)/2;
                      pitch_search=(pitch_search+prev_final_val_pitch)/2;
                      yaw_center=(yaw_search+prev_final_val_yaw)/2;

                      final_val_roll=(roll_search+prev_final_val_roll)/2;
                      final_val_pitch=(pitch_search+prev_final_val_pitch)/2;
                      final_val_yaw=(yaw_search+prev_final_val_yaw)/2;
                      // final_val=(yaw_search+prev_final_val)/2;
                      
                      min_J=J_k;
                      count_inner=0;
                      ROS_WARN("center cahnage:%f,%f,%f",final_val_roll,final_val_pitch,final_val_yaw);
                      break;
                    }

                    //Searching other side
                    ++count_inner;

                    roll_search=roll_center+(particle_index*search_resolution);
                    pitch_search=pitch_center+(particle_index*search_resolution);
                    yaw_search=yaw_center+(particle_index*search_resolution);
                    ROS_INFO("searching at:%f",yaw_search);  
                    J_k=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z, roll_search, pitch_search, yaw_search);
                    if(abs(J_k-min_J)<0.1 || J_k<min_J)
                    {
                      //roll_center=roll_search;
                      //pitch_center=pitch_search;
                      prev_final_val_roll=final_val_roll;
                      prev_final_val_pitch=final_val_pitch;
                      prev_final_val_yaw=final_val_yaw;

                      roll_search=(roll_search+prev_final_val_roll)/2;
                      pitch_search=(pitch_search+prev_final_val_pitch)/2;
                      yaw_center=(yaw_search+prev_final_val_yaw)/2;

                      final_val_roll=(roll_search+prev_final_val_roll)/2;
                      final_val_pitch=(pitch_search+prev_final_val_pitch)/2;
                      final_val_yaw=(yaw_search+prev_final_val_yaw)/2;
                      // final_val=(yaw_search+prev_final_val)/2;

                      min_J=J_k;
                      count_inner=0;
                      ROS_WARN("center cahnage:%f,%f,%f,",final_val_roll,final_val_pitch,final_val_yaw);
                      break;
                    }
                    ++count_inner;
                  }
                  //delta=abs(prev_final_val-final_val);
                  ++count;
                }
                ROS_WARN("final values:%f,%f,%f,",final_val_roll,final_val_pitch,final_val_yaw);
              }break;

      case 8 :
              {
                int load_count=0;
                
                min_J_k_x=10000000;
                min_J_k_y=10000000;
                min_J_k_z=10000000;
                min_J_k_roll=10000000;
                min_J_k_pitch=10000000;
                min_J_k_yaw=10000000;
                J_k_x=0;
                J_k_y=0;
                J_k_z=0;
                J_k_roll=0;
                J_k_pitch=0;
                J_k_yaw=0;
                float learning_rate=0.1;
                for(int i=0 ; i<2 ; ++i)
                {

                for (int k=0; k<(((half_width_search_xyz*2)/search_resolution_xyz)+1) && ros::ok() ; k++)
                { 
                  load_count+=1;
                  ROS_INFO("Processing:%d/%f",load_count,(1)*(((half_width_search_xyz*2)/search_resolution_xyz)+1));
                  
                  //search for x 
                  J_k_x=jexecutor.J_calc_wEuler_andLog(init_x-half_width_search_xyz+k*search_resolution_xyz,init_y,init_z, init_roll, init_pitch, init_yaw);
                  if (J_k_x<min_J_k_x)
                  {
                    final_val_x=init_x-half_width_search_xyz+k*search_resolution_xyz;
                    //ROS_WARN("Optimized value=%f",init_x-half_width_search+k*search_resolution);
                    min_J_k_x=J_k_x;
                  }

                  //search for y
                  J_k_y=jexecutor.J_calc_wEuler_andLog(init_x,init_y-half_width_search_xyz+k*search_resolution_xyz,init_z, init_roll, init_pitch, init_yaw);
                  if (J_k_y<min_J_k_y)
                  {
                    final_val_y=init_y-half_width_search_xyz+k*search_resolution_xyz;
                    // ROS_WARN("Optimized value=%f",init_y-half_width_search+k*search_resolution_xyz);
                    min_J_k_y=J_k_y;
                  }

                  //search for z
                  J_k_z=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z-half_width_search_xyz+k*search_resolution_xyz, init_roll, init_pitch, init_yaw);
                  if (J_k_z<min_J_k_z)
                  {
                    final_val_z=init_z-half_width_search_xyz+k*search_resolution_xyz;
                    // ROS_WARN("Optimized value=%f",init_z-half_width_search+k*search_resolution);
                    min_J_k_z=J_k_z;
                  }
                }
                ROS_WARN("Optimized value x=%f",final_val_x);
                ROS_WARN("Optimized value y=%f",final_val_y);
                ROS_WARN("Optimized value z=%f",final_val_z);


                load_count=0;
                for (int k=0; k<(((half_width_search_rpy*2)/search_resolution_rpy)+1) && ros::ok() ; k++)
                {
                  load_count+=1;
                  ROS_INFO("Processing:%d/%f",load_count,(1)*(((half_width_search_rpy*2)/search_resolution_rpy)+1));
                  
                  //search for roll
                  J_k_roll=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z, init_roll-half_width_search+k*search_resolution, init_pitch, init_yaw);
                  // ROS_INFO("Processing:%d/%f",load_count,(1)*(((half_width_search*2)/search_resolution)+1));
                  if (J_k_roll<min_J_k_roll)
                  {
                    final_val_roll=init_roll-half_width_search+k*search_resolution;
                    // ROS_WARN("Optimized value=%f",init_roll-half_width_search+k*search_resolution);
                    min_J_k_roll=J_k_roll;
                  }

                  //search for pitch
                  J_k_pitch=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z, init_roll, init_pitch-half_width_search+k*search_resolution, init_yaw);
                  ROS_INFO("Processing:%d/%f",load_count,(1)*(((half_width_search*2)/search_resolution)+1));
                  if (J_k_pitch<min_J_k_pitch)
                  {
                    final_val_pitch=init_pitch-half_width_search+k*search_resolution;
                    // ROS_WARN("Optimized value=%f",init_pitch-half_width_search+k*search_resolution);
                    min_J_k_pitch=J_k_pitch;
                  }      

                  //search for yaw
                  J_k_yaw=jexecutor.J_calc_wEuler_andLog(init_x,init_y,init_z, init_roll, init_pitch, init_yaw-half_width_search+k*search_resolution);
                  ROS_INFO("Processing:%d/%f",load_count,(1)*(((half_width_search*2)/search_resolution)+1));
                  //ROS_WARN("weight=%f",J_k);
                  if (J_k_yaw<min_J_k_yaw)
                  {
                    final_val_yaw=init_yaw-half_width_search+k*search_resolution;
                    ROS_WARN("Optimized value=%f",init_yaw-half_width_search+k*search_resolution);
                    min_J_k_yaw=J_k_yaw;
                  }  
                }
                ROS_WARN("Optimized value roll=%f",final_val_roll);
                ROS_WARN("Optimized value pitch=%f",final_val_pitch);
                ROS_WARN("Optimized value yaw=%f",final_val_yaw);

                init_x=final_val_x;
                init_y=final_val_y;
                init_z=final_val_z;
                init_roll=final_val_roll;
                init_pitch=final_val_pitch;
                init_yaw=final_val_yaw;

                half_width_search_xyz*=learning_rate;
                search_resolution_xyz*=learning_rate;
                half_width_search_rpy*=learning_rate;
                search_resolution_rpy*=learning_rate;
                }
                int n_1=0; char buffer_1[200];
                n_1 += sprintf(buffer_1, "%07.5f %07.5f %07.5f %07.5f %07.5f %07.5f %07.5f\n", final_val_x, final_val_y, final_val_z, final_val_roll, final_val_pitch, final_val_yaw,J_k_yaw);
                filewriter.write(buffer_1, n_1);
                filewriter.flush();
              } break;

      

    } 
  }
  
}
