/**@file	/home/agoy/Documents/workspace/velo_scanmatch/src/trajProcessLoam.cpp
 * @author	agoy
 * @version	704
 * @date
 * 	Created:	21st Feb 2018
 * 	Last Update:	21st Feb 2018
 * There are several possibilities to fuse information for SLAM:
 * 
 * although we can still use graph slam framework:
 * - first, fuse odom with lidar mapping
 * - second, fuse them with GPS, xy prior
 * - third, do search for revisit loop closure possibilities
 *   , this means adding potential lidar scanmatching. However,
 *   , because Loam only stores local map, if revisit is over
 *   a very big loop, we dun have local map already (swept away
 *   by the moving window). Thus only submapping-based loop
 *   closure (like in cartographer) could work.
 * - forth, only using odometry and GPS is also possible. However,
 *   roll, pitch, z are probably errornous.
 *
 * We can test without SLAM first:
 * - use the GPS odometry (i.e. GT) to build map (no Loam)
 * - then with odom and Loam matching (lm only <- last stage of Loam)
 * - compare whether error reduced
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
// #include "proto_stream.h"
// #include "trajectory.pb.h"

#include "time_conversion.h"

#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/buffer.h"

#include "sensor_msgs/PointCloud2.h"

#include "transform.h"
#include "/root/catkin_imcl_ws/src/ndtbasedloc/src/transform_interpolation_buffer.h"

#include "tf/transform_datatypes.h"

#include "src/graph_slam.hpp"
#include "src/loam_sc.hpp"
#include "src/loam_lo.hpp"
#include "src/loam_lm.hpp"


#include "point_types.h"
#include <pcl/point_types.h>
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

#include <pcl/io/pcd_io.h>

#include <pcl/search/impl/search.hpp>

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE

typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
pcl::KdTreeFLANN<pcl::PointXYZ> ref_kdtree;

DEFINE_string(traj_filename, "",
        "Proto stream file containing the pose graph.");
DEFINE_string(odom_filename, "",
        "Proto stream file containing the odom for undistort.");
DEFINE_string(bag_filenames, "",
        "Bags to process, must be in the same order as the trajectories "
        "in 'pose_graph_filename'.");
DEFINE_string(pcd_filename, "",
        "If non empty, during publish also save pcd file (Binary compressed).");
DEFINE_bool(voxel_grid_pcd, false,
        "Voxel Filter before save.");
DEFINE_bool(random_sample_pcd, false,
        "random Filter before save.");

class Accumulator
{
    public:
        Accumulator() : use_ref_(false), use_beamlayer_contraint_(true){
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
        void setBeamlayerContraint(const bool & use_beamlayer_contraint){use_beamlayer_contraint_ = use_beamlayer_contraint;};

    private:
        std::vector<tf::Pose> odometry_poses_;
        std::vector<VPointCloud> pcl_pointclouds_;
        PointCloud::Ptr  total_pointcloud_;
        VPointCloud::Ptr total_vpointcloud_;

        std::vector<tf::Pose> ref_odometry_poses_;
        std::vector<VPointCloud> ref_pcl_pointclouds_;
        PointCloud::Ptr  ref_total_pointcloud_;
        VPointCloud::Ptr ref_total_vpointcloud_;

        const tf::Transform correction_after_  = tf::Transform( tf::createQuaternionFromRPY(1.570795,0,1.570795));
        const tf::Transform correction_before_ = tf::Transform( tf::createQuaternionFromRPY(1.570795,0,1.570795)).inverse();

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_;

        bool use_ref_;
        bool use_beamlayer_contraint_;
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
    constexpr float max_distance_between_two_points = 0.2; //0.5; //0.2; //0.36; //0.2; //0.8; //0.2;
    constexpr int skip_points = 16; //4; //16; //32; //16; //8; //16;
    constexpr int max_relations_per_i = 1;
    constexpr float max_dist_squared = max_distance_between_two_points * max_distance_between_two_points;
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
    if (use_ref_) ref_kdtree.setInputCloud(ref_total_pointcloud_);

    std::vector<float> Js (total_pointcloud_->size());
    std::vector<std::vector<int>>  takens (total_pointcloud_->size());


    omp_lock_t writelock;
    omp_init_lock(&writelock);

    if (use_ref_) {
#pragma omp parallel for num_threads(3)
        for(int i=0;i<total_pointcloud_->points.size();i+=skip_points)
        {   
            Js[i] = 0;

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

                        if (beam_j == neighbor_beam  || false == use_beamlayer_contraint_)
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
        ROS_INFO("Clouds size: %lu, odometry_poses size: %lu, total_points: %lu", ref_pcl_pointclouds_.size(), ref_odometry_poses_.size(), ref_total_pointcloud_->size());
    }
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
    CHECK(!FLAGS_odom_filename.empty())
        << "-odom_filename is missing.";
    CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";

    ros::init(argc, argv, "trajProcess");
    ros::NodeHandle nh, pnh("~");
    bool b_opt_publish_pc, b_opt_calibrate, b_opt_use_ref;
    bool b_opt_use_beamlayer, b_opt_use_passthrough;
    std::string pc_topic, ref_topic;
    pnh.param("publish_pc", b_opt_publish_pc, false);
    pnh.param("use_ref", b_opt_use_ref, false);
    pnh.param<std::string>("pc_topic"  , pc_topic, "/velodyne_right/velodyne_points");
    pnh.param("calibrate", b_opt_calibrate, false);
    pnh.param<std::string>("ref_topic"  , ref_topic, "/velodyne_left/velodyne_points");
    pnh.param("use_beamlayer", b_opt_use_beamlayer, true);
    pnh.param("use_passthrough", b_opt_use_passthrough, false);

    double sensor_pose_euler[6];
    pnh.param<double>("sensor_pose_x", sensor_pose_euler[0], 0);
    pnh.param<double>("sensor_pose_y", sensor_pose_euler[1], 0);
    pnh.param<double>("sensor_pose_z", sensor_pose_euler[2], 0);
    pnh.param<double>("sensor_pose_roll", sensor_pose_euler[3], 0);
    pnh.param<double>("sensor_pose_pitch", sensor_pose_euler[4], 0);
    pnh.param<double>("sensor_pose_yaw", sensor_pose_euler[5], 0);
    float voxel_filter_leaf;
    pnh.param<float>("voxel_filter_leaf", voxel_filter_leaf, 0.01);
    float random_filter_percentage;
    pnh.param<float>("random_filter_percentage", random_filter_percentage, 0.1);
    std::string base_dir_path;
    pnh.param<std::string>("base_dir_path", base_dir_path, "");
    ROS_INFO_STREAM("Using base dir path: " << base_dir_path);
    int b_opt_cloud_feature = 0;
    pnh.param<int>("opt_cloud_feature", b_opt_cloud_feature, 0);


    ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>("total_pc", 1, true);
    ros::Publisher ref_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("total_ref_pc", 1, true);

    ROS_INFO("Using traj file: %s", FLAGS_traj_filename.c_str());
    cartographer::io::ProtoStreamReader reader(base_dir_path + "/" + FLAGS_traj_filename);
    cartographer::mapping::proto::Trajectory proto_traj;
    CHECK(reader.ReadProto(&proto_traj));
    ROS_INFO("nodes contained %d", proto_traj.node_size());

    ROS_INFO("Using odom file: %s", FLAGS_odom_filename.c_str());
    cartographer::io::ProtoStreamReader reader_odom(base_dir_path + "/" + FLAGS_odom_filename);
    cartographer::mapping::proto::Trajectory proto_odom;
    CHECK(reader_odom.ReadProto(&proto_odom));
    ROS_INFO("nodes contained %d", proto_odom.node_size());

    for (auto i : proto_traj.node())
    {
        ROS_INFO("%.10f", cartographer_ros::ToRos(cartographer::common::FromUniversal(i.timestamp())).toSec());
        //LOG(INFO) << cartographer::transform::ToRigid3(i.pose());
    }

    cartographer::transform::TransformInterpolationBuffer
        transform_interpolation_buffer(proto_traj);
    rosbag::Bag bag;
    bag.open(base_dir_path + "/" + FLAGS_bag_filenames, rosbag::bagmode::Read);
    ROS_INFO("Using bag file: %s", FLAGS_bag_filenames.c_str());

    // we use extracting loam features to build map
    // Note: the TIB input here will be used for undistortion
    // e.g. if it is a Loam or a GT output, may be different then the
    // original odom (used for mapping/localization)
    std::shared_ptr<cartographer::transform::TransformInterpolationBuffer> tib =
        std::make_shared<cartographer::transform::TransformInterpolationBuffer>(
            (proto_odom));
    looam::ScanRegistration scan_registrar(tib);

    std::vector<std::string> topics;
    topics.push_back(pc_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    //std::vector<tf::Pose> odometry_poses;
    //std::vector<VPointCloud> pcl_pointclouds;
    Accumulator accumulator;
    accumulator.setUseRef(b_opt_use_ref);
    accumulator.setBeamlayerContraint(b_opt_use_beamlayer);
    //pcl::VoxelGrid<VPointCloud> sor;

    typedef pcl::PointXYZI PointType;
    using PointCloudConstPtr =
        std::shared_ptr<const pcl::PointCloud<PointType>>;

    for (const rosbag::MessageInstance& message : view) {
        sensor_msgs::PointCloud2::ConstPtr pc2_msg = message.instantiate<sensor_msgs::PointCloud2>();
        VPointCloud pcl_cloud;
        pcl::fromROSMsg(*pc2_msg, pcl_cloud);

        // TODO: Put additional spacing condition here...
        if(transform_interpolation_buffer.Has(cartographer_ros::FromRos(pc2_msg->header.stamp)))
        {
            scan_registrar.HandleLaserCloud(pc2_msg);
            // PointCloudConstPtr corner_cloud, surf_cloud;
            PointCloudConstPtr corner_cloud = (scan_registrar.GetCornerPointsLessSharp());
            PointCloudConstPtr surf_cloud = (scan_registrar.GetSurfPointsLessFlat());
            // PointCloudConstPtr corner_cloud = (scan_registrar.GetCornerPointsSharp());
            // PointCloudConstPtr surf_cloud = (scan_registrar.GetSurfPointsFlat());

            // The type currently being used is XYZIR, however loam uses XYZI
            switch (b_opt_cloud_feature) {
            case 1: // corner features extraction
            pcl_cloud.clear();
            // pcl::copyPointCloud(*corner_cloud, pcl_cloud);
            // need to do manually as we want to switch axes
            for (auto& point : corner_cloud->points) {
                VPoint p;
                p.x = point.z;
                p.y = point.x;
                p.z = point.y;
                p.ring = 0;
                p.intensity = point.intensity;
                pcl_cloud.push_back(p);
            }
            break;
            case 2: // surf features extraction
            pcl_cloud.clear();
            // pcl::copyPointCloud(*surf_cloud, pcl_cloud);
            // need to do manually as we want to switch axes
            for (auto& point : surf_cloud->points) {
                VPoint p;
                p.x = point.z;
                p.y = point.x;
                p.z = point.y;
                p.ring = 0;
                p.intensity = point.intensity;
                pcl_cloud.push_back(p);
            }
            break;
            case 0:
            default:;
            }

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

            if (b_opt_use_passthrough)
                accumulator.GetRefVectorClouds()->push_back( pcl_vcloud_noNaN_passthrough);
            else
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

        ROS_INFO("Time message: %.10f %s points:%lu", pc2_msg->header.stamp.toSec(),
                transform_interpolation_buffer.Has(cartographer_ros::FromRos(pc2_msg->header.stamp))?"has":"not", pcl_cloud.size());
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

                if (b_opt_use_passthrough)
                    accumulator.GetRefVectorClouds()->push_back( pcl_vcloud_noNaN_passthrough);
                else
                    accumulator.GetVectorClouds()->push_back( pcl_vcloud_noNaN);

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
            ROS_INFO("Time message: %.10f %s points:%lu", pc2_msg->header.stamp.toSec(),
                    transform_interpolation_buffer.Has(cartographer_ros::FromRos(pc2_msg->header.stamp))?"has":"not", pcl_cloud.size());
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
        //accumulator.AccumulateIRWithTransform(tf::Transform(tf::createQuaternionFromRPY(0, 0, M_PI), tf::Vector3(2.735,0,-1.376))); //denso
        //accumulator.AccumulateIRWithTransform(tf::Transform(tf::createQuaternionFromRPY(0, 0, -184.75*M_PI/180), tf::Vector3(2.735,0,-1.376)));
        //accumulator.AccumulateIRWithTransform(tf::Transform(tf::createQuaternionFromRPY(0, 9.75*M_PI/180, 0), tf::Vector3(0,-2.455,0)));
        //accumulator.AccumulateIRWithTransform(tf::Transform(tf::createQuaternionFromRPY(0.25*M_PI/180, 9.75*M_PI/180, 3.5*M_PI/180), tf::Vector3(0.25,-1.98,0.175)));

        accumulator.AccumulateIRWithTransform(tf::Transform(tf::createQuaternionFromRPY(sensor_pose_euler[3], sensor_pose_euler[4], sensor_pose_euler[5]), tf::Vector3(sensor_pose_euler[0],sensor_pose_euler[1],sensor_pose_euler[2])));
        accumulator.GetTotalVPointCloud()->header.frame_id = "map";
        //accumulator.GetTotalVPointCloud()->header.stamp    = ros::Time::now().toNSec();
        pc_pub.publish(*accumulator.GetTotalVPointCloud());
        if (b_opt_use_ref){
            accumulator.GetRefTotalVPointCloud()->header.frame_id = "map";
            ref_pc_pub.publish(*accumulator.GetRefTotalVPointCloud());
        }
        if (!FLAGS_pcd_filename.empty())
        {
            VPointCloud::Ptr cloud_t = accumulator.GetTotalVPointCloud();
            ROS_INFO_STREAM("Got " << cloud_t->width * cloud_t->height << " data points in frame " << cloud_t->header.frame_id << " with the following fields: " << pcl::getFieldsList (*cloud_t) << std::endl);
            if (FLAGS_random_sample_pcd)
            {
                ::pcl::RandomSample<pcl::PointXYZ> * rand_sampler = new ::pcl::RandomSample<pcl::PointXYZ>();
                VPointCloud::Ptr cloud_filtered (new VPointCloud ());
                std::vector<int> cloud_filtered_indices;
                rand_sampler->setInputCloud(accumulator.GetTotalPointCloud());
                rand_sampler->setSample(random_filter_percentage * accumulator.GetTotalVPointCloud()->size());
                rand_sampler->filter(cloud_filtered_indices);
                pcl::copyPointCloud(*accumulator.GetTotalVPointCloud(), cloud_filtered_indices, *cloud_filtered);
                delete rand_sampler;

                ROS_INFO("After random sample filtering");
                ROS_INFO_STREAM("Got " << cloud_filtered->width * cloud_filtered->height << " data points in frame " << cloud_filtered->header.frame_id << " with the following fields: " << pcl::getFieldsList (*cloud_filtered) << std::endl);
                pcl::io::savePCDFileBinaryCompressed(base_dir_path + "/" + FLAGS_pcd_filename, *cloud_filtered);
            }
            else if (FLAGS_voxel_grid_pcd)
            {
                //pcl::VoxelGrid<VPoint> sor;
                pcl::VoxelGrid<::pcl::PointXYZI> sor;
                //sor.setDownsampleAllData(false);
                ::pcl::PointCloud<::pcl::PointXYZI>::Ptr cloud_filtered (new ::pcl::PointCloud<::pcl::PointXYZI> ());
                pcl::copyPointCloud(*accumulator.GetTotalVPointCloud(),  *cloud_filtered);
                //sor.setInputCloud (accumulator.GetTotalVPointCloud());
                sor.setInputCloud (cloud_filtered);
                sor.setLeafSize (voxel_filter_leaf, voxel_filter_leaf, voxel_filter_leaf);
                sor.filter (*cloud_filtered);

                ROS_INFO("After voxel grid filtering");
                ROS_INFO_STREAM("Got " << cloud_filtered->width * cloud_filtered->height << " data points in frame " << cloud_filtered->header.frame_id << " with the following fields: " << pcl::getFieldsList (*cloud_filtered) << std::endl);
                pcl::io::savePCDFileBinaryCompressed(base_dir_path + "/" + FLAGS_pcd_filename, *cloud_filtered);
            }
            else
                pcl::io::savePCDFileBinaryCompressed(base_dir_path + "/" + FLAGS_pcd_filename, *cloud_t);
        }
        ros::spin();
    }
    else if (b_opt_calibrate)
    {
        omp_set_dynamic(0);
        JExecutor jexecutor(&accumulator);
        double begin_time = ros::Time::now().toSec();
        LOG(INFO) << "Begin at:" << begin_time;
        for (int k=0; k<25 && ros::ok() ; k++)
            //jexecutor.J_calc_wEuler_andLog(0,-1.98,0,0, k-10, 0);
            //jexecutor.J_calc_wEuler_andLog(0,0,0,0, static_cast<double>(k*0.25-2.5), 0);
            //jexecutor.J_calc_wEuler_andLog(0,-1.98,0,0, static_cast<double>(7.5+k*0.25), 0);
            //jexecutor.J_calc_wEuler_andLog(0,-1.98,0,0, static_cast<double>(0+k), 0);
            //jexecutor.J_calc_wEuler_andLog(0,-1.98,0,0, static_cast<double>(9.5+k*0.25), 0);
            //jexecutor.J_calc_wEuler_andLog(0,-1.98-0.25 + k*0.025,0,0, 9.75, 3.5);
            //jexecutor.J_calc_wEuler_andLog(0,-1.98,0,0, 9.75, 0+2.5 + k*0.25);
            //jexecutor.J_calc_wEuler_andLog(0,-1.98,0,0-2.5 + k * 0.25, 9.75, 3.5);
            //jexecutor.J_calc_wEuler_andLog(0+0.25 + k*0.025,-1.98,0, 0.25, 9.75, 3.5);
            //jexecutor.J_calc_wEuler_andLog(0.25,-1.98,0-0.25+ k*0.025, 0.25, 9.75, 3.5);

            jexecutor.J_calc_wEuler_andLog(2.735,0,-1.376, 0.0, 0, -3 - 180 - k*0.25);
        double end_time = ros::Time::now().toSec();
        LOG(INFO) << "End at:" << end_time;
        LOG(INFO) << "Time elapsed: " << end_time - begin_time;
    }
}

