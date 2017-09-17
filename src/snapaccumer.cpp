#include <cstdio>
#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"

// tf
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

// helper functions
//#include "helper/yaw_angle_calc.hpp"

// PCL
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZI> PCLPointCloudXYI;
struct ScansHist
{
  tf::Stamped<tf::Pose> pose;
  double d;
  PCLPointCloudXYI pc;
};

typedef tf::Stamped<tf::Pose> TFPose;
/**
 * A helper function to transfer TFPose.
 * This is because problem no copy constructor (maybe) for TFPose
 * @param[in] source  source of TFPose
 * @param[out] target  TFPose applied here
 *
 * Currently no error catcher
 */
void transferPose(const TFPose & source , TFPose & target)
{
  target.stamp_ = source.stamp_;
  target.frame_id_ = source.frame_id_;
  target.setData(source);
} 

/***
 * This a simple test app that requests a point cloud from the
 * point_cloud_assembler every 80 seconds, and then publishes the
 * resulting data
 */
namespace laser_assembler
{

class PeriodicSnapshotter
{

  // callback for caching latest tf from pc2
  // also now for caching point cloud data as well
  void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    history_mutex_.lock();

    latest_pc2_time=msg->header.stamp;
    waiting_for_pc2 = false;
    pcl::fromROSMsg(*msg, latest_pcl_pc);

    history_mutex_.unlock();
  }
public:

  PeriodicSnapshotter(const double & window_size=3.0, const double & step_size=0.2, const double & publish_per_dist=0.5, const double & timer_period=0.1, bool publish_before_fullsized=false) : window_size_(window_size), step_size_(step_size), publish_per_dist_(publish_per_dist), timer_period_(timer_period), publish_before_fullsized_(publish_before_fullsized)
  {
    // indicate that we are waiting for first pc2 msg
    waiting_for_pc2 = true;

    // make sure we start at zero
    dist_tot = 0;
    dist_accum = 0;
    last_dist_tot = 0;
    dist_publish = 0;

    // begin index, we cache to faster
    begin_hist_index_ = 0;

    // must publish once first to keep publishing camera_last
    has_published_once = false;

    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud2 > ("assembled_cloud2", 1);

    // Create a publisher for odometry type

    pub_odom = n_.advertise<nav_msgs::Odometry> ("/accum_odom", 5);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans2>("assemble_scans2");

    // Start the timer that will trigger the processing loop (timerCallback)
    timer_ = n_.createTimer(ros::Duration(timer_period_), &PeriodicSnapshotter::timerCallback, this);

    // Need to track if we've called the timerCallback at least once
    first_time_ = true;

    // setup watch on the cloud itself to track the tf
    sub_ = n_.subscribe("/velodyne_cloud_registered",1,&laser_assembler::PeriodicSnapshotter::pc2Callback, this);
  }

  void getCurrentCameraPose(const ros::Time & t, tf::Stamped<tf::Pose> & pose);

  void timerCallback(const ros::TimerEvent& e)
  {
    // dummy variable (not used in SE2)
    // disable angle calculation for speed
#if 0
    double pitch, roll;
#endif 

    // helping to get axis angle representation
    Eigen::Matrix3d emat;

    // check if first cloud has arrived
    if (waiting_for_pc2) return;

    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_)
    {
      first_time_ = false;
      // use this first one as first ref to estimate motion
      prev_tref = latest_pc2_time; //e.current_real;
      // get current position of camera
      // as last position
      // TODO: extend to SE3
      getCurrentCameraPose(latest_pc2_time, last_tf_pose);

      // first one, put into history

      ScansHist sh;
      sh.d = 0; // first distance is zero
      sh.pose = last_tf_pose;
      history_.push_back(sh);

      x_prev = last_tf_pose.getOrigin().x();
      y_prev = last_tf_pose.getOrigin().y();

      // disable angle calculation for speed
#if 0
	last_tf_pose.getBasis().getRPY(pitch, yaw_prev, roll);
	tf::matrixTFToEigen(last_tf_pose.getBasis(), emat);
	Eigen::AngleAxisd angleaxis(emat);
	yaw_prev = angleaxis.angle();
#endif 

      return;
    }

    // check if camera has moved certain distance or heading
    bool has_moved = false;
    // get current one and compare with previous
    getCurrentCameraPose(latest_pc2_time, current_tf_pose);
    // get details can collect distance
    // TODO: extend to SE3
    double x_now = current_tf_pose.getOrigin().x();
    double y_now = current_tf_pose.getOrigin().z();
    double delta_x = x_now - x_prev;
    double delta_y = y_now - y_prev;
    double dist = sqrt ( delta_x*delta_x + delta_y*delta_y );
    last_dist_tot = dist_tot;
    dist_tot +=dist;
    dist_accum +=dist;
    dist_publish += dist;

    if ( step_size_ <= dist_accum ) 
    {
      ROS_INFO("Triggering because translation of: %.2f", dist_accum);
      // accumulate in history
      ScansHist sh;
      sh.d = dist_tot;
      sh.pc = PCLPointCloudXYI(latest_pcl_pc);
      transferPose(current_tf_pose, sh.pose);
      history_.push_back(sh);

      // reset accumulator
      dist_accum=0;
      // indicate a step size has been passed
      has_moved = true;

      // ok, now check if we need to recalculate window
      if ( dist_tot > window_size_ )
      {
	while (dist_tot - history_.at(begin_hist_index_).d > window_size_) {
	  begin_hist_index_++;
	} 
      }
      // don publish if this flag is set
      else if (false == publish_before_fullsized_) {
	// recheck if really close to fullsized
	// hardcoded factor here
	has_moved = false;
	if (dist_tot * 1.5 > window_size_) {
	  has_moved = true;
	}
      }

      // we add another publisher distance accumulator
      if (has_moved)
      {
	// recheck if really we should publish
	if (dist_publish < publish_per_dist_)
	{
	  has_moved = false;
	}
	else
	{
	  // well, reset if so
	  dist_publish = 0;
	}
      }
    }
    else ROS_INFO("trans is : x:%.2f y:%.2f total:%.3f", delta_x, delta_y, dist_accum);


    // get details can collect heading change
    // TODO: extend to SE3

    x_prev = x_now;
    y_prev = y_now;

    if (has_moved)
    {
      tf::Stamped<tf::Pose> begin_pose;
      transferPose(history_.at(begin_hist_index_).pose, begin_pose);
      tf::Stamped<tf::Pose> begin_pose_inverse;
      begin_pose_inverse.setData(history_.at(begin_hist_index_).pose.inverse());
      ROS_INFO("begin_hist_index_: %ld", begin_hist_index_);

      latest_accum_pc.clear();
      for (std::vector<ScansHist>::const_iterator i = history_.begin()+begin_hist_index_; i != history_.end(); ++i) {
	latest_accum_pc += i->pc;
      }
      pcl_ros::transformPointCloud(latest_accum_pc , latest_accum_pc , begin_pose_inverse);
      transferPose(begin_pose, current_last_tf_pose);
      
      // we moved pref_tref here as it is still needed for transform to camera
      prev_tref = e.current_real;
      // store previous time and pose for next iteration
      last_tf_pose = current_tf_pose;
      has_published_once = true;
    }

    // maintain happy camera_last tf happy
    if (has_published_once)
    {
      //ros::Time transform_expiration = e.current_real + ros::Duration(timer_period_);
      tf::StampedTransform tmp_tf_stamped;
      tmp_tf_stamped.setData(current_last_tf_pose);
      tmp_tf_stamped.stamp_ = ros::Time::now();
      tmp_tf_stamped.frame_id_ = "camera_init";
      tmp_tf_stamped.child_frame_id_ = "camera_last";
      tf_caster_.sendTransform(tmp_tf_stamped);

      nav_msgs::Odometry odom_accum;

      if (has_moved) {
	sensor_msgs::PointCloud2 accumulated;
	pcl::toROSMsg(latest_accum_pc, accumulated);
	accumulated.header.stamp = ros::Time::now();
	accumulated.header.frame_id = "/camera_last";
	pub_.publish(accumulated);
	odom_accum.header.stamp = tmp_tf_stamped.stamp_;
	odom_accum.header.frame_id = tmp_tf_stamped.frame_id_;
	odom_accum.child_frame_id = tmp_tf_stamped.child_frame_id_;
	tf::poseTFToMsg( tmp_tf_stamped, odom_accum.pose.pose);
	pub_odom.publish(odom_accum);
      }

    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceClient client_;
  ros::Timer timer_;
  bool first_time_;
  ros::Time prev_tref;
  ros::Time latest_pc2_time;
  PCLPointCloudXYI latest_pcl_pc;
  PCLPointCloudXYI latest_accum_pc;
  
  tf::TransformListener tf_listen_;
  tf::TransformBroadcaster tf_caster_;
  tf::Stamped<tf::Pose> current_tf_pose;
  tf::Stamped<tf::Pose> last_tf_pose;
  tf::Stamped<tf::Pose> current_last_tf_pose;

  double x_prev;
  double y_prev;

  bool waiting_for_pc2;
  double dist_tot, dist_accum, dist_publish;
  double last_dist_tot;
  bool has_published_once;

  // for window size and step size
  std::vector<ScansHist> history_;
  boost::mutex history_mutex_;
  double window_size_;
  double step_size_;
  double publish_per_dist_;
  double timer_period_;

  size_t begin_hist_index_;
  bool publish_before_fullsized_;
  ros::Publisher pub_odom;
} ;

// Give time to transform look the camera pose
// Carefull the camera frame id is HARDCODED
void PeriodicSnapshotter::getCurrentCameraPose(const ros::Time & t, tf::Stamped<tf::Pose> & pose)
{
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)), t, "aft_mapped");
  try
  {
    this->tf_listen_.transformPose("camera_init", ident, pose);
  }
  catch(tf::TransformException & e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
  }
  // check if transformPose return same time in the stamped msg
  //ROS_INFO("time same when transformPose: %s", t == pose.stamp_ ? "true" : "false");
}


}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n, pnh("~");
  //ROS_INFO("Waiting for [build_cloud] to be advertised");
  //ros::service::waitForService("build_cloud");
  //ROS_INFO("Found build_cloud! Starting the snapshotter");
  // parameterize
  double timer_period, step_size, publish_per_dist, window_size;
  bool publish_before_window_filled;
  pnh.param("timer_period", timer_period, 0.1);
  pnh.param("window_size", window_size, 3.0);
  pnh.param("step_size", step_size, 0.2);
  pnh.param("publish_per_dist", publish_per_dist, 0.5);
  pnh.param("publish_before_window_filled", publish_before_window_filled, false);
  PeriodicSnapshotter snapshotter(window_size, step_size, publish_per_dist, timer_period, publish_before_window_filled);
  ros::spin();
  return 0;
}
