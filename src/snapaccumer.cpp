#include <cstdio>
#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"

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
//#include <mutex> 

//// Templated message type for holding generic sensor messages.
//template<typename T>
//struct Message {
  //typename T::ConstPtr msg;
  //std::string tag;
  //typedef std::shared_ptr<Message<T> > Ptr;
  //typedef std::shared_ptr<const Message<T> > ConstPtr;
  //Message(const typename T::ConstPtr m, const std::string& t)
    //: msg(m), tag(t) {}
//};

//// Typedefs for queues of all sensor types.
//typedef std::vector<Message<sensor_msgs::PointCloud2>::ConstPtr> RosPC2Que;
//typedef std::vector<Message<pcl::PointCloud<pcl::PointXYZI> >::ConstPtr> PclPcQue;

// struct to handle scans history
// in future we can handle scans ourselves
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

  PeriodicSnapshotter(const double & window_size=1.5, const double & step_size=0.1, const double & timer_period=0.1) : window_size_(window_size), step_size_(step_size), timer_period_(timer_period)
  {
    // indicate that we are waiting for first pc2 msg
    waiting_for_pc2 = true;

    // make sure we start at zero
    dist_tot = 0;
    dist_accum = 0;
    last_dist_tot = 0;

    // begin index, we cache to faster
    begin_hist_index_ = 0;

    // must publish once first to keep publishing camera_last
    has_published_once = false;

    // Create a publisher for the clouds that we assemble
    //pub_ = n_.advertise<sensor_msgs::PointCloud2> ("assembled_cloud2", 1);
    pub_ = n_.advertise<pcl::PointCloud<pcl::PointXYZI> > ("assembled_cloud2", 1);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans2>("assemble_scans2");

    // Start the timer that will trigger the processing loop (timerCallback)
    //timer_ = n_.createTimer(ros::Duration(5,0), &PeriodicSnapshotter::timerCallback, this);
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
	//getCurrentCameraPose(e.current_real, last_tf_pose);
      getCurrentCameraPose(latest_pc2_time, last_tf_pose);

      // first one, put into history

      ScansHist sh;
      sh.d = 0; // first distance is zero
      sh.pose = last_tf_pose;
      history_.push_back(sh);

      x_prev = last_tf_pose.getOrigin().x();
      y_prev = last_tf_pose.getOrigin().y();
      //last_tf_pose.getBasis().getEulerYPR(yaw_prev, pitch, roll);
      //last_tf_pose.getBasis().getEulerYPR(pitch, yaw_prev, roll);

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
    //getCurrentCameraPose(e.current_real, current_tf_pose);
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

    // WARNING: HARDCODED Thresh
    if ( step_size_ <= dist_accum ) 
    {
      ROS_INFO("Triggering because translation of: %.2f", dist_accum);
      // accumulate in history
      ScansHist sh;
      sh.d = dist_tot;
      sh.pc = PCLPointCloudXYI(latest_pcl_pc);
      //sh.pose = current_tf_pose;
      // probably we can't do as above
      transferPose(current_tf_pose, sh.pose);
      history_.push_back(sh);

      // reset accumulator
      dist_accum=0;
      // indicate a step size has been passed
      has_moved = true;

      // ok, now check if we need to recalculate window
      if ( dist_tot > window_size_ ) //&& (dist_tot - last_dist_tot ) > step_size_)
      {
	while (dist_tot - history_.at(begin_hist_index_).d > window_size_) {
	  begin_hist_index_++;
	} 
      }
    }
    else ROS_INFO("trans is : x:%.2f y:%.2f total:%.3f", delta_x, delta_y, dist_accum);


    // get details can collect heading change
    // TODO: extend to SE3

    // disable angle calculation for speed
#if 0
      double yaw_now;
#endif 

      //if (! has_moved)
      //{
	//current_tf_pose.getBasis().getEulerYPR(yaw_now, pitch , roll);
	//current_tf_pose.getBasis().getEulerYPR(pitch, yaw_now , roll);

	// disable angle calculation for speed
#if 0
	current_tf_pose.getBasis().getRPY(pitch, yaw_now , roll);
	tf::matrixTFToEigen(current_tf_pose.getBasis(), emat);
	Eigen::AngleAxisd angleaxis(emat);
	yaw_now = angleaxis.angle();
#endif 

	//double delta_yaw = ANGLEDIFF(yaw_now , yaw_prev);
	//// WARNING: HARDCODED Thresh
	//if (0.25 <= delta_yaw) 
	//{
	  //ROS_INFO("Triggering because rotation of: %.2f %.2f", yaw_now, delta_yaw);
	  ////has_moved = true;
	//}
	//else ROS_INFO("rot is : %.2f %.2f %.2f", yaw_now, yaw_prev, delta_yaw);
      //}

    // TODO: extend to SE3
    x_prev = x_now;
    y_prev = y_now;
    //yaw_prev = yaw_now;

    if (has_moved)
    {
      tf::Stamped<tf::Pose> begin_pose;
      transferPose(history_.at(begin_hist_index_).pose, begin_pose);
      tf::Stamped<tf::Pose> begin_pose_inverse;
      //transferPose(TFPose(history_.at(begin_hist_index_).pose.inverse(), 0,""), begin_pose_inverse );
      begin_pose_inverse.setData(history_.at(begin_hist_index_).pose.inverse());
      ROS_INFO("begin_hist_index_: %ld", begin_hist_index_);
      //begin_pose_inverse.setData(history_.back().pose.inverse());
      // we need to go though the history and find latest window
      double found_window_size;
      int counter = 0;
      PCLPointCloudXYI accum_pc;
      for (std::vector<ScansHist>::const_iterator i = history_.begin()+begin_hist_index_; i != history_.end(); ++i, ++counter) {
	PCLPointCloudXYI temp_pc;
	// first pc is the same transform , speed up a bit
	//if (i != history_.begin() + begin_hist_index_) 
	if (true) 
	{
	  //pcl_ros::transformPointCloud(i->pc, temp_pc,  i->pose.inverse() * begin_pose);
	  //pcl_ros::transformPointCloud(i->pc, temp_pc, begin_pose * i->pose.inverse());
	  //pcl_ros::transformPointCloud(i->pc, temp_pc, i->pose * begin_pose_inverse);
	  //pcl_ros::transformPointCloud(i->pc, temp_pc, i->pose.inverse() );
	  pcl_ros::transformPointCloud(i->pc, temp_pc, begin_pose_inverse);
	  //pcl_ros::transformPointCloud(temp_pc, temp_pc, i->pose * begin_pose_inverse);
	  //pcl_ros::transformPointCloud(temp_pc, temp_pc, begin_pose_inverse * i->pose );
	}
	else
	  temp_pc = PCLPointCloudXYI(i->pc);
	accum_pc += temp_pc;
	//pcl_ros::transformPointCloud(accum_pc, accum_pc, begin_pose_inverse);
#if 0
	if ( (dist_tot - i->d) <= window_size_) {
	  // we should either found or frames are still too early
	  //begin_pose = i->pose;
	  // probably we can't do as above
	  transferPose(i->pose, begin_pose);
	  found_window_size = dist_tot - i->d;
	  //accum_pc += i->pc;
	} 
	else {
	  break;
	} 
#endif
      }
      ROS_INFO("window size: %.3f, %d", found_window_size, counter);

      // Need to run through one more time to inverse each frame

#if 0
      // Populate our service request based on our timer callback times
      AssembleScans2 srv;
      //srv.request.begin = e.last_real;
      //srv.request.begin = prev_tref;
      srv.request.begin = begin_pose.stamp_;
      //srv.request.end   = e.current_real;
      srv.request.end   = latest_pc2_time;

      // Make the service call
      if (client_.call(srv))
      {
	//ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
	ROS_INFO("Published Cloud with points") ;
	//pub_.publish(srv.response.cloud);
	//we got assembled scans but for reprocessing we need to transform to camera frame
        pcl::PointCloud<pcl::PointXYZI> pcl_pc;
	pcl::fromROSMsg(srv.response.cloud, pcl_pc);
	//pcl_ros::transformPointCloud("camera", prev_tref, pcl_pc, "camera_init", pcl_pc, tf_listen_);
	//pcl_ros::transformPointCloud( pcl_pc, pcl_pc, last_tf_pose.inverse());
	//pcl_ros::transformPointCloud( pcl_pc, pcl_pc, begin_pose);
	//pcl_ros::transformPointCloud( pcl_pc, pcl_pc, current_tf_pose.inverse());
	//pcl_pc.header.frame_id = "camera_last";
	pcl_pc.header.frame_id = "camera_init";
	pub_.publish(pcl_pc);
	has_published_once = true;
	//current_last_tf_pose.setData(begin_pose);
	transferPose(begin_pose, current_last_tf_pose);
	//current_last_tf_pose.setData(current_tf_pose);
      }
      else
      {
	ROS_ERROR("Error making service call\n") ;
      }

#else
      //pcl_ros::transformPointCloud( accum_pc, accum_pc, begin_pose.inverse());
      //pcl_ros::transformPointCloud( accum_pc, accum_pc, last_tf_pose.inverse());
      //accum_pc.header.frame_id = "camera_init";
      accum_pc.header.frame_id = "camera_last";
      //accum_pc.header.frame_id = "camera";
      pub_.publish(accum_pc);
      transferPose(begin_pose, current_last_tf_pose);
      //transferPose(begin_pose, history_.back().pose);
      //transferPose(history_.back().pose, current_last_tf_pose);
#endif
      
      // we moved pref_tref here as it is still needed for transform to camera
      prev_tref = e.current_real;
      // store previous time and pose for next iteration
      last_tf_pose = current_tf_pose;
    }

    // maintain happy camera_last tf happy
    if (true) //(has_published_once)
    {
      ros::Time transform_expiration = e.current_real + ros::Duration(timer_period_);
      tf::StampedTransform tmp_tf_stamped(current_last_tf_pose,
      //tf::StampedTransform tmp_tf_stamped(last_tf_pose,
	  transform_expiration,
	  "camera_init", "camera_last");
      tf_caster_.sendTransform(tmp_tf_stamped);
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
  
  tf::TransformListener tf_listen_;
  tf::TransformBroadcaster tf_caster_;
  tf::Stamped<tf::Pose> current_tf_pose;
  tf::Stamped<tf::Pose> last_tf_pose;
  tf::Stamped<tf::Pose> current_last_tf_pose;

  double x_prev;
  double y_prev;

  // disable angle calculation for speed
#if 0
    double yaw_prev;
#endif 

  bool waiting_for_pc2;
  double dist_tot, dist_accum;
  double last_dist_tot;
  bool has_published_once;

  // for window size and step size
  std::vector<ScansHist> history_;
  boost::mutex history_mutex_;
  double window_size_;
  double step_size_;
  double timer_period_;

  size_t begin_hist_index_;
} ;

// Give time to transform look the camera pose
// Carefull the camera frame id is HARDCODED
void PeriodicSnapshotter::getCurrentCameraPose(const ros::Time & t, tf::Stamped<tf::Pose> & pose)
{
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)), t, "camera");
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
  ros::NodeHandle n;
  ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("build_cloud");
  ROS_INFO("Found build_cloud! Starting the snapshotter");
  PeriodicSnapshotter snapshotter;
  ros::spin();
  return 0;
}
