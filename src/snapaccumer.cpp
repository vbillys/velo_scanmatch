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


// struct to handle scans history
// in future we can handle scans ourselves
struct ScansHist
{
  tf::Stamped<tf::Pose> pose;
  double d;
};

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
  void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    latest_pc2_time=msg->header.stamp;
    waiting_for_pc2 = false;
  }
public:

  PeriodicSnapshotter(const double & window_size=1.5, const double & step_size=1.5, const double & timer_period=0.5) : window_size_(window_size), step_size_(step_size), timer_period_(timer_period)
  {
    // indicate that we are waiting for first pc2 msg
    waiting_for_pc2 = true;

    // make sure we start at zero
    dist_tot = 0;

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
    dist_tot +=dist;

    // WARNING: HARDCODED Thresh
    if ( step_size_ <= dist_tot ) 
    {
      ROS_INFO("Triggering because translation of: %.2f", dist_tot);
      dist_tot=0;
      has_moved = true;
    }
    else ROS_INFO("trans is : x:%.2f y:%.2f total:%.3f", delta_x, delta_y, dist_tot);


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
      // Populate our service request based on our timer callback times
      AssembleScans2 srv;
      //srv.request.begin = e.last_real;
      srv.request.begin = prev_tref;
      srv.request.end   = e.current_real;

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
	pcl_ros::transformPointCloud( pcl_pc, pcl_pc, last_tf_pose.inverse());
	//pcl_ros::transformPointCloud( pcl_pc, pcl_pc, current_tf_pose.inverse());
	pcl_pc.header.frame_id = "camera_last";
	//pcl_pc.header.frame_id = "camera_init";
	pub_.publish(pcl_pc);
	has_published_once = true;
      }
      else
      {
	ROS_ERROR("Error making service call\n") ;
      }
      
      // we moved pref_tref here as it is still needed for transform to camera
      prev_tref = e.current_real;
      // store previous time and pose for next iteration
      last_tf_pose = current_tf_pose;
    }

    // maintain happy camera_last tf happy
    if (true) //(has_published_once)
    {
      ros::Time transform_expiration = e.current_real + ros::Duration(timer_period_);
      tf::StampedTransform tmp_tf_stamped(last_tf_pose,
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
  
  tf::TransformListener tf_listen_;
  tf::TransformBroadcaster tf_caster_;
  tf::Stamped<tf::Pose> current_tf_pose;
  tf::Stamped<tf::Pose> last_tf_pose;

  double x_prev;
  double y_prev;

  // disable angle calculation for speed
#if 0
  double yaw_prev;
#endif 

  bool waiting_for_pc2;
  double dist_tot;
  bool has_published_once;

  // for window size and step size
  std::vector<ScansHist> history_;
  double window_size_;
  double step_size_;
  double timer_period_;

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
