#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include "dataFramework/data_model.hpp"
#include <boost/filesystem.hpp>

//USINGS
using namespace boost::posix_time;

//GLOBALS
data_model model;
ros::Publisher pub;
tf::TransformListener* tflistener;
std::string rootFrame = "odom";
int scanNo =0;
std::string outputModelPath="";
//FUNCTIONS
std::string createFn (int scanNo)
{
    std::string ret ="scan";
    if (scanNo>=0 && scanNo <10)
    {
        ret = ret +"00" + boost::lexical_cast<std::string>(scanNo);
    }
    else
    if (scanNo>=10 && scanNo <100)
    {
        ret = ret +"0" + boost::lexical_cast<std::string>(scanNo);
    }

    else
    if (scanNo>=100)
    {
        ret = ret + boost::lexical_cast<std::string>(scanNo);
    }
    return ret;
}




void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& inputMsg)
{
  sensor_msgs::PointCloud2 msg = *inputMsg;
  ROS_INFO("recived pointcloud, processing");
  ROS_INFO("frame: %s", inputMsg->header.frame_id.c_str());

  std::string target="odom";
  tf::StampedTransform transform;
  bool isOk = tflistener->waitForTransform(target, inputMsg->header.frame_id,inputMsg->header.stamp, ros::Duration(2.5));
  tflistener->lookupTransform("odom", inputMsg->header.frame_id,inputMsg->header.stamp, transform );
  Eigen::Matrix4f matrixf;
  pcl_ros::transformAsMatrix(transform, matrixf);
  if (msg.fields.size()>=3 )
  {
      if (msg.fields[3].name.compare("intesity")==0)
      {
            msg.fields[3].name=std::string("intensity");
       }

  }
  if (isOk)
  {
      std::string pointcloudID = createFn(scanNo++);

      pcl::PointCloud<pcl::PointXYZI> pp;
      pcl::fromROSMsg(msg, pp);
      std::string timeStampISO = boost::posix_time::to_iso_string(inputMsg->header.stamp.toBoost());

      long long int timeStampEPOCH = inputMsg->header.stamp.toSec();



      pcl::io::savePCDFileBinary(outputModelPath+"/"+pointcloudID+".pcd", pp);
      model.setPointcloudName(pointcloudID, pointcloudID+".pcd");
      model.setAffine(pointcloudID,matrixf);
      model.setTimestamp(pointcloudID, inputMsg->header.stamp.toBoost());
      model.saveFile(outputModelPath+"/model.xml");
      ROS_INFO("saved to %s",pointcloudID.c_str());

  }
  else
  {
      ROS_WARN("Cannot get transform :(");
  }

}
int
main (int argc, char** argv)
{


  // Initialize ROS
  ros::init (argc, argv, "expoter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/aggregator/cloud", 10, cloud_cb);
  if (!nh.getParam("oputputPath", outputModelPath))
  {
      outputModelPath="export/";
      ROS_WARN ("Output path is not given saving to %s", outputModelPath.c_str() );
  }
  //add time stamp to path
  outputModelPath += "session"+boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time());

  if(!boost::filesystem::create_directories(outputModelPath))
  {
        ROS_FATAL ("CANNOT CREATE DIRECTORIES!!");
  }
   ROS_INFO ("Data will be saved to  to %s", outputModelPath.c_str() );
  tflistener = new tf::TransformListener;

  // Spin
  while(ros::ok())
  {

    ros::spinOnce();
  }
}
