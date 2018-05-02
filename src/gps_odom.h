// Copyright 2018 Billy
#ifndef SRC_GPS_ODOM_H_
#define SRC_GPS_ODOM_H_

#include <map>
#include <string>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "src/transform.h"

#include "src/topic_name_code.h"

/*!
 * \brief Encapsulate odometry computation from GPS/IMU
 *
 * Process IMU data and GPS(RTK) with sync first then
 * advance based on GPS clocking. (Although GPS is the
 * main data, use closest distance approximation for
 * fusion).
 */
class GpsOdom {
    explicit GpsOdom(
        const rosbag::View& rosbag_view,
        const std::map<std::string, StringCode>& s_mapStringToStringCode)
        : rosbag_view_(rosbag_view),
          s_mapStringToStringCode_(s_mapStringToStringCode) {}
    ~GpsOdom() {}

    GpsOdom(const GpsOdom&) = delete;
    GpsOdom& operator=(const GpsOdom&) = delete;

   private:
    bool gps_harvest_ = false;
    bool gnss_sampled_ = false;
    bool imu_sampled_ = false;
    ::ros::Time last_stamp_imu_, last_stamp_gnss_;
    bool first_time_ = true;
    cartographer::transform::Rigid3d first_rigid3d_;

    sensor_msgs::Imu::Ptr imu_msg_;
    sensor_msgs::NavSatFix::Ptr gnss_msg_;

    const rosbag::View& rosbag_view_;
    const std::map<std::string, StringCode> s_mapStringToStringCode_;
};

#endif  // SRC_GPS_ODOM_H_
