// Copyright 2018 Billy
#ifndef SRC_ODOM_PROCESS_H_
#define SRC_ODOM_PROCESS_H_

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "src/transform.h"

#define FILTER_THRESHOLD 15000  // 1500

class OdomProcessor {
   public:
    using Rigid3d = cartographer::transform::Rigid3d;
    using Quaterniond = Rigid3d::Quaternion;
    using Imu = sensor_msgs::Imu;

    explicit OdomProcessor(const Rigid3d& geopp_pose_rigid3d,
                           const Rigid3d& sensor_pose_rigid3d)
        : tick_distance_(0.00206),
          geopp_pose_rigid3d_(geopp_pose_rigid3d),
          sensor_pose_rigid3d_(sensor_pose_rigid3d) {}
    ~OdomProcessor() {}

    OdomProcessor(const OdomProcessor&) = delete;
    OdomProcessor& operator=(const OdomProcessor&) = delete;

    /*!
     * \brief main processing stub
     */
    Rigid3d Process(const Imu& imu_msg, const int& left_enc,
                    const int& right_enc);

   private:
    Rigid3d sensor_pose_rigid3d_;
    Rigid3d geopp_pose_rigid3d_;
    bool first_time_ = true;
    int l_enc_prev_;
    int r_enc_prev_;
    Rigid3d imuodom_as_rigid3d_;
    double tick_distance_;
    Quaterniond last_imu_ori_;
};      // OdomProcessor
#endif  // SRC_ODOM_PROCESS_H_
