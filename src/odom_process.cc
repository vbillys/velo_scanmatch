// Copyright 2018 Billy

#include "src/odom_process.h"
#include "src/gps_odom_process.h"
//#include "src/odom_calc.h"
#include <ros/console.h>
#include "glog/logging.h"

OdomProcessor::Rigid3d OdomProcessor::Process(const Imu& imu_msg,
                                              const int& enc_l_val,
                                              const int& enc_r_val) {
    Quaterniond current_imu_ori =
        GpsOdomProcessor::ConvertIMUAlphardSpatialDualToENU(imu_msg);
    int l_enc = enc_l_val;
    int r_enc = enc_r_val;

    if (first_time_) {
        first_time_ = false;
        last_imu_ori_ = current_imu_ori;
        l_enc_prev_ = l_enc;
        r_enc_prev_ = r_enc;
        return imuodom_as_rigid3d_;
    }

    if (std::isnan(current_imu_ori.x()) || std::isnan(current_imu_ori.y()) ||
        std::isnan(current_imu_ori.z()) || std::isnan(current_imu_ori.w())) {
        LOG(FATAL) << current_imu_ori.x() << current_imu_ori.y()
                   << current_imu_ori.z() << current_imu_ori.w();
    }
    Quaterniond delta_q = last_imu_ori_.inverse() * current_imu_ori;

    int delta_l_enc = l_enc - l_enc_prev_;
    int delta_r_enc =
        r_enc_prev_ - r_enc;  // Right encoder is in the reverse direction

    // filtering
    if (abs(delta_l_enc) > FILTER_THRESHOLD) {
        printf("delta_l_enc jumped!!! %d\n", delta_l_enc);
        ROS_ERROR("delta_l_enc jumped!!! %d\n", delta_l_enc);
        delta_l_enc = 0;
        delta_r_enc = 0;
    }

    if (abs(delta_r_enc) > FILTER_THRESHOLD) {
        printf("delta_r_enc jumped!!! %d\n", delta_r_enc);
        ROS_ERROR("delta_r_enc jumped!!! %d\n", delta_r_enc);
        delta_r_enc = 0;
        delta_l_enc = 0;
    }

    // convert from encoder ticks to distance
    double l_wd = delta_l_enc * tick_distance_;
    double r_wd = delta_r_enc * tick_distance_;

    // we use Imu for delta_theta
    // Imu z-axis is pointing downwards
    double delta_v = (l_wd + r_wd) / 2;
    cartographer::transform::Rigid3d delta_T(::Eigen::Vector3d(delta_v, 0, 0),
                                             delta_q);
    // update result
    imuodom_as_rigid3d_ = imuodom_as_rigid3d_ * delta_T;

    last_imu_ori_ = current_imu_ori;
    l_enc_prev_ = l_enc;
    r_enc_prev_ = r_enc;
    return sensor_pose_rigid3d_inversed_ * imuodom_as_rigid3d_ *
           sensor_pose_rigid3d_;
}

// curr_time is updated using message.Gettime()
OdomProcessor::Rigid3d OdomProcessor::STKProcess(const STKImu& vns_att_msg, const STKSpd& veh_meas_msg, double curr_time)
{
    // convert vnt_att from RPY to quaterniond
    Quaterniond current_imu_ori = cartographer::transform::RollPitchYaw(vns_att_msg.vector.x, vns_att_msg.vector.y, vns_att_msg.vector.z);

    if(first_time_) {
        first_time_ = false;
        last_imu_ori_ = current_imu_ori;
        last_veh_time_ = curr_time;

        return imuodom_as_rigid3d_;
    }

    if (std::isnan(current_imu_ori.x()) || std::isnan(current_imu_ori.y()) ||
        std::isnan(current_imu_ori.z()) || std::isnan(current_imu_ori.w())) {
        LOG(FATAL) << current_imu_ori.x() << current_imu_ori.y()
                   << current_imu_ori.z() << current_imu_ori.w();
    }

    Quaterniond delta_q = last_imu_ori_.inverse() * current_imu_ori;


    double delta_time = curr_time - last_veh_time_;
    double delta_v = veh_meas_msg.speed * delta_time;


    cartographer::transform::Rigid3d delta_T(::Eigen::Vector3d(delta_v, 0, 0),
                                             delta_q);

    imuodom_as_rigid3d_ = imuodom_as_rigid3d_ * delta_T;

    // update ori and time for next run
    last_imu_ori_ = current_imu_ori;
    last_veh_time_ = curr_time;

    return sensor_pose_rigid3d_inversed_ * imuodom_as_rigid3d_ *
            sensor_pose_rigid3d_;

}