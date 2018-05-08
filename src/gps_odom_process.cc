// Copyright 2018 Billy

#include "src/gps_odom_process.h"

GpsOdomProcessor::Rigid3d GpsOdomProcessor::Process(const Imu& imu_msg,
                                                    const NavSatFix& gnss_msg) {
    if (b_start_from_zero_) {
        return ProcessStartZero(imu_msg, gnss_msg);
    } else {
        return ProcessArbitrary(imu_msg, gnss_msg);
    }
}

GpsOdomProcessor::Rigid3d GpsOdomProcessor::ProcessArbitrary(
    const Imu& imu_msg, const NavSatFix& gnss_msg) {
    using Quaterniond = ::Eigen::Quaterniond;
    using Vector3d = ::Eigen::Vector3d;

    return geopp_pose_rigid3d_ *
           cartographer::transform::Rigid3d(
               cartographer::transform::Rigid3d::Vector(
                   gnss_msg.longitude, gnss_msg.latitude, gnss_msg.altitude),
               ConvertIMUAlphardSpatialDualToENU(imu_msg)) *
           sensor_pose_rigid3d_;
}

GpsOdomProcessor::Rigid3d GpsOdomProcessor::ProcessStartZero(
    const Imu& imu_msg, const NavSatFix& gnss_msg) {
    using Quaterniond = ::Eigen::Quaterniond;
    using Vector3d = ::Eigen::Vector3d;

    Rigid3d pose_converted = cartographer::transform::Rigid3d(
        cartographer::transform::Rigid3d::Vector(
            gnss_msg.longitude, gnss_msg.latitude, gnss_msg.altitude),
        ConvertIMUAlphardSpatialDualToENU(imu_msg));

    if (first_time_) {
        first_rigid3d_inversed_ =
            (pose_converted * sensor_pose_rigid3d_).inverse();
        first_time_ = false;
        return geopp_pose_rigid3d_; // * sensor_pose_rigid3d_;
    } else {
        return geopp_pose_rigid3d_ * first_rigid3d_inversed_ * pose_converted *
               sensor_pose_rigid3d_;
    }
}
