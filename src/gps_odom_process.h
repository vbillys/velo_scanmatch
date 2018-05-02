// Copyright 2018 Billy
#ifndef SRC_GPS_ODOM_PROCESS_H_
#define SRC_GPS_ODOM_PROCESS_H_

#include "src/transform.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

class GpsOdomProcessor {
   public:
    using Rigid3d = cartographer::transform::Rigid3d;
    using Imu = sensor_msgs::Imu;
    using NavSatFix = sensor_msgs::NavSatFix;

    explicit GpsOdomProcessor(const bool& start_at_zero,
                            const Rigid3d& geopp_pose_rigid3d,
                            const Rigid3d& sensor_pose_rigid3d)
        : b_start_from_zero_(start_at_zero),
          geopp_pose_rigid3d_(geopp_pose_rigid3d),
          sensor_pose_rigid3d_(sensor_pose_rigid3d) {}
    ~GpsOdomProcessor() {}

    GpsOdomProcessor(const GpsOdomProcessor&) = delete;
    GpsOdomProcessor& operator=(const GpsOdomProcessor&) = delete;

    /*!
     * \brief main processing stub
     * 
     * calls other Process... with options
     */
    Rigid3d Process(const Imu& imu_msg, const NavSatFix& gnss_msg);

    /*!
     * \brief Convert axes convention 
     * 
     */
    static inline Rigid3d::Quaternion ConvertIMUAlphardSpatialDualToENU(
        const Imu& imu_msg) {
        using Vector3d = ::Eigen::Vector3d;
        using Quaterniond = ::Eigen::Quaterniond;
        Vector3d euler =
            Quaterniond(imu_msg.orientation.w, imu_msg.orientation.x,
                        imu_msg.orientation.y, imu_msg.orientation.z)
                .toRotationMatrix()
                .eulerAngles(2, 1, 0);
        return cartographer::transform::RollPitchYaw(euler[0], -euler[1],
                                                     M_PI / 2 - euler[2]);
    }

   private:
    /*!
     * \brief processing stub
     * 
     * Only can work if these GPS and IMU already synced.
     * Else may cause accuracy issues. THis one NOT start
     * from zero.
     */
    Rigid3d ProcessArbitrary(const Imu& imu_msg, const NavSatFix& gnss_msg);

    /*!
     * \brief processing stub (from zero start)
     * 
     * Only can work if these GPS and IMU already synced.
     * Else may cause accuracy issues. THis one STARTS
     * from zero.
     */
    Rigid3d ProcessStartZero(const Imu& imu_msg, const NavSatFix& gnss_msg);

   private:
    const bool b_start_from_zero_;
    bool first_time_ = true;
    Rigid3d first_rigid3d_inversed_;
    Rigid3d sensor_pose_rigid3d_;
    Rigid3d geopp_pose_rigid3d_;
};

#endif  // SRC_GPS_ODOM_PROCESS_H_
