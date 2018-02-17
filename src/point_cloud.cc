
#include "point_cloud.h"

#include "sensor.pb.h"
#include "transform.h"

namespace cartographer {
namespace sensor {

PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform) {
  PointCloud result;
  result.reserve(point_cloud.size());
  for (const Eigen::Vector3f& point : point_cloud) {
    result.emplace_back(transform * point);
  }
  return result;
}

PointCloud Crop(const PointCloud& point_cloud, const float min_z,
                const float max_z) {
  PointCloud cropped_point_cloud;
  for (const auto& point : point_cloud) {
    if (min_z <= point.z() && point.z() <= max_z) {
      cropped_point_cloud.push_back(point);
    }
  }
  return cropped_point_cloud;
}

}  // namespace sensor
}  // namespace cartographer
