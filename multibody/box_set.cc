#include "box_set.h"

namespace dairlib::multibody {

using Eigen::Vector3d;

BoxSet::BoxSet(
    const std::vector<std::pair<drake::math::RigidTransformd, Eigen::Vector3d>>& cubes) {
  for (const auto& cube : cubes) {
    fcl::Transform3d transform = cube.first.GetAsIsometry3();
    const auto& dimensions = cube.second;

    // Add each box with its dimensions and transform
    AddBox(dimensions.x(), dimensions.y(), dimensions.z(), transform);
  }
  query_sphere_ = std::make_shared<fcl::Sphere<double>>(0.0);
}

void BoxSet::AddBox(double length, double width, double height,
                    const fcl::Transform3d &transform) {
  auto box_geometry = std::make_shared<fcl::Box<double>>(length, width, height);
  auto collision_object = std::make_shared<fcl::CollisionObject<double>>(
      box_geometry, transform);

  boxes_.push_back(collision_object);
}


std::pair<double, Vector3d> BoxSet::CalcSDF(const Vector3d& p) const {
  if (boxes_.empty()) {
    // If there are no boxes, return zero distance and zero gradient
    return {0.0, Vector3d::Zero()};
  }

  // Initialize with a large positive distance
  double min_distance = std::numeric_limits<double>::max();
  fcl::Vector3d min_gradient = Vector3d::Zero();

  fcl::Transform3d query_transform = fcl::Transform3d::Identity();
  query_transform.translation() = p;

  fcl::CollisionObject<double> query_object(query_sphere_, query_transform);

  // Check distance to each box and find the minimum
  for (const auto& box : boxes_) {
    fcl::DistanceRequest<double> request(
        true, // enable_nearest_points
        true  // enable_signed_distance
    );
    fcl::DistanceResult<double> result;

    fcl::distance(&query_object, box.get(), request, result);
    if (result.min_distance < min_distance) {
      min_distance = result.min_distance;
      min_gradient = (p - result.nearest_points[1]).normalized();
      if (min_distance < 0) {
        min_gradient = -min_gradient;
      }
    }
  }

  return {min_distance, min_gradient};
}

}