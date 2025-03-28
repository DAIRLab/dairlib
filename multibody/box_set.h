#pragma once

#include <vector>
#include <utility>
#include <memory>
#include <limits>
#include <fcl/fcl.h>
#include <Eigen/Dense>

#include "drake/math/rigid_transform.h"

namespace dairlib::multibody {
/**
 * A class that holds a collection of boxes and computes the signed distance
 * function (SDF) to the closest box from a given point.
 */
class BoxSet {
 public:

  /**
   * Constructor that initializes the box set with a collection of boxes.
   *
   * @param cubes Vector of pairs, each containing a rigid transform for the
   * box pose and a vector of the box length, width, height
   */
  explicit BoxSet(
      const std::vector<std::pair<drake::math::RigidTransformd, Eigen::Vector3d>>& cubes);

  /**
   * Adds a box to the collection.
   *
   * @param length The length of the box along the x-axis
   * @param width The width of the box along the y-axis
   * @param height The height of the box along the z-axis
   * @param transform The rigid transform representing the position and orientation of the box
   */
  void AddBox(double length, double width, double height,
              const fcl::Transform3d& transform);

  /**
   * Computes the signed distance function (SDF) from a point to the closest box.
   *
   * @param p The query point
   * @return A pair containing the signed distance and its gradient vector.
   *         The signed distance is negative when inside any box, and positive
   *         when outside all boxes. The gradient points in the direction of
   *         the steepest increase in distance.
   */
  std::pair<double, Eigen::Vector3d> CalcSDF(const Eigen::Vector3d& p) const;

  /**
   * @return The number of boxes in the collection
   */
  size_t size() const {
    return boxes_.size();
  }

  bool empty() const {
    return boxes_.empty();
  }

 private:
  // Collection of boxes represented as FCL collision objects
  std::vector<std::shared_ptr<fcl::CollisionObject<double>>> boxes_;
  std::shared_ptr<fcl::Sphere<double>> query_sphere_ = nullptr;

};

}
