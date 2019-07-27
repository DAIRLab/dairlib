#pragma once

#include <memory>

#include "drake/solvers/constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "systems/trajectory_optimization/dircon_kinematic_data.h"

namespace dairlib {

/// An implementation of DirconKinematicData to address distance constraints,
/// which specify the distance between two given points on two bodies
/// computes the constraint as distance(q)^2 - d^2 = 0
template <typename T>
class DirconDistanceData : public DirconKinematicData<T> {
 public:
  DirconDistanceData(const drake::multibody::MultibodyPlant<T>& plant,
      const drake::multibody::Body<T>& body1, const Eigen::Vector3d pt1,
      const drake::multibody::Body<T>& body2, const Eigen::Vector3d pt2,
      const double distance);
  ~DirconDistanceData();

  // The workhorse function, updates and caches everything needed by the
  // outside world
  void updateConstraint(const drake::systems::Context<T>& context);


 private:
    const drake::multibody::Body<T>& body1_;
    const drake::multibody::Body<T>& body2_;
    const Eigen::Vector3d pt1_;
    const Eigen::Vector3d pt2_;
    const double distance_;
};
}  // namespace dairlib
