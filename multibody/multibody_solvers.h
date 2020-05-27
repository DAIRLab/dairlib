#pragma once

#include "multibody/contact_toolkit.h"
#include "solvers/nonlinear_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace multibody {


 /// ContactConstraint class for solving for point to plane contacts.
 /// Specifically, the z-plane (ground plane). The constraint assumes that the
 /// given contact information is for point-gound plane contacts and solves it
 /// accordingly.
 /// The constraint solves for the generalized positions (q)
 ///that satisfies the contact constraints.
template <typename T>
class ContactConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the ccontext
  ContactConstraint(const drake::multibody::MultibodyPlant<T>& plant,
      const multibody::ContactInfo<T>& contact_info,
      std::shared_ptr<drake::systems::Context<T>> context,
      const std::string& description = "");

  /// This constructor will build its own shared_ptr<Context>
  ContactConstraint(const drake::multibody::MultibodyPlant<T>& plant,
      const multibody::ContactInfo<T>& contact_info,
      const std::string& description = "");

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const = 0;

 private:
  std::unique_ptr<multibody::ContactToolkit<T>>
      contact_toolkit_;
  const drake::multibody::MultibodyPlant<T>& plant_;
  std::shared_ptr<drake::systems::Context<T>> context_;
};


/// Constraint class for the distance between two points. Resulting constraint
/// is one dimensional
template <typename T>
class DistanceConstraint : public solvers::NonlinearConstraint<T> {
 public:
  /// This constructor takes a shared_ptr<Context> as an argument to share
  /// cached kinematic/dynamic computation within the ccontext
  DistanceConstraint(const drake::multibody::MultibodyPlant<T>& plant,
      const drake::multibody::Frame<T>* frameA, const Eigen::Vector3d& ptA,
      const drake::multibody::Frame<T>* frameB, const Eigen::Vector3d& ptB,
      double distance, std::shared_ptr<drake::systems::Context<T>> context,
      const std::string& description = "");

  /// This constructor will build its own shared_ptr<Context>
  DistanceConstraint(const drake::multibody::MultibodyPlant<T>& plant,
      const drake::multibody::Frame<T>* frameA, const Eigen::Vector3d& ptA,
      const drake::multibody::Frame<T>* frameB, const Eigen::Vector3d& ptB,
      double distance, const std::string& description = "");

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                                  drake::VectorX<T>* y) const = 0;

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const drake::multibody::Frame<T>* frameA_;
  const Eigen::Vector3d& ptA_;
  const drake::multibody::Frame<T>* frameB_;
  const Eigen::Vector3d& ptB_;
  const double distance_;
  std::shared_ptr<drake::systems::Context<T>> context_;
};

}  // namespace multibody
}  // namespace dairlib
