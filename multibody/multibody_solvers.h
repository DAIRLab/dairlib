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

 /// Constraint class to represent fixed points (equilbria)
 /// may be computed with or without any contact information provided.
 /// The constraint solves for the generalized positions (q) (Not the whole
 /// state), control inputs (u) and constraint forces (lambda).
 /// Contact information is provided through the ContactInfo parameter. All
 /// contact information corresponds to point-ground plane contacts.
 /// If no contact information is provided, q, u and lambda are computed for which
 /// v and vdot are zero. In this case, lambda corresponds to the tree position
 /// constraint forces.
 /// If contact information is provided, contact forces are also taken into
 /// account while computing q, u and lambda for which v and vdot are zero. The
 /// solution of q then also makes sure that the required contacts with the ground
 /// plane are made.
 /// Getter/Setter interfaces have not been provided as the Constraint class
 /// objects would ideally be created inside a solver class and not created
 /// outside.
class FixedPointConstraint : public drake::solvers::Constraint {
 public:
  FixedPointConstraint(const RigidBodyTree<double>& tree,
                       ContactInfo contact_info = ContactInfo(),
                       const std::string& description = "");
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q_u_l,
              Eigen::VectorXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q_u_l,
              drake::AutoDiffVecXd* y) const override;
  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q_u_l,
      drake::VectorX<drake::symbolic::Expression>* y) const override;

 private:
  std::unique_ptr<ContactToolkit<drake::AutoDiffXd>> contact_toolkit_;
  const RigidBodyTree<double>& tree_;
  ContactInfo contact_info_;
  const int num_positions_;
  const int num_velocities_;
  const int num_efforts_;
  const int num_position_forces_;
  const int num_forces_;
};

}  // namespace multibody
}  // namespace dairlib
