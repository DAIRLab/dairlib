#pragma once
#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

// TODO: Needs a fixed vs. floating base mechanism
// Move test methods here as self-verification steps
// Needs joint correspondance

namespace dairlib {
namespace multibody {
template <typename T>
class PinocchioPlant : public drake::multibody::MultibodyPlant<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PinocchioPlant)

  explicit PinocchioPlant(double time_step, const std::string& urdf);

  void BuildPermutations();

  void Finalize();

  void CalcMassMatrix(const drake::systems::Context<T>& context,
                      drake::EigenPtr<drake::MatrixX<T>> M) const;

  drake::VectorX<T> CalcInverseDynamics(
      const drake::systems::Context<T>& context,
      const drake::VectorX<T>& known_vdot,
      const drake::multibody::MultibodyForces<T>& external_forces) const;

  void CalcCentroidalMomentumAndDerivatives(
      const drake::systems::Context<double> &context,
      const drake::EigenPtr<Eigen::VectorXd>& h,
      const drake::EigenPtr<Eigen::MatrixXd>& A,
      const drake::EigenPtr<Eigen::MatrixXd>& Adot) const;

  //
  // Comparisons against MultibodyPlant
  //

  ::testing::AssertionResult TestMassMatrix(
      const drake::systems::Context<T>& context, double tol = 1e-5) const;

  ::testing::AssertionResult TestInverseDynamics(
      const drake::systems::Context<T>& context,
      const drake::VectorX<T>& known_vdot,
      const drake::multibody::MultibodyForces<T>& external_forces,
      double tol) const;

 private:
  pinocchio::Model pinocchio_model_;
  mutable pinocchio::Data pinocchio_data_;

  // permutation matrices maps from Pinocchio to MBP
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> q_perm_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> v_perm_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> u_perm_;
};
}  // namespace multibody
}  // namespace dairlib
