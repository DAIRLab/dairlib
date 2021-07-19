#pragma once
#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"

#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pinocchio/parsers/urdf.hpp"

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


  //
  // Comparisons against MultibodyPlant
  //

  ::testing::AssertionResult TestMassMatrix(
      const drake::systems::Context<T>& context, double tol = 1e-5) const;

 private:
  pinocchio::Model pinocchio_model_;
  mutable pinocchio::Data pinocchio_data_;

  // permutation matrices maps from Pinocchio to MBP
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> q_perm_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> v_perm_;
};
}  // namespace multibody
}  // namespace dairlib
