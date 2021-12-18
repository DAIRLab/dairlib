
#pragma once
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

#include "drake/multibody/plant/multibody_plant.h"

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

  drake::VectorX<T> MapPositionFromDrakeToPinocchio(
      const drake::VectorX<T>& q) const;
  drake::VectorX<T> MapVelocityFromDrakeToPinocchio(
      const drake::VectorX<T>& q, const drake::VectorX<T>& v) const;
  drake::VectorX<T> MapVelocityFromPinocchioToDrake(
      const drake::VectorX<T>& q, const drake::VectorX<T>& v) const;

  drake::MatrixX<T> GetVelocityMapFromDrakeToPinocchio(
      const drake::VectorX<T>& quat) const;
  drake::MatrixX<T> GetVelocityMapFromPinocchioToDrake(
      const drake::VectorX<T>& quat) const;

  void RightMultiplicationFromDrakeToPinocchio(
      const drake::VectorX<T>& quat,
      drake::EigenPtr<drake::MatrixX<T>> M) const;

  drake::VectorX<T> CalcInverseDynamics(
      const drake::systems::Context<T>& context,
      const drake::VectorX<T>& known_vdot,
      const drake::multibody::MultibodyForces<T>& external_forces) const;

  void CalcMassMatrix(const drake::systems::Context<T>& context,
                      drake::EigenPtr<drake::MatrixX<T>> M) const;

  drake::Vector3<T> CalcCenterOfMassPositionInWorld(
      const drake::systems::Context<T>& context) const ;

  drake::Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const drake::systems::Context<T>& context) const;

  void CalcJacobianCenterOfMassTranslationalVelocity(
      const drake::systems::Context<T>& context,
      drake::multibody::JacobianWrtVariable with_respect_to,
      const drake::multibody::Frame<T>& frame_A,
      const drake::multibody::Frame<T>& frame_E,
      drake::EigenPtr<drake::Matrix3X<T>> J) const;

  //
  // Comparisons against MultibodyPlant
  //

  bool TestInverseDynamics(
      const drake::systems::Context<T>& context,
      const drake::VectorX<T>& known_vdot,
      const drake::multibody::MultibodyForces<T>& external_forces,
      double tol) const;

  bool TestMassMatrix(const drake::systems::Context<T>& context,
                      double tol = 1e-5) const;

  bool TestCenterOfMass(const drake::systems::Context<T>& context,
                        double tol = 1e-5) const;

  bool TestCenterOfMassVel(const drake::systems::Context<T>& context,
                           double tol = 1e-5) const;

  bool TestCenterOfMassJ(const drake::systems::Context<T>& context,
                         double tol = 1e-5) const;

  void CalcCentroidalMomentumAndDerivatives(
      const drake::systems::Context<double> &context,
      const drake::EigenPtr<Eigen::VectorXd>& h,
      const drake::EigenPtr<Eigen::MatrixXd>& A,
      const drake::EigenPtr<Eigen::MatrixXd>& Adot) const;

 private:
  std::string urdf_;
  bool is_floating_base_;

  pinocchio::Model pinocchio_model_;
  mutable pinocchio::Data pinocchio_data_;

  // permutation matrices maps from Pinocchio to MBP
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> q_perm_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> v_perm_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> u_perm_;
};
}  // namespace multibody
}  // namespace dairlib
