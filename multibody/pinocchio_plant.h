#pragma once
#include <drake/systems/framework/context.h>

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

  explicit PinocchioPlant<double>(double time_step, const std::string& urdf);
  explicit PinocchioPlant<drake::AutoDiffXd>(const drake::multibody::MultibodyPlant<double>& plant, const std::string& urdf);

  void BuildPermutations();

  void FinalizePlant();

//  drake::VectorX<T> MapPositionFromDrakeToPinocchio(
//      const drake::VectorX<T>& q) const;
  drake::VectorX<double> MapPositionFromDrakeToPinocchio(
      const drake::VectorX<double>& q) const;
  drake::VectorX<T> MapVelocityFromDrakeToPinocchio(
      const drake::VectorX<T>& q, const drake::VectorX<T>& v) const;
  drake::VectorX<T> MapVelocityFromPinocchioToDrake(
      const drake::VectorX<T>& q, const drake::VectorX<T>& v) const;

  drake::MatrixX<T> GetVelocityMapFromDrakeToPinocchio(
      const drake::VectorX<T>& quat) const;
  drake::MatrixX<double> GetVelocityMapFromPinocchioToDrake(
      const drake::VectorX<double>& quat) const;

//  void SetPositionsAndVelocities(const drake::VectorX<T>& q,
//                                 const drake::VectorX<T>& v);

  /**
   * This function updates the pinocchio data struct with forward kinematics.
   * If derivatives are needed, call the function with Derivatives
   * @param q
   * @param v
   */

  void UpdateForwardKinematics(const drake::systems::Context<double>& context);

  /**
   * Computes ForwardKinematics and ComputeJointJacobians
   * @param q
   * @param v
   */
  void UpdateForwardKinematicsDerivatives(
      const drake::systems::Context<drake::AutoDiffXd>& context);

  /**
   * @brief This function updates the pinocchio data struct with centroidal
   * dynamics If derivatives are needed, call the function with Derivatives
   * @param q
   * @param v
   */
  void UpdateCentroidalDynamics(
      const drake::systems::Context<double>& context) const;

  /**
   * @brief: Computes computeCentroidalDynamicsDerivatives
   * Updates: dh_dq, dhdot_dq, dhdot_dv, dhdot_da (dh_dv)
   * Note: acceleration a is set to zero
   * @param q
   * @param v
   */
  void UpdateCentroidalDynamicsDerivatives(
      const drake::systems::Context<drake::AutoDiffXd>& context) const;

  void RightMultiplicationFromDrakeToPinocchio(
      const drake::VectorX<double>& quat,
      drake::EigenPtr<drake::MatrixX<double>> M) const;

  drake::VectorX<T> CalcInverseDynamics(
      const drake::systems::Context<T>& context,
      const drake::VectorX<T>& known_vdot,
      const drake::multibody::MultibodyForces<T>& external_forces) const;

  void CalcMassMatrix(const drake::systems::Context<T>& context,
                      drake::EigenPtr<drake::MatrixX<T>> M) const;

  /**
 *
 * @param context
 * @param frame_B
 * @param p_BQi
 * @param frame_A
 * @param p_AQi
 */
  void CalcPointsPositions(const drake::systems::Context<T>& context,
                           const drake::multibody::Frame<T>& frame_B,
                           const Eigen::Ref<const drake::MatrixX<T>>& p_BQi,
                           const drake::multibody::Frame<T>& frame_A,
                           drake::EigenPtr<drake::MatrixX<T>> p_AQi) const override;

  drake::Vector3<T> CalcCenterOfMassPositionInWorld(
      const drake::systems::Context<T>& context) const override;

  drake::Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const drake::systems::Context<T>& context) const override;

  void CalcJacobianCenterOfMassTranslationalVelocity(
      const drake::systems::Context<T>& context,
      drake::multibody::JacobianWrtVariable with_respect_to,
      const drake::multibody::Frame<T>& frame_A,
      const drake::multibody::Frame<T>& frame_E,
      drake::EigenPtr<drake::Matrix3X<T>> J) const override;

  drake::multibody::SpatialMomentum<T> CalcSpatialMomentumInWorldAboutPoint(
      const drake::systems::Context<T>& context,
      const drake::Vector3<T>& p_WoP_W) const override;

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

 private:
  std::string urdf_;
  bool is_floating_base_;

  pinocchio::Model pinocchio_model_;
  mutable pinocchio::Data pinocchio_data_;
  const pinocchio::ReferenceFrame pinocchio_world_ =
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  const std::vector<std::string> endEffectorIds_;
  std::vector<size_t> endEffectorFrameIds_;

  int n_q_;
  int n_v_;

  // permutation matrices maps from Pinocchio to MBP
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> q_perm_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> v_perm_;
  // Maps from pinocchio v to drake q
  Eigen::MatrixXd vq_perm_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> u_perm_;
};
}  // namespace multibody
}  // namespace dairlib
