#pragma once
#include <drake/systems/framework/context.h>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

#include "drake/multibody/plant/multibody_plant.h"
#include "multibody/pinocchio_utils/pinocchio_interface.h"

// TODO: Needs a fixed vs. floating base mechanism
// Move test methods here as self-verification steps
// Needs joint correspondence

namespace dairlib {
namespace multibody {

template <typename T>
class PinocchioPlant : public drake::multibody::MultibodyPlant<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PinocchioPlant);

  PinocchioPlant(double time_step, const std::string& urdf);
  PinocchioPlant(const drake::multibody::MultibodyPlant<double>& plant, const std::string& urdf);


  void FinalizePlant();


  drake::MatrixX<T> GetVelocityMapFromDrakeToPinocchio(
      const drake::VectorX<T>& quat) const;
  drake::MatrixX<double> GetVelocityMapFromPinocchioToDrake(
      const drake::VectorX<double>& quat) const;

  drake::VectorX<T> CalcInverseDynamics(
      const drake::systems::Context<T>& context,
      const drake::VectorX<T>& known_vdot,
      const drake::multibody::MultibodyForces<T>& external_forces) const;

  drake::VectorX<T> CalcInverseDynamicsWithGravity(
      const drake::systems::Context<T>& context,
      const drake::VectorX<T>& known_vdot,
      const drake::multibody::MultibodyForces<T>& external_forces) const;

  void CalcJacobianTranslationalVelocity(
      const drake::systems::Context<T>& context,
      drake::multibody::JacobianWrtVariable with_respect_to,
      const drake::multibody::Frame<T>& frame_B,
      const Eigen::Ref<const drake::Matrix3X<T>>& p_BoBi_B,
      const drake::multibody::Frame<T>& frame_A, const
      drake::multibody::Frame<T>& frame_E,
      drake::EigenPtr<drake::MatrixX<T>> Js_v_ABi_E) const override;


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
                           drake::EigenPtr<drake::MatrixX<T>> p_AQi) const;

  drake::Vector3<T> CalcCenterOfMassPositionInWorld(
      const drake::systems::Context<T>& context) const;

  drake::Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const drake::systems::Context<T>& context) const;

  void CalcJacobianCenterOfMassTranslationalVelocity(
      const drake::systems::Context<T>& context,
      drake::multibody::JacobianWrtVariable with_respect_to,
      const drake::multibody::Frame<T>& frame_A,
      const drake::multibody::Frame<T>& frame_E,
      drake::EigenPtr<drake::Matrix3X<T>> J) const;

 private:

  drake::Matrix3<T> skew(const drake::Vector3<T>& v) const;
  drake::Vector6<T> MapVDotToBodyFrame(const drake::VectorX<T>& q,
                                       const drake::VectorX<T>& v,
                                       const drake::VectorX<T>& vdot) const;

  mutable drake::Matrix6X<T> J_work_;

  void DoFinalizePinocchioPlant();

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

  std::unique_ptr<PinocchioInterface> interface_;

  // Maps from pinocchio v to drake q
  Eigen::MatrixXd vq_perm_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> u_perm_;
};
}  // namespace multibody
}  // namespace dairlib
