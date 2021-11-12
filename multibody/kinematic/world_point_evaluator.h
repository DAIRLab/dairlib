#pragma once
#include "multibody/kinematic/kinematic_evaluator.h"
#include "systems/controllers/osc/osc_view_frame.h"

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace dairlib {
namespace multibody {

/// Basic contact evaluator for a point on a body w.r.t. the world
template <typename T>
class WorldPointEvaluator : public KinematicEvaluator<T> {
 public:
  /// The basic constructor for WorldPointEvaluator, defined via a rotation
  /// matrix.
  /// @param plant
  /// @param pt_A the contact point on the body
  /// @param frame_A the frame of reference for pt_A
  /// @param rotation The rotation matrix R s.t. the evaluation is R * r_A_world
  ////   (default identity matrix)
  /// @param offset The nominal world location of pt_A (default [0;0;0])
  /// @param active_directions The directions, a subset of {0,1,2} to be
  ///    considered active. These will correspond to rows of the rotation matrix
  ////   Default is {0,1,2}.

  WorldPointEvaluator(
      const drake::multibody::MultibodyPlant<T>& plant,
      const Eigen::Vector3d pt_A, const drake::multibody::Frame<T>& frame_A,
      const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(),
      const Eigen::Vector3d offset = Eigen::Vector3d::Zero(),
      std::vector<int> active_directions = {0, 1, 2});

  /// The same constructor as the above one except for the argument
  /// `view_frame` which is the frame which the Jacobian and JdotV are
  /// expressed in.

  WorldPointEvaluator(
      const drake::multibody::MultibodyPlant<T>& plant,
      const Eigen::Vector3d pt_A, const drake::multibody::Frame<T>& frame_A,
      const systems::controllers::OscViewFrame<T>& view_frame,
      const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(),
      const Eigen::Vector3d offset = Eigen::Vector3d::Zero(),
      std::vector<int> active_directions = {0, 1, 2});

  /// Constructor for WorldPointEvaluator, defined via a normal direction
  /// This will automatically construct an appropriate rotation matrix.
  /// This constructor is primarily used for world contacts, where the normal
  /// can represent the contact normal, and the tangential directions are
  /// defined only w.r.t. the normal.
  /// @param plant
  /// @param pt_A the contact point on the body
  /// @param frame_A the frame of reference for pt_A
  /// @param normal The normal unit vector (must be specified)
  /// @param offset The nominal world location of pt_A
  /// @param xy_active If true, then the tangential directions are active
  ///    If false, they be inactive, and thus still appear in
  ///    the "Full" terms, but not the "Active" values
  ///    Note that this defaults to true.

  WorldPointEvaluator(const drake::multibody::MultibodyPlant<T>& plant,
                      const Eigen::Vector3d pt_A,
                      const drake::multibody::Frame<T>& frame_A,
                      const Eigen::Vector3d normal,
                      const Eigen::Vector3d offset = Eigen::Vector3d::Zero(),
                      bool tangent_active = true);

  drake::VectorX<T> EvalFull(
      const drake::systems::Context<T>& context) const override;

  void EvalFullJacobian(const drake::systems::Context<T>& context,
                        drake::EigenPtr<drake::MatrixX<T>> J) const override;

  drake::VectorX<T> EvalFullJacobianDotTimesV(
      const drake::systems::Context<T>& context) const override;

  using KinematicEvaluator<T>::EvalFullJacobian;
  using KinematicEvaluator<T>::plant;

  std::vector<std::shared_ptr<drake::solvers::Constraint>>
  CreateConicFrictionConstraints() const override;

  std::vector<std::shared_ptr<drake::solvers::Constraint>>
  CreateLinearFrictionConstraints(int num_faces = 8) const override;

  /// Identify this evaluator as frictional, for use when calling
  /// CreateConicFrictionConstraint and CreateLinearFrictionConstraint
  /// The normal direction is always assumed to be at index 2 in this
  /// evaluator's output.
  void set_frictional() { is_frictional_ = true; };

 private:
  const Eigen::Vector3d pt_A_;
  const drake::multibody::Frame<T>& frame_A_;
  const Eigen::Vector3d offset_;
  const drake::math::RotationMatrix<double> rotation_;
  const systems::controllers::OscViewFrame<T>* view_frame_ = nullptr;
  bool is_frictional_ = false;
};

}  // namespace multibody
}  // namespace dairlib
