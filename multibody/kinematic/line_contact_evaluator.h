#pragma once
#include "multibody/kinematic/kinematic_evaluator.h"
#include "multibody/view_frame.h"

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace dairlib {
namespace multibody {

/// Basic contact evaluator for a line contact on a body w.r.t. the world
template <typename T>
class LineContactEvaluator : public KinematicEvaluator<T> {
 public:
  /// The basic constructor for LineContactEvaluator, defined via a rotation
  /// matrix.
  /// @param plant
  /// @param pt_A the contact point on the body
  /// @param frame_A the frame of reference for pt_A
  /// @param rotation The rotation matrix R which maps vectors in the line frame
  /// to vectors in frame_A
  ////   (default identity matrix)
  /// @param offset The nominal world location of pt_A (default [0;0;0])
  /// @param active_directions The directions, a subset of {0,1,2} to be
  ///    considered active. These will correspond to rows of the rotation matrix
  ////   Default is {0,1,2}.

  LineContactEvaluator(
      const drake::multibody::MultibodyPlant<T>& plant,
      const Eigen::Vector3d pt_A, const drake::multibody::Frame<T>& frame_A,
      double contact_len,
      const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(),
      const Eigen::Vector3d offset = Eigen::Vector3d::Zero(),
      std::vector<int> active_directions = {0, 1, 2, 3, 4});

  /// The same constructor as the above one except for the argument
  /// `view_frame`.
  /// `WorldPointEvaluator` computes position, Jacobian and JdotV in the world
  /// frame, and expresses them in `ViewFrame` (i.e. rotates the vectors and
  /// matrix).

  LineContactEvaluator(
      const drake::multibody::MultibodyPlant<T>& plant,
      const Eigen::Vector3d pt_A, const drake::multibody::Frame<T>& frame_A,
      double contact_len, const multibody::ViewFrame<T>& view_frame,
      const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(),
      const Eigen::Vector3d offset = Eigen::Vector3d::Zero(),
      std::vector<int> active_directions = {0, 1, 2});


  drake::VectorX<T> EvalFull(
      const drake::systems::Context<T>& context) const override;

  void EvalFullJacobian(const drake::systems::Context<T>& context,
                        drake::EigenPtr<drake::MatrixX<T>> J) const override;

  drake::VectorX<T> EvalFullJacobianDotTimesV(
      const drake::systems::Context<T>& context) const override;

  using KinematicEvaluator<T>::EvalFullJacobian;
  using KinematicEvaluator<T>::plant;

  std::vector<std::shared_ptr<drake::solvers::Constraint>>
  CreateLinearFrictionConstraints(int num_faces = 4) const override;

  /// Identify this evaluator as frictional, for use when calling
  /// CreateConicFrictionConstraint and CreateLinearFrictionConstraint
  /// The normal direction is always assumed to be at index 2 in this
  /// evaluator's output.
  void set_frictional() { is_frictional_ = true; };

 private:
  const Eigen::Vector3d pt_A_;
  const drake::multibody::Frame<T>& frame_A_;
  const Eigen::Vector3d offset_;
  const double contact_half_len_;
  const drake::math::RotationMatrix<double> R_fl_;
  const multibody::ViewFrame<T>* view_frame_ = nullptr;
  bool is_frictional_ = false;
};

}  // namespace multibody
}  // namespace dairlib
