#include "multibody/kinematic/world_line_evaluator.h"
#include "solvers/constraint_factory.h"

using drake::MatrixX;
using drake::VectorX;
using drake::math::RotationMatrix;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::Constraint;
using drake::systems::Context;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using std::shared_ptr;
using std::vector;

namespace dairlib {
namespace multibody {

template <typename T>
WorldLineEvaluator<T>::WorldLineEvaluator(const MultibodyPlant<T>& plant,
                                            Vector3d pt_A,
                                            const Frame<T>& frame_A,
                                            const Matrix3d rotation,
                                            const Vector3d offset,
                                            std::vector<int> active_directions)
    : KinematicEvaluator<T>(plant, 3),
      pt_A_(pt_A),
      frame_A_(frame_A),
      offset_(offset),
      rotation_(rotation) {
  this->set_active_inds(active_directions);
}

template <typename T>
WorldLineEvaluator<T>::WorldLineEvaluator(
    const MultibodyPlant<T>& plant, Vector3d pt_A, const Frame<T>& frame_A,
    const multibody::ViewFrame<T>& view_frame,
    const Matrix3d rotation, const Vector3d offset,
    std::vector<int> active_directions)
    : KinematicEvaluator<T>(plant, 3),
      pt_A_(pt_A),
      frame_A_(frame_A),
      offset_(offset),
      rotation_(rotation),
      view_frame_(&view_frame) {
  this->set_active_inds(active_directions);
}

template <typename T>
VectorX<T> WorldLineEvaluator<T>::EvalFull(const Context<T>& context) const {
  VectorX<T> pt_world(3);
  const drake::multibody::Frame<T>& world = plant().world_frame();

  plant().CalcPointsPositions(context, frame_A_, pt_A_.template cast<T>(),
                              world, &pt_world);

  return rotation_ * (pt_world - offset_);
}

template <typename T>
void WorldLineEvaluator<T>::EvalFullJacobian(
    const Context<T>& context, drake::EigenPtr<MatrixX<T>> J) const {
  const drake::multibody::Frame<T>& world = plant().world_frame();

  MatrixX<T> J_spatial = MatrixX<T>::Zero(5, plant().num_velocities());
  drake::math::RotationMatrix<T> R_wl =
      frame_A_.CalcPoseInWorld(context).rotation();

  // .template cast<T> converts pt_A_, as a double, into type T
  plant().CalcJacobianSpatialVelocity(
      context, drake::multibody::JacobianWrtVariable::kV, frame_A_,
      pt_A_.template cast<T>(), world, world, &J_spatial);

  J->topRows(3) = J_spatial.bottomRows(3);
  J->bottomRows(2) = ((R_wl.matrix() * rotation_.template cast<T>()).transpose() * J_spatial.topRows(3)).bottomRows(2);


  if (view_frame_ == nullptr) {
    *J = rotation_ * (*J);
  } else {
    *J = view_frame_->CalcWorldToFrameRotation(plant(), context) *
        (rotation_ * (*J));
  }
}

template <typename T>
VectorX<T> WorldLineEvaluator<T>::EvalFullJacobianDotTimesV(
    const Context<T>& context) const {
  const drake::multibody::Frame<T>& world = plant().world_frame();

  MatrixX<T> Jdot_times_V = plant().CalcBiasTranslationalAcceleration(
      context, drake::multibody::JacobianWrtVariable::kV, frame_A_,
      pt_A_.template cast<T>(), world, world);

  if (view_frame_ == nullptr) {
    return rotation_ * Jdot_times_V;
  } else {
    return view_frame_->CalcWorldToFrameRotation(plant(), context) *
        (rotation_ * Jdot_times_V);
  }
}

template <typename T>
vector<shared_ptr<Constraint>>
WorldLineEvaluator<T>::CreateConicFrictionConstraints() const {
  // The normal index is 2
  vector<shared_ptr<Constraint>> constraints;
  if (is_frictional_) {
    constraints.push_back(
        solvers::CreateConicFrictionConstraint(this->mu(), 2));
    // Include redundant bounding box constraint
    Vector3d lb = Vector3d::Constant(-std::numeric_limits<double>::infinity());
    Vector3d ub = Vector3d::Constant(std::numeric_limits<double>::infinity());
    lb(2) = 0;
    constraints.push_back(std::make_shared<BoundingBoxConstraint>(lb, ub));
  }
  return constraints;
}

template <typename T>
vector<shared_ptr<Constraint>>
WorldLineEvaluator<T>::CreateLinearFrictionConstraints(int num_faces) const {
  vector<shared_ptr<Constraint>> constraints;
  // The normal index is 2
  if (is_frictional_) {
    constraints.push_back(
        solvers::CreateLinearFrictionConstraint(this->mu(), num_faces, 2));
    // Include redundant bounding box constraint
    Vector3d lb = Vector3d::Constant(-std::numeric_limits<double>::infinity());
    Vector3d ub = Vector3d::Constant(std::numeric_limits<double>::infinity());
    lb(2) = 0;
    constraints.push_back(std::make_shared<BoundingBoxConstraint>(lb, ub));
  }
  return constraints;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::WorldLineEvaluator)

}  // namespace multibody
}  // namespace dairlib