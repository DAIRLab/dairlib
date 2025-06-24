#include "multibody/kinematic/world_point_evaluator.h"

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
WorldPointEvaluator<T>::WorldPointEvaluator(const MultibodyPlant<T>& plant,
                                            const Vector3d& pt_A,
                                            const Frame<T>& frame_A,
                                            const Matrix3d& R_GW,
                                            const Vector3d& offset,
                                            std::vector<int> active_directions)
    : KinematicEvaluator<T>(plant, 3),
      pt_A_(pt_A),
      frame_A_(frame_A),
      offset_(offset),
      R_WB_(R_GW.transpose()) {
  this->set_active_inds(active_directions);
}

template <typename T>
WorldPointEvaluator<T>::WorldPointEvaluator(
    const MultibodyPlant<T>& plant, const Vector3d& pt_A,
    const Frame<T>& frame_A, const multibody::ViewFrame<T>& view_frame,
    const Matrix3d& R_GW, const Vector3d& offset,
    std::vector<int> active_directions)
    : KinematicEvaluator<T>(plant, 3),
      pt_A_(pt_A),
      frame_A_(frame_A),
      offset_(offset),
      R_WB_(R_GW.transpose()),
      view_frame_(&view_frame) {
  this->set_active_inds(active_directions);
}

template <typename T>
WorldPointEvaluator<T>::WorldPointEvaluator(const MultibodyPlant<T>& plant,
                                            const Vector3d& pt_A,
                                            const Frame<T>& frame_A,
                                            const Vector3d& normal,
                                            const Vector3d& offset,
                                            bool tangent_active)
    : KinematicEvaluator<T>(plant, 3),
      pt_A_(pt_A),
      frame_A_(frame_A),
      offset_(offset),
      R_WB_(RotationMatrix<double>::MakeFromOneVector(normal, 2).transpose()) {
  if (!tangent_active) {
    this->set_active_inds({2});  // only z is active
  }
}

template <typename T>
VectorX<T> WorldPointEvaluator<T>::EvalFull(const Context<T>& context) const {
  VectorX<T> pt_world(3);
  const drake::multibody::Frame<T>& world = plant().world_frame();

  plant().CalcPointsPositions(context, frame_A_, pt_A_.template cast<T>(),
                              world, &pt_world);

  return R_WB_ * (pt_world - offset_);
}

template <typename T>
void WorldPointEvaluator<T>::EvalFullJacobian(
    const Context<T>& context, drake::EigenPtr<MatrixX<T>> J) const {
  const drake::multibody::Frame<T>& world = plant().world_frame();

  // .template cast<T> converts pt_A_, as a double, into type T
  plant().CalcJacobianTranslationalVelocity(
      context, drake::multibody::JacobianWrtVariable::kV, frame_A_,
      pt_A_.template cast<T>(), world, world, J);

  if (view_frame_ == nullptr) {
    *J = R_WB_ * (*J);
  } else {
    *J = view_frame_->CalcWorldToFrameRotation(plant(), context) *
         (R_WB_ * (*J));
  }
}

template <typename T>
VectorX<T> WorldPointEvaluator<T>::EvalFullJacobianDotTimesV(
    const Context<T>& context) const {
  const drake::multibody::Frame<T>& world = plant().world_frame();

  MatrixX<T> Jdot_times_V = plant().CalcBiasTranslationalAcceleration(
      context, drake::multibody::JacobianWrtVariable::kV, frame_A_,
      pt_A_.template cast<T>(), world, world);

  if (view_frame_ == nullptr) {
    return R_WB_ * Jdot_times_V;
  } else {
    return view_frame_->CalcWorldToFrameRotation(plant(), context) *
           (R_WB_ * Jdot_times_V);
  }
}

template <typename T>
vector<shared_ptr<Constraint>>
WorldPointEvaluator<T>::CreateConicFrictionConstraints() const {
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
WorldPointEvaluator<T>::CreateLinearFrictionConstraints(int num_faces) const {
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
    class ::dairlib::multibody::WorldPointEvaluator)

}  // namespace multibody
}  // namespace dairlib