#include "multibody/kinematic/line_contact_evaluator.h"
#include "solvers/constraint_factory.h"

using drake::MatrixX;
using drake::VectorX;
using drake::math::RotationMatrix;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialAcceleration;
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
LineContactEvaluator<T>::LineContactEvaluator(
    const MultibodyPlant<T>& plant, Vector3d pt_A, const Frame<T>& frame_A,
    double contact_len, const Matrix3d rotation, const Vector3d offset,
    std::vector<int> active_directions)
    : KinematicEvaluator<T>(plant, 3),
      contact_half_len_(contact_len/2),
      pt_A_(pt_A),
      frame_A_(frame_A),
      offset_(offset),
      R_fl_(rotation) {
  this->set_active_inds(active_directions);
}

template <typename T>
LineContactEvaluator<T>::LineContactEvaluator(
    const MultibodyPlant<T>& plant, Vector3d pt_A, const Frame<T>& frame_A,
    double contact_len, const multibody::ViewFrame<T>& view_frame,
    const Matrix3d rotation, const Vector3d offset,
    std::vector<int> active_directions)
    : KinematicEvaluator<T>(plant, 3),
      pt_A_(pt_A),
      frame_A_(frame_A),
      contact_half_len_(contact_len/2),
      offset_(offset),
      R_fl_(rotation),
      view_frame_(&view_frame) {
  this->set_active_inds(active_directions);
}

template <typename T>
VectorX<T> LineContactEvaluator<T>::EvalFull(const Context<T>& context) const {
  VectorX<T> xyz_py_world(5);
  const drake::multibody::Frame<T>& world = plant().world_frame();
  auto pose = frame_A_.CalcPoseInWorld(context);
  xyz_py_world.head(3) = pose.translation();
  xyz_py_world.tail(2) =
      drake::math::RollPitchYaw<T>(pose.rotation()).vector().tail(2);
  return xyz_py_world;
}

template <typename T>
void LineContactEvaluator<T>::EvalFullJacobian(
    const Context<T>& context, drake::EigenPtr<MatrixX<T>> J) const {

  const drake::multibody::Frame<T>& world = plant().world_frame();
  MatrixX<T> J_spatial = MatrixX<T>::Zero(6, plant().num_velocities());
  drake::math::RotationMatrix<T> R_wf =
      frame_A_.CalcPoseInWorld(context).rotation();

  // .template cast<T> converts pt_A_, as a double, into type T
  plant().CalcJacobianSpatialVelocity(
      context, drake::multibody::JacobianWrtVariable::kV, frame_A_,
      pt_A_.template cast<T>(), world, world, &J_spatial);

  if (view_frame_ == nullptr) {
    J->topRows(3) = J_spatial.bottomRows(3);
    J->bottomRows(2) = ((R_wf * R_fl_.template cast<T>()).inverse() *
        J_spatial.topRows(3)).bottomRows(2);
  } else {
    auto R_vf_w =  view_frame_->CalcWorldToFrameRotation(plant(), context);
    J->topRows(3) = R_vf_w * J_spatial.bottomRows(3);
    J->bottomRows(2) = ((R_wf * R_fl_.template cast<T>()).inverse() * R_vf_w.transpose() *
            J_spatial.topRows(3)).bottomRows(2);
  }
}

template <typename T>
VectorX<T> LineContactEvaluator<T>::EvalFullJacobianDotTimesV(
    const Context<T>& context) const {
  const drake::multibody::Frame<T>& world = plant().world_frame();

  drake::math::RotationMatrix<T> R_wf =
      frame_A_.CalcPoseInWorld(context).rotation();
  SpatialAcceleration<T> JdotV_spatial = plant().CalcBiasSpatialAcceleration(
      context, drake::multibody::JacobianWrtVariable::kV, frame_A_,
      pt_A_.template cast<T>(), world, frame_A_);

  VectorX<T> JdotV = VectorX<T>::Zero(5);

  if (view_frame_ == nullptr) {
    JdotV.head(3) = R_wf * JdotV_spatial.translational();
    JdotV.tail(2) = ((R_wf * R_fl_.template cast<T>()).inverse() *
        JdotV_spatial.rotational()).tail(2);
  } else {
    auto R_vf_w =  view_frame_->CalcWorldToFrameRotation(plant(), context);
    JdotV.head(3) = R_vf_w * R_wf.matrix() * JdotV_spatial.translational();
    JdotV.tail(2) = ((R_wf * R_fl_.template cast<T>()).inverse() *
        R_vf_w.transpose() * JdotV_spatial.rotational()).tail(2);
  }
  return JdotV;
}


template <typename T>
vector<shared_ptr<Constraint>>
LineContactEvaluator<T>::CreateLinearFrictionConstraints(int num_faces) const {
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
    class ::dairlib::multibody::LineContactEvaluator)

}  // namespace multibody
}  // namespace dairlib