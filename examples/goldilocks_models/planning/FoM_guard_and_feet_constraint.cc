#include "examples/goldilocks_models/planning/FoM_guard_and_feet_constraint.h"

using std::cout;
using std::endl;
using std::isinf;
using std::isnan;
using std::list;
using std::make_shared;
using std::make_unique;
using std::map;
using std::string;
using std::unique_ptr;
using std::vector;

using Eigen::AutoDiffScalar;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::VectorX;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::math::DiscardGradient;
using drake::math::initializeAutoDiff;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::to_string;
using drake::solvers::VariableRefList;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;

namespace dairlib {
namespace goldilocks_models {
namespace planning {

/// Guard constraint
FomGuardConstraint::FomGuardConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const vector<std::pair<const Vector3d, const Frame<double>&>>&
        swing_foot_contacts,
    const VectorXd& lb, const VectorXd& ub, const std::string& description)
    : NonlinearConstraint<double>(
          2 * swing_foot_contacts.size(),
          plant.num_positions() + plant.num_velocities(), lb, ub, description),
      plant_(plant),
      world_(plant.world_frame()),
      context_(plant.CreateDefaultContext()),
      swing_foot_contacts_(swing_foot_contacts) {}

void FomGuardConstraint::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& x, VectorX<double>* y) const {
  plant_.SetPositions(context_.get(), x.head(plant_.num_positions()));

  drake::VectorX<double> pt(3);
  drake::MatrixX<double> J(3, plant_.num_velocities());

  *y = VectorX<double>(2 * swing_foot_contacts_.size());
  for (int i = 0; i < swing_foot_contacts_.size(); i++) {
    const auto& contact = swing_foot_contacts_.at(i);
    // fill in position
    this->plant_.CalcPointsPositions(*context_, contact.second, contact.first,
                                     world_, &pt);
    y->segment<1>(2 * i) = pt.tail<1>();

    // fill in velocity
    plant_.CalcJacobianTranslationalVelocity(
        *context_, drake::multibody::JacobianWrtVariable::kV, contact.second,
        contact.first, world_, world_, &J);
    y->segment<1>(2 * i + 1) =
        J.bottomRows<1>() * x.tail(plant_.num_velocities());
  }
}

/// Swing foot position constraint
FomSwingFootPosConstraint::FomSwingFootPosConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const Frame<double>& pelvis_frame,
    const std::vector<std::pair<const Vector3d, const Frame<double>&>>&
        stance_foot_contacts,
    const std::pair<const Vector3d, const Frame<double>&>& swing_foot_origin,
    const Eigen::Vector3d& lb, const Eigen::Vector3d& ub,
    const std::string& description)
    : NonlinearConstraint<double>(3, plant.num_positions(), lb, ub,
                                  description),
      plant_(plant),
      world_(plant.world_frame()),
      context_(plant.CreateDefaultContext()),
      pelvis_frame_(pelvis_frame),
      stance_foot_contacts_(stance_foot_contacts),
      swing_foot_origin_(swing_foot_origin),
      n_q_(plant.num_positions()) {
  toe_length_ =
      (stance_foot_contacts.at(0).first - stance_foot_contacts.at(1).first)
          .norm();

  // Rear contact should be the first element. We check the value here
  DRAKE_DEMAND(stance_foot_contacts.at(0).first(0) == 0.088);
}

void FomSwingFootPosConstraint::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& q, VectorX<double>* y) const {
  plant_.SetPositions(context_.get(), q);

  // Swing foot position
  drake::VectorX<double> swing_pt(3);
  this->plant_.CalcPointsPositions(*context_, swing_foot_origin_.second,
                                   swing_foot_origin_.first, world_, &swing_pt);
  // Pelvis pose
  auto pelvis_pose = pelvis_frame_.CalcPoseInWorld(*context_);
  const Vector3d& pelvis_pos = pelvis_pose.translation();
  const auto& pelvis_rot_mat = pelvis_pose.rotation();
  // Stance foot position
  drake::VectorX<double> stance_pt1(3);
  this->plant_.CalcPointsPositions(
      *context_, stance_foot_contacts_.at(0).second,
      stance_foot_contacts_.at(0).first, world_, &stance_pt1);
  drake::VectorX<double> stance_pt2(3);
  this->plant_.CalcPointsPositions(
      *context_, stance_foot_contacts_.at(1).second,
      stance_foot_contacts_.at(1).first, world_, &stance_pt2);

  // Get foot pos wrt pelvis
  // Option1: use pelvis frame
  Vector3d foot_pos_in_local_frame =
      pelvis_rot_mat.transpose() * (swing_pt - pelvis_pos);
  // Option2: use pelvis's x axis to construct a 2D frame.
  /*Vector2d normalized_pelvis_x =
      pelvis_rot_mat.matrix().topLeftCorner<2, 1>().normalized();
  Vector2d normalized_pelvis_y(-normalized_pelvis_x(1), normalized_pelvis_x(0));
  MatrixXd rot_mat(2, 2);
  rot_mat << normalized_pelvis_x, normalized_pelvis_y;
  Vector2d foot_pos_in_local_frame =
      rot_mat.transpose() * (swing_pt - pelvis_pos);*/

  // Get foot distance wrt toe (on xy plane)
  Vector3d vec_a = stance_pt2 - stance_pt1;
  Vector3d vec_b = swing_pt - stance_pt1;
  vec_b(2) = 0;
  Vector3d a_cross_b = vec_a.cross(vec_b);
  double signed_distance = a_cross_b.norm() / toe_length_;
  if (a_cross_b(2) < 0) {
    signed_distance *= -1;
  }

  // Assign
  *y = VectorX<double>(3);
  *y << foot_pos_in_local_frame.head<2>(), signed_distance;
}

/// Swing foot distance constraint
FomSwingFootDistanceConstraint::FomSwingFootDistanceConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::pair<const Vector3d, const Frame<double>&>& swing_foot_origin,
    const Eigen::Vector3d& swing_foot_init_pos, double distance,
    bool constant_start_pose, const std::string& description)
    : NonlinearConstraint<double>(
          1,
          constant_start_pose ? plant.num_positions()
                              : 2 * plant.num_positions(),
          VectorXd::Zero(1), distance * VectorXd::Ones(1), description),
      plant_(plant),
      world_(plant.world_frame()),
      context_(plant.CreateDefaultContext()),
      swing_foot_origin_(swing_foot_origin),
      swing_foot_init_pos_(swing_foot_init_pos),
      constant_start_pose_(constant_start_pose),
      n_q_(plant.num_positions()) {}

void FomSwingFootDistanceConstraint::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& x, VectorX<double>* y) const {
  drake::VectorX<double> pt_f(3);
  plant_.SetPositions(context_.get(), x.tail(n_q_));
  this->plant_.CalcPointsPositions(*context_, swing_foot_origin_.second,
                                   swing_foot_origin_.first, world_, &pt_f);

  drake::VectorX<double> pt_0(3);
  if (constant_start_pose_) {
    pt_0 = swing_foot_init_pos_;
  } else {
    plant_.SetPositions(context_.get(), x.head(n_q_));
    this->plant_.CalcPointsPositions(*context_, swing_foot_origin_.second,
                                     swing_foot_origin_.first, world_, &pt_0);
  }

  *y = VectorX<double>(1);
  y->head<1>() << (pt_f - pt_0).head<2>().norm();
}

/// Step length constraint
FomStepLengthConstraint::FomStepLengthConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::pair<const Vector3d, const Frame<double>&>& stance_foot_origin,
    const std::pair<const Vector3d, const Frame<double>&>& swing_foot_origin,
    const Eigen::Vector3d& stance_foot_init_pos, double distance,
    bool constant_start_pose, const std::string& description)
    : NonlinearConstraint<double>(
          1,
          constant_start_pose ? plant.num_positions()
                              : 2 * plant.num_positions(),
          VectorXd::Zero(1), distance * VectorXd::Ones(1), description),
      plant_(plant),
      world_(plant.world_frame()),
      context_(plant.CreateDefaultContext()),
      stance_foot_origin_(stance_foot_origin),
      swing_foot_origin_(swing_foot_origin),
      stance_foot_init_pos_(stance_foot_init_pos),
      constant_start_pose_(constant_start_pose),
      n_q_(plant.num_positions()) {}

void FomStepLengthConstraint::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& x, VectorX<double>* y) const {
  drake::VectorX<double> pt_f(3);
  plant_.SetPositions(context_.get(), x.tail(n_q_));
  this->plant_.CalcPointsPositions(*context_, swing_foot_origin_.second,
                                   swing_foot_origin_.first, world_, &pt_f);

  drake::VectorX<double> pt_0(3);
  if (constant_start_pose_) {
    pt_0 = stance_foot_init_pos_;
  } else {
    plant_.SetPositions(context_.get(), x.head(n_q_));
    this->plant_.CalcPointsPositions(*context_, stance_foot_origin_.second,
                                     stance_foot_origin_.first, world_, &pt_0);
  }

  *y = VectorX<double>(1);
  y->head<1>() << (pt_f - pt_0).norm();
}

/// Constraint for velocity one step after horizon
OneStepAheadVelConstraint::OneStepAheadVelConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::pair<const Vector3d, const Frame<double>&>& stance_foot_origin,
    double stride_period, const std::string& description)
    : NonlinearConstraint<double>(
          2, plant.num_positions() + plant.num_velocities() + 2,
          Vector2d::Zero(), Vector2d::Zero(), description),
      plant_(plant),
      world_(plant.world_frame()),
      context_(plant.CreateDefaultContext()),
      stance_foot_origin_(stance_foot_origin),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()),
      n_x_(plant.num_positions() + plant.num_velocities()) {
  double height = 0.85;  // approximation
  double omega = std::sqrt(9.81 / height);
  omega_sinh_ = omega * std::sinh(omega * stride_period);
  cosh_ = std::cosh(omega * stride_period);
}

void OneStepAheadVelConstraint::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& x_and_com_vel,
    VectorX<double>* y) const {
  plant_.SetPositions(context_.get(), x_and_com_vel.head(n_q_));

  // Stance foot position (I use toe origin as approximation)
  drake::VectorX<double> pt(3);
  this->plant_.CalcPointsPositions(*context_, stance_foot_origin_.second,
                                   stance_foot_origin_.first, world_, &pt);
  // COM position and velocity
  Vector2d CoM;
  Vector2d CoM_dot;
  CoM = plant_.CalcCenterOfMassPositionInWorld(*context_).head<2>();
  MatrixX<double> J_com(3, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      *context_, drake::multibody::JacobianWrtVariable::kV, world_, world_,
      &J_com);
  CoM_dot = J_com.topRows<2>() * x_and_com_vel.segment(n_q_, n_v_);

  // Testing -- Use pelvis as a proxy to COM
  //  CoM = x_and_com_vel.segment<2>(4);
  //  CoM_dot = x_and_com_vel.segment<2>(n_q_ + 3);

  // Velocity at the end of mode after horizon. (Given the initial position and
  // velocity, we can get the solution to the LIPM dynamics. Hence, the velocity
  // at the end of step)
  Vector2d v;
  v = (CoM.head<2>() - pt.head<2>()) * omega_sinh_ + CoM_dot.head<2>() * cosh_;

  *y = v - x_and_com_vel.segment<2>(n_x_);
}

/// Constraint for lipm mapping
LastStepLipmMappingConstraint::LastStepLipmMappingConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::pair<const Vector3d, const Frame<double>&>& stance_foot_origin,
    const std::string& description)
    : NonlinearConstraint<double>(
          6, plant.num_positions() + plant.num_velocities() + 6,
          VectorXd::Zero(6), VectorXd::Zero(6), description),
      plant_(plant),
      world_(plant.world_frame()),
      context_(plant.CreateDefaultContext()),
      stance_foot_origin_(stance_foot_origin),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()) {}

void LastStepLipmMappingConstraint::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& x_com_comdot_ft,
    VectorX<double>* y) const {
  plant_.SetPositions(context_.get(), x_com_comdot_ft.head(n_q_));

  // COM position and velocity
  Vector2d CoM;
  Vector2d CoM_dot;
  CoM = plant_.CalcCenterOfMassPositionInWorld(*context_).head<2>();
  MatrixX<double> J_com(3, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      *context_, drake::multibody::JacobianWrtVariable::kV, world_, world_,
      &J_com);
  CoM_dot = J_com.topRows<2>() * x_com_comdot_ft.segment(n_q_, n_v_);

  // Stance foot position (I use toe origin as approximation)
  drake::VectorX<double> pt(3);
  this->plant_.CalcPointsPositions(*context_, stance_foot_origin_.second,
                                   stance_foot_origin_.first, world_, &pt);

  // Assign
  drake::VectorX<double> com_comdot_ft(6);
  com_comdot_ft << CoM, CoM_dot, pt.head<2>();

  *y = x_com_comdot_ft.tail<6>() - com_comdot_ft;
}

/// V2 for swing foot constraint
/// V2 takes swing foot position as decision variable

/// Swing foot position variable constraint
FomSwingFootPosVariableConstraint::FomSwingFootPosVariableConstraint(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::pair<const Vector3d, const Frame<double>&>& swing_foot_origin,
    const std::string& description)
    : NonlinearConstraint<double>(3, plant.num_positions() + 3,
                                  Vector3d::Zero(), Vector3d::Zero(),
                                  description),
      plant_(plant),
      world_(plant.world_frame()),
      context_(plant.CreateDefaultContext()),
      swing_foot_origin_(swing_foot_origin),
      n_q_(plant.num_positions()) {}

void FomSwingFootPosVariableConstraint::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& q_and_ft_pos,
    VectorX<double>* y) const {
  plant_.SetPositions(context_.get(), q_and_ft_pos.head(n_q_));

  // Swing foot position
  drake::VectorX<double> pt(3);
  this->plant_.CalcPointsPositions(*context_, swing_foot_origin_.second,
                                   swing_foot_origin_.first, world_, &pt);

  *y = pt - q_and_ft_pos.segment<3>(n_q_);
}

/// Swing foot position constraint
FomSwingFootPosConstraintV2::FomSwingFootPosConstraintV2(
    const drake::multibody::MultibodyPlant<double>& plant,
    const Frame<double>& pelvis_frame, const Eigen::Vector2d& lb,
    const Eigen::Vector2d& ub, const std::string& description)
    : NonlinearConstraint<double>(2, plant.num_positions() + 3, lb, ub,
                                  description),
      plant_(plant),
      world_(plant.world_frame()),
      context_(plant.CreateDefaultContext()),
      pelvis_frame_(pelvis_frame),
      n_q_(plant.num_positions()) {}

void FomSwingFootPosConstraintV2::EvaluateConstraint(
    const Eigen::Ref<const VectorX<double>>& q_and_ft_pos,
    VectorX<double>* y) const {
  plant_.SetPositions(context_.get(), q_and_ft_pos.head(n_q_));

  // Pelvis pose
  auto pelvis_pose = pelvis_frame_.CalcPoseInWorld(*context_);
  const Vector3d& pelvis_pos = pelvis_pose.translation();
  const auto& pelvis_rot_mat = pelvis_pose.rotation();

  Vector3d foot_pos_in_local_frame =
      pelvis_rot_mat.transpose() * (q_and_ft_pos.segment<3>(n_q_) - pelvis_pos);

  *y = VectorX<double>(2);
  *y = foot_pos_in_local_frame.head<2>();
}

}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
