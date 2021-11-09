#pragma once

#include <string>

#include "multibody/multibody_utils.h"
#include "solvers/nonlinear_constraint.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"

namespace dairlib {
namespace goldilocks_models {
namespace planning {

// Constraints are
//   contact 1 vertical pos = 0
//   contact 1 vertical vel <= 0  ( = 0 if zero impact)
//   contact 2 vertical pos = 0
//   contact 2 vertical vel <= 0  ( = 0 if zero impact)
//   ...
class FomGuardConstraint : public solvers::NonlinearConstraint<double> {
 public:
  FomGuardConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>&
          swing_foot_contacts,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const std::string& description = "fom_guard_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
      swing_foot_contacts_;
};

// Swing foot position distance constraint for collision avoidance
// Constraints are
//    swing foot x and y lies within a rectangle wrt pelvis
//    swing foot y lies in a halfplane wrt stance foot
class FomSwingFootPosConstraint : public solvers::NonlinearConstraint<double> {
 public:
  FomSwingFootPosConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::Frame<double>& pelvis_frame,
      const std::vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>&
          stance_foot_contacts,
      const std::pair<const Eigen::Vector3d,
                      const drake::multibody::Frame<double>&>&
          swing_foot_origin,
      const Eigen::Vector3d& lb, const Eigen::Vector3d& ub,
      const std::string& description = "fom_swing_ft_pos_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& q,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const drake::multibody::Frame<double>& pelvis_frame_;
  const std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
      stance_foot_contacts_;
  const std::pair<const Eigen::Vector3d,
                  const drake::multibody::Frame<double>&>& swing_foot_origin_;

  int n_q_;
  double toe_length_;
};

// Swing foot travel distance constraint
// Constraints are ...
// The constraint should be imposed on 2D translation instead of 3D, because the
// real robot might not step on the ground in time (which makes the constraint
// infeasible).
class FomSwingFootDistanceConstraint
    : public solvers::NonlinearConstraint<double> {
 public:
  FomSwingFootDistanceConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::pair<const Eigen::Vector3d,
                      const drake::multibody::Frame<double>&>&
          swing_foot_origin,
      const Eigen::Vector3d& swing_foot_init_pos, double distance,
      bool constant_start_pose,
      const std::string& description = "fom_swing_ft_dist_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& q,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const std::pair<const Eigen::Vector3d,
                  const drake::multibody::Frame<double>&>& swing_foot_origin_;

  Eigen::Vector3d swing_foot_init_pos_;

  bool constant_start_pose_;

  int n_q_;
};

// Step length constraint (from stance foot to swing foot)
// Constraints are
class FomStepLengthConstraint : public solvers::NonlinearConstraint<double> {
 public:
  FomStepLengthConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::pair<const Eigen::Vector3d,
                      const drake::multibody::Frame<double>&>&
          stance_foot_origin,
      const std::pair<const Eigen::Vector3d,
                      const drake::multibody::Frame<double>&>&
          swing_foot_origin,
      const Eigen::Vector3d& swing_foot_init_pos, double distance,
      bool constant_start_pose,
      const std::string& description = "fom_step_length_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& q,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const std::pair<const Eigen::Vector3d,
                  const drake::multibody::Frame<double>&>& stance_foot_origin_;
  const std::pair<const Eigen::Vector3d,
                  const drake::multibody::Frame<double>&>& swing_foot_origin_;

  Eigen::Vector3d stance_foot_init_pos_;

  bool constant_start_pose_;

  int n_q_;
};

// The constraint that gives the COM velocity at the end of mode AFTER the
// planner's horizon, based on LIPM dynamics (an approximation).
// Constraints are
//      predicted_comdot = f(com, comdot)
//   where f is the LIPM solution, com and comdot are the current position and
//   velocity.
//   predicted_comdot is a 2D decision variable, and com/comdot is a function of
//   robot's state.
class OneStepAheadVelConstraint : public solvers::NonlinearConstraint<double> {
 public:
  OneStepAheadVelConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::pair<const Eigen::Vector3d,
                      const drake::multibody::Frame<double>&>&
          stance_foot_origin,
      double stride_period,
      const std::string& description = "one_step_ahead_vel_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const std::pair<const Eigen::Vector3d,
                  const drake::multibody::Frame<double>&>& stance_foot_origin_;
  int n_q_;
  int n_v_;
  int n_x_;
  double omega_sinh_;
  double cosh_;
};

// The constraint that gives the COM state and stance foot position of the last
// mode (post impact).
// Constraints are
//      [com, comdot, stance_ft_pos] = f(x_post_impact)
//   where f is the mapping function for COM, COM vel and stance foot.
class LastStepLipmMappingConstraint
    : public solvers::NonlinearConstraint<double> {
 public:
  LastStepLipmMappingConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::pair<const Eigen::Vector3d,
                      const drake::multibody::Frame<double>&>&
          stance_foot_origin,
      const std::string& description = "last_step_lipm_mapping_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const std::pair<const Eigen::Vector3d,
                  const drake::multibody::Frame<double>&>& stance_foot_origin_;
  int n_q_;
  int n_v_;
};

/// V2 for swing foot constraint ///////////////////////////////////////////////
/// V2 takes swing foot position as decision variable

// Swing foot position distance constraint for collision avoidance
// Constraints are
class FomSwingFootPosVariableConstraint
    : public solvers::NonlinearConstraint<double> {
 public:
  FomSwingFootPosVariableConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::pair<const Eigen::Vector3d,
                      const drake::multibody::Frame<double>&>&
          swing_foot_origin,
      const std::string& description = "fom_swing_ft_pos_var_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& q,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const std::pair<const Eigen::Vector3d,
                  const drake::multibody::Frame<double>&>& swing_foot_origin_;

  int n_q_;
};

// Swing foot position distance constraint for collision avoidance
// Constraints are
class FomSwingFootPosConstraintV2
    : public solvers::NonlinearConstraint<double> {
 public:
  FomSwingFootPosConstraintV2(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::Frame<double>& pelvis_frame,
      const Eigen::Vector2d& lb, const Eigen::Vector2d& ub,
      const std::string& description = "fom_swing_ft_pos_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& q,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const drake::multibody::Frame<double>& pelvis_frame_;

  int n_q_;
};

}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib
