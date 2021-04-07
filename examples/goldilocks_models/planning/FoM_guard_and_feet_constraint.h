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
class FomSwingFootPosConstraint : public solvers::NonlinearConstraint<double> {
 public:
  FomSwingFootPosConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::Frame<double>& pelvis_frame,
      const std::pair<const Eigen::Vector3d,
                      const drake::multibody::Frame<double>&>&
          swing_foot_origin,
      const Eigen::Vector2d& lb, const Eigen::Vector2d& ub,
      const std::string& description = "fom_swing_ft_pos_constraint");

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& q,
                          drake::VectorX<double>* y) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const drake::multibody::Frame<double>& pelvis_frame_;
  const std::pair<const Eigen::Vector3d,
                  const drake::multibody::Frame<double>&>& swing_foot_origin_;

  int n_q_;
};

// Swing foot travel distance constraint for the first mode
// Constraints are
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

/// V2 for swing foot constraint
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
