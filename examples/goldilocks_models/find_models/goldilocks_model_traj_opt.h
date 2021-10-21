#include <chrono>
#include <memory>
#include <string>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "examples/goldilocks_models/find_models/dynamics_constraint.h"
#include "examples/goldilocks_models/find_models/kinematics_constraint.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "solvers/optimization_utils.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace dairlib {
namespace goldilocks_models {

using dairlib::systems::trajectory_optimization::DirconOptions;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::Cost;

class GoldilocksModelTrajOpt
    : public systems::trajectory_optimization::HybridDircon<double> {
 public:
  GoldilocksModelTrajOpt(
      const drake::multibody::MultibodyPlant<double>& plant,
      std::vector<int> num_time_samples, std::vector<double> minimum_timestep,
      std::vector<double> maximum_timestep,
      std::vector<DirconKinematicDataSet<double>*> constraints,
      std::vector<DirconOptions> options, const ReducedOrderModel& rom,
      bool is_get_nominal, const InnerLoopSetting& setting, int rom_option,
      int robot_option, double constraint_scale,
      bool pre_and_post_impact_efforts = false);

  Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  reduced_model_input(int index) const;

  // Eigen::VectorBlock<const VectorXDecisionVariable> reduced_model_position(
  //     int index, int n_s) const;

  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  tau_post_impact_vars_by_mode(int mode) const;
  drake::solvers::VectorXDecisionVariable tau_vars_by_mode(
      int mode, int time_index) const;

  // Version 1 of dynamics constraints
  std::shared_ptr<find_models::DynamicsConstraint> dynamics_constraint_at_head;
  std::vector<Binding<Constraint>> dynamics_constraint_at_head_bindings;
  std::shared_ptr<find_models::DynamicsConstraint> dynamics_constraint_at_tail;
  std::vector<Binding<Constraint>> dynamics_constraint_at_tail_bindings;

  // Version 2 of dynamics constraints
  std::vector<std::shared_ptr<find_models::DynamicsConstraintV2>>
      dynamics_constraint_at_knot;
  std::vector<Binding<Constraint>> dynamics_constraint_at_knot_bindings;

  // Const Kinematics for testing
  std::vector<std::shared_ptr<ConstKinematicsConstraint>>
      cosnt_kinematics_constraint;
  std::vector<Binding<Constraint>> cosnt_kinematics_constraint_bindings;

  // Collections of costs
  std::vector<Binding<Cost>> cost_x_bindings_;
  std::vector<Binding<Cost>> cost_u_bindings_;
  std::vector<Binding<Cost>> cost_collocation_lambda_bindings_;
  std::vector<Binding<Cost>> cost_lambda_x_diff_bindings_;
  std::vector<Binding<Cost>> cost_lambda_diff_bindings_;
  std::vector<Binding<Cost>> cost_pos_diff_bindings_;
  std::vector<Binding<Cost>> cost_vel_diff_bindings_;
  std::vector<Binding<Cost>> cost_u_diff_bindings_;
  std::vector<Binding<Cost>> cost_q_hip_roll_bindings_;
  std::vector<Binding<Cost>> cost_q_hip_yaw_bindings_;
  std::vector<Binding<Cost>> cost_q_quat_xyz_bindings_;
  std::vector<Binding<Cost>> cost_joint_acceleration_bindings_;
  std::vector<Binding<Cost>> cost_regularization_bindings_;

  std::vector<Binding<Cost>> cost_tau_bindings_;

 private:
  int num_knots_;
  int n_tau_;
  // VectorXDecisionVariable s_vars_;
  drake::solvers::VectorXDecisionVariable tau_vars_;
  drake::solvers::VectorXDecisionVariable tau_post_impact_vars_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
