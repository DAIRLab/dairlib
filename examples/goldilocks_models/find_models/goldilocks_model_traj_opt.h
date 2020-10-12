#include <chrono>
#include <memory>
#include <string>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "examples/goldilocks_models/find_models/dynamics_constraint.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "solvers/optimization_utils.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "drake/geometry/geometry_visualization.h"
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

class GoldilocksModelTrajOpt {
 public:
  GoldilocksModelTrajOpt(
      const ReducedOrderModel& rom,
      std::unique_ptr<systems::trajectory_optimization::HybridDircon<double>>
          dircon_in,
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<int>& num_time_samples,
      std::vector<DirconKinematicDataSet<double>*> constraints,
      bool is_get_nominal, const InnerLoopSetting& setting, int rom_option,
      int robot_option, double constraint_scale);
  GoldilocksModelTrajOpt(){};

  Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  reduced_model_input(int index, int n_tau) const;

  // Eigen::VectorBlock<const VectorXDecisionVariable> reduced_model_position(
  //     int index, int n_s) const;

  std::unique_ptr<systems::trajectory_optimization::HybridDircon<double>>
      dircon;

  // Version 1 of dynamics constraints
  std::shared_ptr<find_models::DynamicsConstraint> dynamics_constraint_at_head;
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>>
      dynamics_constraint_at_head_bindings;
  std::shared_ptr<find_models::DynamicsConstraint> dynamics_constraint_at_tail;
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>>
      dynamics_constraint_at_tail_bindings;

  // Version 2 of dynamics constraints
  std::vector<std::shared_ptr<find_models::DynamicsConstraintV2>>
      dynamics_constraint_at_knot;
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>>
      dynamics_constraint_at_knot_bindings;

  std::vector<drake::solvers::Binding<drake::solvers::Cost>> tau_cost_bindings;

  void ConstructStateCubicSplineInfo(
      const drake::solvers::MathematicalProgramResult& result,
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<int>& num_time_samples,
      std::vector<DirconKinematicDataSet<double>*> constraints,
      Eigen::VectorXd* times, Eigen::MatrixXd* states,
      Eigen::MatrixXd* derivatives) const;

 private:
  int num_knots_;
  // VectorXDecisionVariable s_vars_;
  drake::solvers::VectorXDecisionVariable tau_vars_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
