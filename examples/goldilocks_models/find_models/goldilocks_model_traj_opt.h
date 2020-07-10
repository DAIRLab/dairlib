#include <chrono>
#include <memory>
#include <string>

#include "common/find_resource.h"
#include "examples/goldilocks_models/find_models/dynamics_constraint.h"
#include "examples/goldilocks_models/find_models/kinematics_constraint.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "solvers/optimization_utils.h"
#include "common/file_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
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

using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;

using drake::VectorX;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::Cost;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib {
namespace goldilocks_models {

using systems::SubvectorPassThrough;
using systems::trajectory_optimization::DirconDynamicConstraint;
using systems::trajectory_optimization::DirconKinConstraintType;
using systems::trajectory_optimization::DirconKinematicConstraint;
using systems::trajectory_optimization::DirconOptions;
using systems::trajectory_optimization::HybridDircon;

class GoldilocksModelTrajOpt {
 public:
  GoldilocksModelTrajOpt(const ReducedOrderModel& rom,
                         std::unique_ptr<HybridDircon<double>> dircon_in,
                         const MultibodyPlant<double>& plant,
                         const std::vector<int>& num_time_samples,
                         bool is_get_nominal, bool is_add_tau_in_cost,
                         int rom_option, int robot_option,
                         double constraint_scale);
  GoldilocksModelTrajOpt(){};

  Eigen::VectorBlock<const VectorXDecisionVariable> reduced_model_input(
      int index, int n_tau) const;

  // Eigen::VectorBlock<const VectorXDecisionVariable> reduced_model_position(
  //     int index, int n_s) const;

  std::unique_ptr<HybridDircon<double>> dircon;

  std::shared_ptr<find_models::DynamicsConstraint> dynamics_constraint_at_head;
  std::vector<Binding<Constraint>> dynamics_constraint_at_head_bindings;
//  std::shared_ptr<find_models::DynamicsConstraint> dynamics_constraint_at_tail;
//  std::vector<Binding<Constraint>> dynamics_constraint_at_tail_bindings;

  std::vector<Binding<Cost>> tau_cost_bindings;

  void ConstructStateCubicSplineInfo(
      const MathematicalProgramResult& result,
      const MultibodyPlant<double>& plant,
      const std::vector<int>& num_time_samples,
      vector<DirconKinematicDataSet<double>*> constraints,
      Eigen::VectorXd* times, Eigen::MatrixXd* states,
      Eigen::MatrixXd* derivatives) const;

 private:
  int num_knots_;
  // VectorXDecisionVariable s_vars_;
  VectorXDecisionVariable tau_vars_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
