#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/com_pose_system.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/multibody_utils.h"
#include "solvers/nonlinear_constraint.h"
#include "systems/goldilocks_models/file_utils.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "systems/trajectory_optimization/dircon/dircon_opt_constraints.h"
#include "solvers/optimization_utils.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"


#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

using std::cout;
using std::endl;
using std::map;
using std::shared_ptr;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::VectorX;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::SolutionResult;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::trajectories::PiecewisePolynomial;

using dairlib::goldilocks_models::readCSV;
using dairlib::goldilocks_models::writeCSV;
using dairlib::systems::SubvectorPassThrough;


namespace dairlib {
using systems::trajectory_optimization::Dircon;
using systems::trajectory_optimization::DirconMode;
using systems::trajectory_optimization::CachedAccelerationConstraint;
using systems::trajectory_optimization::DirconCollocationConstraint;
using systems::trajectory_optimization::DirconKinematicConstraint;
using systems::trajectory_optimization::DirconDynamicConstraint;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::HybridDircon;
using dairlib::systems::trajectory_optimization::PointPositionConstraint;


void DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);

  string full_name =
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  plant.Finalize();

  ///
  /// Build new evaluators
  ///

  // Create maps for joints
  map<string, int> positions_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> velocities_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> actuators_map = multibody::makeNameToActuatorsMap(plant);

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();
  // int n_x = n_q + n_v;

  // Set up contact/distance evaluators
  auto left_loop_eval = LeftLoopClosureEvaluator(plant);
  auto right_loop_eval = RightLoopClosureEvaluator(plant);

  auto left_toe_pair = LeftToeFront(plant);
  auto left_heel_pair = LeftToeRear(plant);
  auto right_toe_pair = RightToeFront(plant);
  auto right_heel_pair = RightToeRear(plant);

  std::vector<int> toe_active_inds{0,1,2};
  std::vector<int> heel_active_inds{1,2};

  auto left_toe_eval = multibody::WorldPointEvaluator(plant,
    left_toe_pair.first, left_toe_pair.second, Eigen::Matrix3d::Identity(),
    Eigen::Vector3d::Zero(), toe_active_inds);
  left_toe_eval.SetFrictional();

  auto left_heel_eval = multibody::WorldPointEvaluator(plant,
    left_heel_pair.first, left_heel_pair.second, Eigen::Matrix3d::Identity(),
    Eigen::Vector3d::Zero(), heel_active_inds);
  left_heel_eval.SetFrictional();

  auto right_toe_eval = multibody::WorldPointEvaluator(plant,
    right_toe_pair.first, right_toe_pair.second, Eigen::Matrix3d::Identity(),
    Eigen::Vector3d::Zero(), toe_active_inds);
  right_toe_eval.SetFrictional();

  auto right_heel_eval = multibody::WorldPointEvaluator(plant,
    right_heel_pair.first, right_heel_pair.second, Eigen::Matrix3d::Identity(),
    Eigen::Vector3d::Zero(), heel_active_inds);
  right_heel_eval.SetFrictional();

  auto evaluators = multibody::KinematicEvaluatorSet<double>(plant);
  evaluators.add_evaluator(&left_toe_eval);
  evaluators.add_evaluator(&left_heel_eval);
  evaluators.add_evaluator(&right_toe_eval);
  evaluators.add_evaluator(&right_heel_eval);
  evaluators.add_evaluator(&left_loop_eval);
  evaluators.add_evaluator(&right_loop_eval);

  int num_knotpoints = 20;
  double min_T = .2;
  double max_T = 2;
  double mu = 2;
  auto double_support =
      DirconMode<double>(evaluators, num_knotpoints, min_T, max_T);
  double_support.set_mu(mu);
  // Set x-y coordinates as relative
  double_support.MakeConstraintRelative(0, 0); // left_toe, x
  double_support.MakeConstraintRelative(0, 1); // left_toe, y
  double_support.MakeConstraintRelative(1, 0); // left_heel x
  double_support.MakeConstraintRelative(1, 1); // left_heel, y
  double_support.MakeConstraintRelative(2, 0); // right_toe, x
  double_support.MakeConstraintRelative(2, 1); // right_toe, y
  double_support.MakeConstraintRelative(3, 0); // right_heel, x
  double_support.MakeConstraintRelative(3, 1); // right_heel, y

  auto context = plant.CreateDefaultContext();
  auto context2 = plant.CreateDefaultContext();

  auto phiddot_constraint = CachedAccelerationConstraint<double>(plant, evaluators,
      context.get(), "new accel");

  auto phidot_constraint = multibody::KinematicVelocityConstraint<double>(plant, evaluators,
      context.get(), "new phidot");

  auto phi_constraint = multibody::KinematicPositionConstraint<double>(plant,
      evaluators, Eigen::VectorXd::Zero(evaluators.count_active()),
      Eigen::VectorXd::Zero(evaluators.count_active()),
      double_support.relative_constraints(),
      context.get(), "new phi");

  auto collocation_constraint = DirconCollocationConstraint<double>(
      plant, evaluators, context.get(), context2.get(), 0, 0);

  ///
  /// Build old evaluators
  ///

  // Set up contact/distance constraints
  const Body<double>& toe_left = plant.GetBodyByName("toe_left");
  const Body<double>& toe_right = plant.GetBodyByName("toe_right");
  Vector3d pt_front_contact(-0.0457, 0.112, 0);
  Vector3d pt_rear_contact(0.088, 0, 0);
  bool isXZ = false;
  Vector3d ground_normal(0, 0, 1);
  auto left_toe_front_constraint = DirconPositionData<double>(
      plant, toe_left, pt_front_contact, isXZ, ground_normal);
  auto left_toe_rear_constraint = DirconPositionData<double>(
      plant, toe_left, pt_rear_contact, isXZ, ground_normal);
  auto right_toe_front_constraint = DirconPositionData<double>(
      plant, toe_right, pt_front_contact, isXZ, ground_normal);
  auto right_toe_rear_constraint = DirconPositionData<double>(
      plant, toe_right, pt_rear_contact, isXZ, ground_normal);
  left_toe_front_constraint.addFixedNormalFrictionConstraints(mu);
  left_toe_rear_constraint.addFixedNormalFrictionConstraints(mu);
  right_toe_front_constraint.addFixedNormalFrictionConstraints(mu);
  right_toe_rear_constraint.addFixedNormalFrictionConstraints(mu);

  const auto& thigh_left = plant.GetBodyByName("thigh_left");
  const auto& heel_spring_left = plant.GetBodyByName("heel_spring_left");
  const auto& thigh_right = plant.GetBodyByName("thigh_right");
  const auto& heel_spring_right = plant.GetBodyByName("heel_spring_right");
  double rod_length = 0.5012;  // from cassie_utils
  Vector3d pt_on_heel_spring = Vector3d(.11877, -.01, 0.0);
  Vector3d pt_on_thigh_left = Vector3d(0.0, 0.0, 0.045);
  Vector3d pt_on_thigh_right = Vector3d(0.0, 0.0, -0.045);
  auto distance_constraint_left = DirconDistanceData<double>(
      plant, thigh_left, pt_on_thigh_left, heel_spring_left, pt_on_heel_spring,
      rod_length);
  auto distance_constraint_right = DirconDistanceData<double>(
      plant, thigh_right, pt_on_thigh_right, heel_spring_right,
      pt_on_heel_spring, rod_length);

  // get rid of redundant constraint
  vector<int> skip_constraint_inds;
  skip_constraint_inds.push_back(3);
  skip_constraint_inds.push_back(9);

  // Double stance (all four contact points)
  vector<DirconKinematicData<double>*> double_stance_all_constraint;
  double_stance_all_constraint.push_back(&left_toe_front_constraint);
  double_stance_all_constraint.push_back(&left_toe_rear_constraint);
  double_stance_all_constraint.push_back(&right_toe_front_constraint);
  double_stance_all_constraint.push_back(&right_toe_rear_constraint);
  double_stance_all_constraint.push_back(&distance_constraint_left);
  double_stance_all_constraint.push_back(&distance_constraint_right);
  auto double_all_dataset = DirconKinematicDataSet<double>(
      plant, &double_stance_all_constraint, skip_constraint_inds);
  auto double_all_options =
      DirconOptions(double_all_dataset.countConstraints(), plant);
  // Be careful in setting relative constraint, because we skip constraints
  ///                 || lf    | lr    | rf    | rr      | fourbar
  /// Before skipping || 0 1 2 | 3 4 5 | 6 7 8 | 9 10 11 | 12 13
  /// After skipping  || 0 1 2 |   3 4 | 5 6 7 |   8  9  | 10 11
  double_all_options.setConstraintRelative(0, true);
  double_all_options.setConstraintRelative(1, true);
  double_all_options.setConstraintRelative(3, true);
  double_all_options.setConstraintRelative(5, true);
  double_all_options.setConstraintRelative(6, true);
  double_all_options.setConstraintRelative(8, true);

  auto old_constraint = DirconKinematicConstraint(plant, double_all_dataset,
    double_all_options.getConstraintsRelative());

  auto old_coll_constraint = DirconDynamicConstraint(plant, double_all_dataset,
      true);
  DRAKE_DEMAND(old_coll_constraint.num_vars() == collocation_constraint.num_vars());

  Eigen::VectorXd vars =
      Eigen::VectorXd::Random(n_q + n_v + n_u + evaluators.count_full() + 6);
  drake::AutoDiffVecXd phiddot, y_old, phi, phidot, coll, coll_old;

  auto vars_autodiff = drake::math::initializeAutoDiff(vars);

  drake::AutoDiffVecXd vars_phi(n_q + 6);
  vars_phi << vars_autodiff.head(n_q), vars_autodiff.tail(6);

  phiddot_constraint.DoEval(vars_autodiff.head(
        n_q + n_v + n_u + evaluators.count_full()), &phiddot);
  phi_constraint.DoEval(vars_phi, &phi);
  phidot_constraint.DoEval(vars_autodiff.head(n_q + n_v), &phidot);
  old_constraint.DoEval(vars_autodiff, &y_old);

  std::cout << "Phiddot: " << std::endl <<
      autoDiffToValueMatrix(phiddot - y_old.head(evaluators.count_active()))
      << std::endl;
  std::cout << "Phidot: " << std::endl <<
      autoDiffToValueMatrix(phidot - y_old.segment(evaluators.count_active(),
          evaluators.count_active()))
      << std::endl;
  std::cout << "Phi: " << std::endl <<
      autoDiffToValueMatrix(phi - y_old.tail(evaluators.count_active()))
      << std::endl;

  std::cout << "dPhiddot: " << std::endl <<
      autoDiffToGradientMatrix(phiddot - y_old.head(evaluators.count_active()))
      << std::endl;
  std::cout << "dPhidot: " << std::endl <<
      autoDiffToGradientMatrix(phidot - y_old.segment(evaluators.count_active(),
          evaluators.count_active()))
      << std::endl;
  std::cout << "dPhi: " << std::endl <<
      autoDiffToGradientMatrix(phi - y_old.tail(evaluators.count_active()))
      << std::endl;

  auto vars_collocation = drake::math::initializeAutoDiff(
      Eigen::VectorXd::Random(collocation_constraint.num_vars()));
  collocation_constraint.DoEval(vars_collocation, &coll);
  old_coll_constraint.DoEval(vars_collocation, &coll_old);
  std::cout << "Coll: " << std::endl << coll - coll_old << std::endl;
//   std::cout << "dColl: " << std::endl <<
//       autoDiffToGradientMatrix(coll - coll_old) << std::endl;

  return;
}
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain(argc, argv);
}
