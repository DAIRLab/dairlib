#include <memory>
#include <chrono>

#include <gflags/gflags.h>

#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

#include "attic/systems/trajectory_optimization/dircon_util.h"
#include "attic/systems/trajectory_optimization/dircon_position_data.h"
#include "attic/systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "attic/systems/trajectory_optimization/hybrid_dircon.h"
#include "attic/systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/goldilocks_models/file_utils.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::trajectories::PiecewisePolynomial;
using drake::MatrixX;
using std::vector;
using std::shared_ptr;
using std::cout;
using std::endl;

DEFINE_int64(steps, 10, "Number of steps");
DEFINE_double(rate, 1, "Playback rate");
DEFINE_string(file, "data/", "File name to load");

/// Inputs: initial trajectory
/// Outputs: trajectory optimization problem
namespace dairlib {

using systems::trajectory_optimization::HybridDircon;
using systems::trajectory_optimization::DirconDynamicConstraint;
using systems::trajectory_optimization::DirconKinematicConstraint;
using systems::trajectory_optimization::DirconOptions;
using systems::trajectory_optimization::DirconKinConstraintType;

void visualizeGait(std::string file, int steps, double rate) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "PlanarWalkerWithTorso.urdf", drake::multibody::joints::kFixed, &tree);

  // Buildtrajectory optimization
  // int n = tree.get_num_positions();
  // int nu = tree.get_num_actuators();

  int leftLegIdx = tree.FindBodyIndex("left_lower_leg");
  int rightLegIdx = tree.FindBodyIndex("right_lower_leg");

  Vector3d pt;
  pt << 0, 0, -.5;
  bool isXZ = true;

  auto leftFootConstraint = DirconPositionData<double>(tree, leftLegIdx, pt,
                                                       isXZ);
  auto rightFootConstraint = DirconPositionData<double>(tree, rightLegIdx, pt,
                                                        isXZ);

  Vector3d normal;
  normal << 0, 0, 1;
  double mu = 1;
  leftFootConstraint.addFixedNormalFrictionConstraints(normal, mu);
  rightFootConstraint.addFixedNormalFrictionConstraints(normal, mu);

  std::vector<DirconKinematicData<double>*> leftConstraints;
  leftConstraints.push_back(&leftFootConstraint);
  auto leftDataSet = DirconKinematicDataSet<double>(tree, &leftConstraints);

  std::vector<DirconKinematicData<double>*> rightConstraints;
  rightConstraints.push_back(&rightFootConstraint);
  auto rightDataSet = DirconKinematicDataSet<double>(tree, &rightConstraints);

  auto leftOptions = DirconOptions(leftDataSet.countConstraints());
  leftOptions.setConstraintRelative(0, true);

  auto rightOptions = DirconOptions(rightDataSet.countConstraints());
  rightOptions.setConstraintRelative(0, true);

  std::vector<int> timesteps;
  timesteps.push_back(20);
  timesteps.push_back(1);
  std::vector<double> min_dt;
  min_dt.push_back(.01);
  min_dt.push_back(.01);
  std::vector<double> max_dt;
  max_dt.push_back(.3);
  max_dt.push_back(.3);

  int N = 1 - timesteps.size();
  for (uint i = 0; i < timesteps.size(); i++) {
    N += timesteps[i];
  }

  std::vector<DirconKinematicDataSet<double>*> dataset_list;
  dataset_list.push_back(&leftDataSet);
  dataset_list.push_back(&rightDataSet);

  std::vector<DirconOptions> options_list;
  options_list.push_back(leftOptions);
  options_list.push_back(rightOptions);

  auto trajopt = std::make_shared<HybridDircon<double>>(tree, timesteps, min_dt,
                                                        max_dt, dataset_list,
                                                        options_list);

  // read in file and set decision variable values
  MatrixXd z = dairlib::goldilocks_models::readCSV(file);

  // go through solver interface
  auto solver = std::make_shared<drake::solvers::SnoptSolver>();

  drake::solvers::MathematicalProgramResult result;
  result.set_x_val(z);

  drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
      trajopt->ReconstructStateTrajectory(result);

  // MatrixXd transformation = MatrixXd::Identity(2*n);

  // auto ttraj =  transformation*pp_xtraj ;

  // std::cout <<<< std::endl;

  // repeat trajectory
  // std::vector<MatrixXd<Polynomial<double>>> trajectory_matrix;
  std::vector<MatrixX<Polynomial<double>>> trajectory_matrix;
  std::vector<double> breaks;
  breaks.push_back(pp_xtraj.start_time());

  double step_distance = pp_xtraj.scalarValue(pp_xtraj.end_time(), 0) -
                         pp_xtraj.scalarValue(pp_xtraj.start_time(), 0);


  for (int j = 0; j < steps; j++) {
    double t_start = pp_xtraj.end_time() + (j-1)*(pp_xtraj.end_time() - pp_xtraj.start_time());
    int ind_start = (j == 0) ? 0 : 0;
    if (j % 2 == 1) {
      // mirror left right legs
      for (int i = ind_start; i < pp_xtraj.get_number_of_segments(); i++) {
        MatrixX<Polynomial<double>> mat = pp_xtraj.getPolynomialMatrix(i);
        mat(0, 0) += step_distance*j;

        auto tmp = mat(3, 0);
        mat(3, 0) = mat(5, 0);
        mat(5, 0) = tmp;

        tmp = mat(4, 0);
        mat(4, 0) = mat(6, 0);
        mat(6, 0) = tmp;

        tmp = mat(10, 0);
        mat(10, 0) = mat(12, 0);
        mat(12, 0) = tmp;

        tmp = mat(11, 0);
        mat(11, 0) = mat(13, 0);
        mat(13, 0) = tmp;

        trajectory_matrix.push_back(mat);
        breaks.push_back(t_start + pp_xtraj.end_time(i));
      }
    } else {
      for (int i = ind_start; i < pp_xtraj.get_number_of_segments(); i++) {
        MatrixX<Polynomial<double>> mat = pp_xtraj.getPolynomialMatrix(i);
        mat(0, 0) += step_distance*j;
        trajectory_matrix.push_back(mat);
        breaks.push_back(t_start + pp_xtraj.end_time(i));
      }
    }
  }

  auto pp_xtraj_repeated = drake::trajectories::PiecewisePolynomial<double>(
      trajectory_matrix, breaks);

  // visualizer
  drake::lcm::DrakeLcm lcm;
  drake::systems::DiagramBuilder<double> builder;
  auto state_source = builder.AddSystem<drake::systems::TrajectorySource>(
        pp_xtraj_repeated);
  auto publisher = builder.AddSystem<drake::systems::DrakeVisualizer>(tree,
                                                                      &lcm);
  publisher->set_publish_period(1.0 / 60.0);
  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(rate);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj_repeated.end_time());
  }

  return;
}
}


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  dairlib::visualizeGait(FLAGS_file, FLAGS_steps, FLAGS_rate);
}

