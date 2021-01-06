#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>

#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"

#include "examples/Spirit/animate_spirit.h"

DEFINE_double(duration, 3, "The squat duration");

using drake::AutoDiffXd;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.
 
  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph = std::make_unique<SceneGraph<double>>();
  Parser parser(plant.get());
  Parser parser_vis(plant_vis.get(), scene_graph.get());
  std::string full_name =
      dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");

  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);
 
  plant->Finalize();
  plant_vis->Finalize();

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant->num_positions() +
                       plant->num_velocities());

  int nx = plant->num_positions() + plant->num_velocities();
  int nq = plant->num_positions();
  int N = 100; // number of timesteps

  std::vector<MatrixXd> init_x;

  // Initialize state trajectory
  std::vector<double> init_time;
  VectorXd xState(nx);
  xState = Eigen::VectorXd::Zero(plant->num_positions() + plant->num_velocities());
  auto positions_map = dairlib::multibody::makeNameToPositionsMap(*plant);
  auto velocities_map = dairlib::multibody::makeNameToVelocitiesMap(*plant);
  int num_joints = 12;

  //Intialize the quaternion position of the body
  xState(positions_map.at("base_qw")) = 1;
  xState(positions_map.at("base_qx")) = 0;
  xState(positions_map.at("base_qy")) = 0;
  xState(positions_map.at("base_qz")) = 0;

  //Set Joint Velocities
  for (int j = 0; j < num_joints; j++){   
    xState(nq + velocities_map.at( "joint_"+std::to_string(j)+"dot" )) = 0.2;
  }
  

  double time = 0;
  for (int i = 0; i < N; i++) {
    time=i*FLAGS_duration/(N-1);
    init_time.push_back(time);

    // Integrate joint positions based on velocities
    for (int j =0; j < num_joints; j++){
        xState(positions_map.at("joint_"+std::to_string(j))) = xState(nq + velocities_map.at("joint_" + std::to_string(j)+"dot" )) * time;
    }
    
    //Add to knotpoint state matrix
    init_x.push_back(xState);
  }
  auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);

  dairlib::runAnimate( std::move(plant), plant_vis.get(), std::move(scene_graph), init_x_traj);
}

