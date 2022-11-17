#include <drake/systems/framework/diagram_builder.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <iostream>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/rendering/multibody_position_to_geometry_pose.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/solvers/solve.h>
#include <drake/common/yaml/yaml_io.h>
#include "common/find_resource.h"
#include "systems/primitives/subvector_pass_through.h"
#include "examples/Cassie/kinematic_centroidal_mpc/cassie_reference_utils.h"
#include "examples/Cassie/kinematic_centroidal_mpc/simple_models/planar_slip_lifter.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/visualization_utils.h"
#include "examples/Cassie/kinematic_centroidal_mpc/simple_models/planar_slip_constraints.h"

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::geometry::DrakeVisualizer;

int main(int argc, char* argv[]) {
  // Create fix-spring Cassie MBP
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);
  MultibodyPlant<double> plant_vis(0.0);

  Parser parser(&plant);
  Parser parser_vis(&plant_vis, &scene_graph);

  std::string full_name =
      dairlib::FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);
  plant.Finalize();
  plant_vis.Finalize();
  Eigen::VectorXd reference_state = GenerateNominalStand(plant, 0.8, 0.2, false);

  auto context = plant.CreateDefaultContext();

  auto left_toe_pair = dairlib::LeftToeFront(plant);
  auto left_heel_pair = dairlib::LeftToeRear(plant);
  auto right_toe_pair = dairlib::RightToeFront(plant);
  auto right_heel_pair = dairlib::RightToeRear(plant);

  std::vector<int> active_inds{0, 1, 2};

  auto left_toe_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, left_toe_pair.first, left_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);

  auto left_heel_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, left_heel_pair.first, left_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);

  auto right_toe_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, right_toe_pair.first, right_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);

  auto right_heel_eval = dairlib::multibody::WorldPointEvaluator<double>(
      plant, right_heel_pair.first, right_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);

//  auto left_slip_eval = dairlib::multibody::WorldPointEvaluator<double>(
//      plant, {0,0,0}, left_heel_pair.second,
//      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);
//
//  auto right_slip_eval = dairlib::multibody::WorldPointEvaluator<double>(
//      plant, {0,0,0}, right_heel_pair.second,
//      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), active_inds);

  dairlib::multibody::SetPositionsAndVelocitiesIfNew<double>(plant, reference_state, context.get());
//  std::cout<<left_slip_eval.EvalFull(*context)<<std::endl;
  PlanarSlipLifter lifter(plant,
                          context.get(),
                          {left_toe_eval, right_toe_eval},
                          {left_toe_eval, left_heel_eval, right_toe_eval, right_heel_eval},
                          {{0, {0, 1}}, {1, {2, 3}}},
                          reference_state.head(plant.num_positions()),
                          2000,
                          0.5,
                          {0.1, -0.1});

  Eigen::Vector2d slip_com = {0.2,0.7};
  Eigen::Vector2d slip_vel = {0.1,0.3};
  Eigen::Vector4d slip_feet = {0.0,0.0,0.0,0.00};
  Eigen::Vector4d slip_foot_vel = {0.11,0.12,0.15,0.18};

  Eigen::VectorXd slip_state(2+2+4+4);
  slip_state << slip_com,slip_vel,slip_feet,slip_foot_vel;
  Eigen::VectorXd complex_state(6 + 3 + 3 * 3 * 4 + plant.num_positions() + plant.num_velocities());

  auto start = std::chrono::high_resolution_clock::now();
  lifter.Lift(slip_state, &complex_state);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() << std::endl;

  start = std::chrono::high_resolution_clock::now();
  lifter.Lift(slip_state, &complex_state);
  finish = std::chrono::high_resolution_clock::now();
  elapsed = finish - start;
  std::cout << "Solve time 2:" << elapsed.count() << std::endl;

  auto context_2 = plant.CreateDefaultContext();
  auto constraint = PlanarSlipReductionConstraint<double>(plant, context_2.get(), {left_toe_eval, right_toe_eval}, complex_state.size(), 0);

  drake::VectorX<double> error(slip_state.size());
  drake::VectorX<double> input(slip_state.size() + complex_state.size());
  input << slip_state,complex_state;
  constraint.DoEval(input, &error);
  std::cout<<"Max Error in inverse test: "<< error.cwiseAbs().maxCoeff() << std::endl;

  if(true){
    // Build temporary diagram for visualization
    drake::systems::DiagramBuilder<double> builder_ik;
    drake::geometry::SceneGraph<double>& scene_graph_ik = *builder_ik.AddSystem<drake::geometry::SceneGraph>();
    scene_graph_ik.set_name("scene_graph_ik");
    MultibodyPlant<double> plant_ik(0.0);
    Parser parser(&plant_ik, &scene_graph_ik);
    std::string full_name =
        dairlib::FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
    parser.AddModelFromFile(full_name);
    plant_ik.Finalize();

    // Visualize
    const auto& x_const = complex_state.tail(plant.num_positions() + plant.num_velocities());
    drake::trajectories::PiecewisePolynomial<double> pp_xtraj(x_const);

    dairlib::multibody::ConnectTrajectoryVisualizer(&plant_ik, &builder_ik,
                                                    &scene_graph_ik, pp_xtraj);
    auto diagram = builder_ik.Build();
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.1);
    simulator.Initialize();
    simulator.AdvanceTo(0.1);
  }
}

