#include <gflags/gflags.h>
#include <cmath>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>

#include "examples/franka_ball_rolling/systems/c3_state_estimator.h"
#include "examples/franka_ball_rolling/systems/robot_output_passthrough.h"
#include "systems/robot_lcm_systems.h"
#include <drake/multibody/tree/multibody_element.h>
#include <drake/multibody/parsing/parser.h>

#include "systems/framework/lcm_driven_loop.h"
#include "examples/franka_ball_rolling/parameters/c3_state_estimator_params.h"
#include "dairlib/lcmt_robot_output.hpp"

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::math::RigidTransform;


int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  C3StateEstimatorParams state_estimator_param = drake::yaml::LoadYamlFile<C3StateEstimatorParams>(
          "examples/franka_ball_rolling/parameters/c3_state_estimator_params.yaml");

  DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm drake_lcm;

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.AddModelFromFile(state_estimator_param.franka_model);
  parser.AddModelFromFile(state_estimator_param.offset_model);
  parser.AddModelFromFile(state_estimator_param.ground_model);
  parser.AddModelFromFile(state_estimator_param.end_effector_model);
  parser.AddModelFromFile(state_estimator_param.ball_model);


  /// Fix base of finger to world
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE = RigidTransform<double>(state_estimator_param.tool_attachment_frame);
  RigidTransform<double> X_F_G = RigidTransform<double>(state_estimator_param.ground_offset_frame);


  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"), plant.GetFrameByName("end_effector_base"), X_F_EE);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"), plant.GetFrameByName("visual_table_offset"), X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"), plant.GetFrameByName("ground"), X_F_G);

  plant.Finalize();

  /// subscribe to franka state
  auto passthrough = 
    builder.AddSystem<dairlib::systems::RobotOutputPassthrough>(plant);
  
  /// state estimation block
  std::vector<double> p_FIR_values = state_estimator_param.p_FIR_value;
  std::vector<double> v_FIR_values = state_estimator_param.v_FIR_value;

  auto state_estimator = 
    builder.AddSystem<dairlib::systems::C3StateEstimator>(p_FIR_values, v_FIR_values);
  builder.Connect(passthrough->get_output_port(0),
    state_estimator->get_input_port(0));

  /// connect state_susbcriber ball position port
  auto to_ball_position =
          builder.AddSystem<dairlib::systems::TrueBallToEstimatedBall>(
              state_estimator_param.ball_noise_stddev, 1.0/state_estimator_param.estimation_rate);
  builder.Connect(passthrough->get_output_port(0),
      to_ball_position->get_input_port(0));
  builder.Connect(to_ball_position->get_output_port(0),
      state_estimator->get_input_port(1));

  drake::lcm::DrakeLcm* pub_lcm;
  pub_lcm = &drake_lcm;

  auto sender = builder.AddSystem<dairlib::systems::RobotOutputSender>(plant, true);
  auto robot_output_pub = builder.AddSystem(
    LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
      "FRANKA_STATE_ESTIMATE", pub_lcm, 
      {drake::systems::TriggerType::kForced}));
  builder.Connect(state_estimator->get_output_port(0), 
    sender->get_input_port(0));
  builder.Connect(state_estimator->get_output_port(1), 
    sender->get_input_port(1));
  builder.Connect(*sender, *robot_output_pub);  


  auto sys = builder.Build();
  auto sys_context = sys->CreateDefaultContext();
  // dairlib::DrawAndSaveDiagramGraph(*sys, "examples/franka_ball_rolling/diagram_run_state_estimator");

  dairlib::systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &drake_lcm, std::move(sys), passthrough,
      "FRANKA_OUTPUT", true);
  
  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

int main(int argc, char* argv[]) {
  return DoMain(argc, argv);
}
