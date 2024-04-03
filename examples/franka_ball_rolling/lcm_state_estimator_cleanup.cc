#include <gflags/gflags.h>
#include <cmath>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/tree/multibody_element.h>
#include <drake/multibody/parsing/parser.h>

#include "dairlib/lcmt_robot_output.hpp"

#include "examples/franka_ball_rolling/systems/c3_state_estimator.h"
#include "examples/franka_ball_rolling/systems/robot_output_passthrough.h"

#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "examples/franka_ball_rolling/parameters/c3_state_estimator_params.h"

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::math::RigidTransform;


int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  /* -------------------------------------------------------------------------------------------*/
  C3StateEstimatorParams state_estimator_param = drake::yaml::LoadYamlFile<C3StateEstimatorParams>(
          "examples/franka_ball_rolling/parameters/c3_state_estimator_params.yaml");

  DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm drake_lcm;

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.AddModels(state_estimator_param.franka_model);
  parser.AddModels(state_estimator_param.offset_model);
  parser.AddModels(state_estimator_param.ground_model);
  parser.AddModels(state_estimator_param.end_effector_model);
  parser.AddModels(state_estimator_param.ball_model);


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

  /* -------------------------------------------------------------------------------------------*/
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

  /* ------------------------------- old sender publisher block------------------------------------------------*/
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

  /* ------------------------------- new sender publisher block------------------------------------------------*/
  // new sender and publisher, in the end will replace the above block
  // plant_franka is the plant that only loads franka
  // TODO: in the future, use a single plant and model index to simplify all these
  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka);
  parser_franka.AddModels(state_estimator_param.franka_model);
  plant_franka.WeldFrames(plant_franka.world_frame(), plant_franka.GetFrameByName("panda_link0"), X_WI);
  plant_franka.Finalize();

  auto sender_franka = builder.AddSystem<dairlib::systems::RobotOutputSender>(plant_franka, true);
  auto franka_output_pub = builder.AddSystem(
    LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
      "FRANKA_STATE_ESTIMATE_NEW", pub_lcm,
      {drake::systems::TriggerType::kForced}));
  // hard code port index for now
  builder.Connect(state_estimator->get_output_port(2),
                  sender_franka->get_input_port(0));
  builder.Connect(state_estimator->get_output_port(1),
                  sender_franka->get_input_port(1));
  builder.Connect(*sender_franka, *franka_output_pub);

  MultibodyPlant<double> plant_object(0.0);
  Parser parser_object(&plant_object);
  drake::multibody::ModelInstanceIndex object_index =
          parser_object.AddModels(state_estimator_param.ball_model)[0];
  plant_object.Finalize();
  auto sender_object = builder.AddSystem<dairlib::systems::ObjectStateSender>(plant_object, object_index);
  auto object_state_pub = builder.AddSystem(
            LcmPublisherSystem::Make<dairlib::lcmt_object_state>(
                    "BALL_STATE_ESTIMATE_NEW", pub_lcm,
                    {drake::systems::TriggerType::kForced}));
  // hard code port index for now
  builder.Connect(state_estimator->get_output_port(3),
                  sender_object->get_input_port(0));
  builder.Connect(*sender_object, *object_state_pub);


  /* ------------------------------- build diagram ------------------------------------------------*/
  auto diagram = builder.Build();
  diagram->set_name(("Diagram_State_Estimation"));
  auto diagram_context = diagram->CreateDefaultContext();
  dairlib::DrawAndSaveDiagramGraph(*diagram, "examples/franka_ball_rolling/diagram_lcm_state_estimator");

  dairlib::systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
          &drake_lcm, std::move(diagram), passthrough,
          "FRANKA_OUTPUT", true);
  
  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

int main(int argc, char* argv[]) {
  return DoMain(argc, argv);
}