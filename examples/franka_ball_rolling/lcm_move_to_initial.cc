#include <vector>
#include <math.h>
#include <gflags/gflags.h>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/math/rigid_transform.h>
#include "drake/systems/primitives/trajectory_source.h"


#include "systems/robot_lcm_systems.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_c3.hpp"
#include "multibody/multibody_utils.h"


#include "examples/franka_ball_rolling/parameters/simulate_franka_params.h"
#include "examples/franka_ball_rolling/parameters/heuristic_gait_params.h"


#include "examples/franka_ball_rolling/systems/move_to_initial.h"
#include "systems/framework/lcm_driven_loop.h"

DEFINE_int32(TTL, 0,
             "TTL level for publisher. "
             "Default value is 0.");

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::math::RigidTransform;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::Context;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;

using Eigen::VectorXd;
using Eigen::MatrixXd;

int DoMain(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  SimulateFrankaParams sim_param = drake::yaml::LoadYamlFile<SimulateFrankaParams>(
          "examples/franka_ball_rolling/parameters/simulate_franka_params.yaml");
  HeuristicGaitParams heuristic_param = drake::yaml::LoadYamlFile<HeuristicGaitParams>(
            "examples/franka_ball_rolling/parameters/heuristic_gait_params.yaml");


  drake::lcm::DrakeLcm drake_lcm;
  drake::lcm::DrakeLcm drake_lcm_network("udpm://239.255.76.67:7667?ttl=1");

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  drake::multibody::ModelInstanceIndex franka_index = parser.AddModels(sim_param.franka_model)[0];
  drake::multibody::ModelInstanceIndex ground_index = parser.AddModels(sim_param.ground_model)[0];
  drake::multibody::ModelInstanceIndex end_effector_index = parser.AddModels(sim_param.end_effector_model)[0];
  drake::multibody::ModelInstanceIndex ball_index = parser.AddModels(sim_param.ball_model)[0];
  
  /// Fix base of finger to world
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_F_EE = RigidTransform<double>(sim_param.tool_attachment_frame);
  RigidTransform<double> X_F_G = RigidTransform<double>(sim_param.ground_offset_frame);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"), X_WI);
  plant.WeldFrames(plant.GetFrameByName("panda_link7"), plant.GetFrameByName("end_effector_base"), X_F_EE);
  plant.WeldFrames(plant.GetFrameByName("panda_link0"), plant.GetFrameByName("ground"), X_F_G);
  plant.Finalize();

  DiagramBuilder<double> builder;
  auto franka_state_reciver =builder.AddSystem<systems::RobotOutputReceiver>(plant, franka_index);


  /* -------------------------------------------------------------------------------------------*/
  auto initial_sender = builder.AddSystem<dairlib::systems::MoveToInitial>(sim_param,heuristic_param);
  auto state_force_sender = builder.AddSystem<systems::RobotC3Sender>(14, 9, 6, 9);

  builder.Connect(franka_state_reciver->get_output_port(0), initial_sender->get_input_port(0));

  builder.Connect(initial_sender->get_output_port(), state_force_sender->get_input_port(0));
  /* -------------------------------------------------------------------------------------------*/
  // determine if ttl 0 or 1 should be used for publishing
  drake::lcm::DrakeLcm* pub_lcm;
  if (FLAGS_TTL == 0) {
    std::cout << "Using TTL=0" << std::endl;
    pub_lcm = &drake_lcm;
  }
  else if (FLAGS_TTL == 1) {
    std::cout << "Using TTL=1" << std::endl;
    pub_lcm = &drake_lcm_network;
  }
  auto control_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_c3>(
        "CONTROLLER_INPUT", pub_lcm,
        {drake::systems::TriggerType::kForced}, 0.0));
  builder.Connect(state_force_sender->get_output_port(),
      control_publisher->get_input_port());
  /* -------------------------------------------------------------------------------------------*/

  auto diagram = builder.Build();
//  DrawAndSaveDiagramGraph(*diagram, "examples/franka_ball_rolling/lcm_impedance_controller");
//  DrawAndSaveDiagramGraph(*diagram_contact, "examples/franka_ball_rolling/lcm_impedance_controller_contact");

  auto context_d = diagram->CreateDefaultContext();
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
            &drake_lcm, std::move(diagram), franka_state_reciver, "FRANKA_STATE_ESTIMATE_NEW", true);

  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) { dairlib::DoMain(argc, argv);}
