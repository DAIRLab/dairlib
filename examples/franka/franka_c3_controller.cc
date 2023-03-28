
#include <gflags/gflags.h>
#include <dairlib/lcmt_radio_out.hpp>

#include "examples/franka/franka_controller_params.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "lcm/lcm_trajectory.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "examples/franka/systems/end_effector_trajectory.h"

namespace dairlib {

using drake::math::RigidTransform;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;


DEFINE_string(controller_settings, "",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaControllerParams>(
          "examples/franka/franka_controller_params.yaml");
  DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");


  drake::multibody::MultibodyPlant<double> plant_franka(0.0);
  Parser parser(&plant_franka, nullptr);
  parser.AddModelFromFile("examples/franka/urdf/franka_box.urdf");
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_franka.WeldFrames(plant_franka.world_frame(), plant_franka.GetFrameByName("panda_link0"),
                          X_WI);
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto trajectory_sender =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_saved_traj>(
          controller_params.c3_channel,
          &lcm,
          TriggerTypeSet({TriggerType::kForced})));







//  auto controller = builder.AddSystem<systems::controllers::C3Controller_franka>(
//      plant_franka, plant_f, plant_franka, *context,
//      context_f, *context_franka, *plant_ad,
//      *plant_ad_f, *context_ad, *context_ad_f,
//      scene_graph, *diagram_f, contact_geoms,
//      num_friction_directions, mu, Q, R, G, U,
//      xdesired, pp);

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_c3_controller"));
  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(owned_diagram), state_receiver,
      controller_params.state_channel, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }