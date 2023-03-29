
#include <dairlib/lcmt_radio_out.hpp>
#include <gflags/gflags.h>

#include "examples/franka/franka_c3_controller_params.h"
#include "examples/franka/systems/end_effector_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "systems/controllers/c3_controller.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using dairlib::solvers::LCSFactory;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;

using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;

DEFINE_string(controller_settings, "",
              "Controller settings such as channels. Attempting to minimize "
              "number of gflags");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // load parameters
  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  FrankaC3ControllerParams controller_params =
      drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(
          "examples/franka/franka_controller_params.yaml");
  C3Options c3_options =
      drake::yaml::LoadYamlFile<C3Options>(controller_params.c3_options_file);

  ///

  MultibodyPlant<double> plant_franka(0.0);
  Parser parser_franka(&plant_franka, nullptr);
  parser_franka.AddModelFromFile("examples/franka/urdf/franka.urdf");
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant_franka.WeldFrames(plant_franka.world_frame(),
                          plant_franka.GetFrameByName("panda_link0"), X_WI);
  plant_franka.Finalize();

  ///
  MultibodyPlant<double> plant_plate(0.0);
  Parser parser_plate(&plant_plate);
  parser_plate.package_map().Add("franka_urdfs", "examples/franka/urdf");
  parser_plate.AddModelFromFile("examples/franka/urdf/plate_end_effector.urdf");
  parser_plate.AddModelFromFile("examples/franka/urdf/tray.sdf");
  plant_plate.Finalize();

  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_plate_ad =
      drake::systems::System<double>::ToAutoDiffXd(plant_plate);

  auto plate_context = plant_plate.CreateDefaultContext();
  auto plate_context_ad = plant_plate_ad->CreateDefaultContext();

  ///
  drake::geometry::GeometryId plate_geoms =
      plant_plate.GetCollisionGeometriesForBody(
          plant_plate.GetBodyByName("plate"))[0];
  drake::geometry::GeometryId tray_geoms =
      plant_plate.GetCollisionGeometriesForBody(
          plant_plate.GetBodyByName("tray"))[0];
  //  drake::geometry::GeometryId ground_geoms =
  //      plant_plate.GetCollisionGeometriesForBody(plant_plate.GetBodyByName("box"))[0];
  //  drake::geometry::GeometryId sphere_geoms2 =
  //      plant_plate.GetCollisionGeometriesForBody(plant_plate.GetBodyByName("sphere2"))[0];
  std::vector<drake::geometry::GeometryId> contact_geoms = {plate_geoms,
                                                            tray_geoms};
  std::vector<SortedPair<GeometryId>> contact_pairs;
  for(int i = 0; i < contact_geoms.size(); ++i){
    for(int j = i+1; j < contact_geoms.size(); ++j){
      
    }
  }
  contact_pairs.push_back(SortedPair(contact_geoms[0], contact_geoms[1]));
  contact_pairs.push_back(SortedPair(contact_geoms[1], contact_geoms[2]));
  contact_pairs.push_back(SortedPair(contact_geoms[0], contact_geoms[3]));
  contact_pairs.push_back(SortedPair(contact_geoms[2], contact_geoms[3]));
  contact_pairs.push_back(SortedPair(contact_geoms[1], contact_geoms[3]));

  ///

  DiagramBuilder<double> builder;

  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);
  auto trajectory_sender =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_saved_traj>(
          controller_params.c3_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));

  auto lcs = LCSFactory::LinearizePlantToLCS(
      plant_plate, *plate_context, *plant_plate_ad, *plate_context_ad,
      contact_pairs, controller_params.num_friction_directions,
      controller_params.mu, controller_params.dt, 5);

  auto controller = builder.AddSystem<systems::C3Controller>(lcs.first, c3_options);

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