
#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <gflags/gflags.h>

#include "examples/deform/deform_utils.h"
#include "examples/deform/elastoplastic_controller.h"
#include "examples/deform/parameter_headers/elastoplastic_c3_options.h"
#include "examples/deform/parameter_headers/lcm_channels.h"
#include "solvers/lcs_factory.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/senders/c3_state_sender.h"
#include "systems/simple_robot_object_kinematics.h"
#include "systems/system_utils.h"
#include "systems/trajectory_optimization/c3_output_systems.h"

#include "drake/common/drake_copyable.h"
#include "drake/multibody/parsing/parser.h"

namespace dairlib {

using dairlib::examples::deform::ElastoPlasticController;
using dairlib::solvers::LCSFactory;
using dairlib::systems::SimpleRobotObjectKinematics;
using dairlib::systems::SubvectorPassThrough;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::lcm::DrakeLcm;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::Parser;
using drake::systems::ConstantVectorSource;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::VectorXd;

DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");
DEFINE_string(demo, "1d_rigid", "Demo type");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load parameters.
  //   ElastoPlasticControllerParams control_params =
  //       drake::yaml::LoadYamlFile<ElastoPlasticControllerParams>(
  //           "examples/deform/parameters/demo_" + FLAGS_demo +
  //           "/control_params.yaml");
  DeformLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<DeformLcmChannels>(
          "examples/deform/parameters/lcm_channels_sim.yaml");
  ElastoPlasticC3Options elastoplastic_c3_options =
      drake::yaml::LoadYamlFile<ElastoPlasticC3Options>(
          "examples/deform/parameters/demo_" + FLAGS_demo +
          "/elastoplastic_c3_options.yaml");
  std::cout << "g_lambda size in run_elastoplastic_control: "
            << elastoplastic_c3_options.GetC3Options().g_lambda.size()
            << std::endl;

  // Create the LCS plant containing the two-link "elastoplastic," EE, and
  // ground.
  DiagramBuilder<double> plant_lcs_builder;
  auto [plant_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_lcs_builder, 0.0);
  std::vector<ModelInstanceIndex> robot_obj_indices =
      AddLCSModelsToPlant(&plant_lcs, &scene_graph, FLAGS_demo);
  ModelInstanceIndex robot_idx = robot_obj_indices[0];
  ModelInstanceIndex object_idx = robot_obj_indices[1];
  plant_lcs.Finalize();

  // Autodiff.
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_lcs);

  auto plant_lcs_diagram = plant_lcs_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_lcs_diagram->CreateDefaultContext();
  auto& plant_lcs_context = plant_lcs_diagram->GetMutableSubsystemContext(
      plant_lcs, diagram_context.get());
  auto plant_lcs_context_ad = plant_lcs_autodiff->CreateDefaultContext();

  // Build the contact pairs based on the demo.
  std::vector<std::vector<SortedPair<GeometryId>>> contact_pairs;
  if (FLAGS_demo == "1d" || FLAGS_demo == "1d_rigid") {
    GeometryId ee = plant_lcs.GetCollisionGeometriesForBody(
        plant_lcs.GetBodyByName("ee"))[0];
    GeometryId ground = plant_lcs.GetCollisionGeometriesForBody(
        plant_lcs.GetBodyByName("ground"))[0];
    GeometryId frictional_slider = plant_lcs.GetCollisionGeometriesForBody(
        plant_lcs.GetBodyByName("frictional_slider"))[1];  // idx 1 for sphere
    GeometryId object_for_ee_contact;
    if (FLAGS_demo == "1d") {
      object_for_ee_contact = plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("springed"))[0];
    } else {
      object_for_ee_contact = plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("frictional_slider"))[0];  // idx 0 for box
    }

    // Add pairs:  ground-frictional_slider, EE-object_for_ee_contact
    std::vector<SortedPair<GeometryId>> obj_ground_pairs;
    obj_ground_pairs.push_back(
        SortedPair<GeometryId>(ground, frictional_slider));
    contact_pairs.push_back(obj_ground_pairs);
    std::vector<SortedPair<GeometryId>> obj_ee_pairs;
    obj_ee_pairs.push_back(SortedPair<GeometryId>(ee, object_for_ee_contact));
    contact_pairs.push_back(obj_ee_pairs);

  }  // No need for an else to here, since AddLCSModelsToPlant complains first.

  // Piece together the diagram:  6 steps.
  DiagramBuilder<double> builder;

  // (1/6) Our C3 controller.
  auto controller = builder.AddSystem<ElastoPlasticController>(
      plant_lcs, &plant_lcs_context, *plant_lcs_autodiff,
      plant_lcs_context_ad.get(), contact_pairs, elastoplastic_c3_options);

  // (2/6) Subscribers and state receivers:  radio, object state, robot state.
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(FLAGS_lcm_url);
  DrakeLcm lcm_network(FLAGS_lcm_url);
  auto radio_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
          lcm_channel_params.radio_channel, &lcm_network));
  auto object_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_object_state>(
          lcm_channel_params.object_state_channel, &lcm_network));
  auto robot_state_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          lcm_channel_params.robot_state_channel, &lcm_network));
  auto object_state_receiver =
      builder.AddSystem<systems::ObjectStateReceiver>(plant_lcs, object_idx);
  auto robot_state_receiver =
      builder.AddSystem<dairlib::systems::RobotOutputReceiver>(plant_lcs,
                                                               robot_idx);
  builder.Connect(*object_state_sub, *object_state_receiver);
  builder.Connect(*robot_state_sub, *robot_state_receiver);

  // (3/6) Constant vector source for desired goal state.
  VectorXd x_lcs_pos = elastoplastic_c3_options.q_target;
  VectorXd x_lcs_vel = VectorXd::Zero(plant_lcs.num_velocities());
  VectorXd x_lcs_des =
      VectorXd::Zero(plant_lcs.num_positions() + plant_lcs.num_velocities());
  x_lcs_des.segment(0, plant_lcs.num_positions()) = x_lcs_pos;
  x_lcs_des.segment(plant_lcs.num_positions(), plant_lcs.num_velocities()) =
      x_lcs_vel;
  auto x_lcs_des_source = builder.AddSystem<ConstantVectorSource>(x_lcs_des);
  builder.Connect(x_lcs_des_source->get_output_port(),
                  controller->get_input_port_target());

  // (4/6) Simple robot-object kinematics system to piece together LCS state.
  auto kinematics = builder.AddSystem<SimpleRobotObjectKinematics>(
      plant_lcs, robot_idx, object_idx, true);
  builder.Connect(object_state_receiver->get_output_port(),
                  kinematics->get_input_port_object_state());
  builder.Connect(robot_state_receiver->get_output_port(),
                  kinematics->get_input_port_robot_state());
  builder.Connect(kinematics->get_output_port_lcs_state(),
                  controller->get_input_port_lcs_state());

  // (5/6) C3 state sender and C3 output sender.
  std::vector<std::string> state_names = elastoplastic_c3_options.state_names;
  auto c3_state_sender = builder.AddSystem<systems::C3StateSender>(
      plant_lcs.num_positions() + plant_lcs.num_velocities(), state_names);
  builder.Connect(x_lcs_des_source->get_output_port(),
                  c3_state_sender->get_input_port_target_state());
  builder.Connect(kinematics->get_output_port_lcs_state(),
                  c3_state_sender->get_input_port_actual_state());
  auto c3_output_sender = builder.AddSystem<systems::C3OutputSender>();
  builder.Connect(controller->get_output_port_c3_solution(),
                  c3_output_sender->get_input_port_c3_solution());
  builder.Connect(controller->get_output_port_c3_intermediates(),
                  c3_output_sender->get_input_port_c3_intermediates());

  // (6/6) Publishers:  current/desired LCS states, C3 output/forces, efforts,
  // costs.
  auto c3_target_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_target_state_channel, &lcm_network,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_actual_state_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_state>(
          lcm_channel_params.c3_actual_state_channel, &lcm_network,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(c3_state_sender->get_output_port_target_c3_state(),
                  c3_target_state_publisher->get_input_port());
  builder.Connect(c3_state_sender->get_output_port_actual_c3_state(),
                  c3_actual_state_publisher->get_input_port());
  auto c3_output_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_output>(
          lcm_channel_params.c3_debug_output_channel, &lcm_network,
          TriggerTypeSet({TriggerType::kForced})));
  auto c3_forces_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_forces>(
          lcm_channel_params.c3_force_channel, &lcm_network,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(c3_output_sender->get_output_port_c3_debug(),
                  c3_output_publisher->get_input_port());
  builder.Connect(controller->get_output_port_c3_forces(),
                  c3_forces_publisher->get_input_port());
  auto efforts_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          lcm_channel_params.robot_input_channel, &lcm_network,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(controller->get_output_port_efforts(),
                  efforts_publisher->get_input_port());
  auto c3_costs_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_c3_costs>(
          lcm_channel_params.c3_costs_channel, &lcm_network,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(controller->get_output_port_c3_costs(),
                  c3_costs_publisher->get_input_port());

  // Build diagram.
  auto owned_diagram = builder.Build();
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  shared_diagram->set_name(("elastoplastic_control_" + FLAGS_demo));
  dairlib::DrawAndSaveDiagramGraph(*shared_diagram);

  // Run lcm-driven simulation.
  dairlib::systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_network, shared_diagram, robot_state_receiver,
      lcm_channel_params.robot_state_channel, true);

  // Wait for the first robot and object messages.
  drake::log()->info("Waiting for first robot message...");
  drake::lcm::Subscriber<dairlib::lcmt_robot_output> robot_sub(
      lcm, lcm_channel_params.robot_state_channel);
  LcmHandleSubscriptionsUntil(lcm, [&]() { return robot_sub.count() > 0; });
  drake::log()->info("Waiting for first object message...");
  drake::lcm::Subscriber<dairlib::lcmt_object_state> object_sub(
      lcm, lcm_channel_params.object_state_channel);
  LcmHandleSubscriptionsUntil(lcm, [&]() { return object_sub.count() > 0; });
  drake::log()->info("Received first robot and object messages.");

  drake::log()->info("Elastoplastic controller started");
  loop.Simulate(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
