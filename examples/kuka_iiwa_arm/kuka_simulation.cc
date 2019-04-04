/// @file
///
/// Implements a simulation of the KUKA iiwa arm.  Like the driver for the
/// physical arm, this simulation communicates over LCM using lcmt_iiwa_status
/// and lcmt_iiwa_command messages. It is intended to be a be a direct
/// replacement for the KUKA iiwa driver and the actual robot hardware.

#include <memory>

#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/kuka_torque_controller.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/rbt_inverse_dynamics_controller.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_bool(visualize_frames, true, "Visualize end effector frames");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(torque_control, true, "Simulate using torque control mode.");

namespace dairlib {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
using drake::manipulation::util::SimDiagramBuilder;
using drake::systems::ConstantVectorSource;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::FrameVisualizer;
using drake::systems::RigidBodyPlant;
using drake::systems::Simulator;
using drake::systems::controllers::rbt::InverseDynamicsController;
using drake::systems::controllers::StateFeedbackControllerInterface;

int DoMain() {
  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;

  // Adds a plant.
  RigidBodyPlant<double>* plant = nullptr;
  const char* kModelPath =
      "../drake/manipulation/models/iiwa_description/"
      "urdf/iiwa14_polytope_collision.urdf";
  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : dairlib::FindResourceOrThrow(kModelPath));
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        urdf, drake::multibody::joints::kFixed, tree.get());
    drake::multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);
    plant = builder.AddPlant(std::move(tree));
  }
  // Creates and adds LCM publisher for visualization.
  builder.AddVisualizer(&lcm);
  builder.get_visualizer()->set_publish_period(drake::examples::kuka_iiwa_arm::kIiwaLcmStatusPeriod);

  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  const int num_joints = tree.get_num_positions();
  DRAKE_DEMAND(num_joints % drake::examples::kuka_iiwa_arm::kIiwaArmNumJoints == 0);
  const int num_iiwa = num_joints/drake::examples::kuka_iiwa_arm::kIiwaArmNumJoints;

  // Adds a iiwa controller.
  StateFeedbackControllerInterface<double>* controller = nullptr;
  if (FLAGS_torque_control) {
    printf("Using torque control\n");
    drake::VectorX<double> stiffness, damping_ratio;
    drake::examples::kuka_iiwa_arm::SetTorqueControlledIiwaGains(&stiffness, &damping_ratio);
    stiffness = stiffness.replicate(num_iiwa, 1);
    damping_ratio = damping_ratio.replicate(num_iiwa, 1);
    controller = builder.AddController<drake::examples::kuka_iiwa_arm::KukaTorqueController<double>>(
        RigidBodyTreeConstants::kFirstNonWorldModelInstanceId, tree.Clone(),
        stiffness, damping_ratio);
  } else {
    drake::VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
    drake::examples::kuka_iiwa_arm::SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
    iiwa_kp = iiwa_kp.replicate(num_iiwa, 1);
    iiwa_kd = iiwa_kd.replicate(num_iiwa, 1);
    iiwa_ki = iiwa_ki.replicate(num_iiwa, 1);
    controller = builder.AddController<InverseDynamicsController<double>>(
        RigidBodyTreeConstants::kFirstNonWorldModelInstanceId, tree.Clone(),
        iiwa_kp, iiwa_ki, iiwa_kd,
        false /* without feedforward acceleration */);
  }

  // Create the command subscriber and status publisher.
  drake::systems::DiagramBuilder<double>* base_builder = builder.get_mutable_builder();
  auto command_sub = base_builder->AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>("IIWA_COMMAND",
                                                                 &lcm));
  command_sub->set_name("command_subscriber");
  auto command_receiver =
      base_builder->AddSystem<drake::examples::kuka_iiwa_arm::IiwaCommandReceiver>(num_joints);
  command_receiver->set_name("command_receiver");
  std::vector<int> iiwa_instances =
      {RigidBodyTreeConstants::kFirstNonWorldModelInstanceId};
  auto external_torque_converter =
      base_builder->AddSystem<drake::examples::kuka_iiwa_arm::IiwaContactResultsToExternalTorque>(
          tree, iiwa_instances);
  auto status_pub = base_builder->AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_status>("IIWA_STATUS",
                                                               &lcm, drake::examples::kuka_iiwa_arm::kIiwaLcmStatusPeriod));
  status_pub->set_name("status_publisher");
  auto status_sender = base_builder->AddSystem<drake::examples::kuka_iiwa_arm::IiwaStatusSender>(num_joints);
  status_sender->set_name("status_sender");

  base_builder->Connect(command_sub->get_output_port(),
                        command_receiver->GetInputPort("command_message"));
  base_builder->Connect(command_receiver->get_commanded_state_output_port(),
                        controller->get_input_port_desired_state());
  base_builder->Connect(plant->get_output_port(0),
                        status_sender->get_state_input_port());
  base_builder->Connect(command_receiver->get_output_port(0),
                        status_sender->get_command_input_port());
  base_builder->Connect(controller->get_output_port_control(),
                        status_sender->get_commanded_torque_input_port());
  base_builder->Connect(plant->torque_output_port(),
                        status_sender->get_measured_torque_input_port());
  base_builder->Connect(plant->contact_results_output_port(),
                        external_torque_converter->get_input_port(0));
  base_builder->Connect(external_torque_converter->get_output_port(0),
                        status_sender->get_external_torque_input_port());
  base_builder->Connect(status_sender->get_output_port(0),
                        status_pub->get_input_port());
  // Connect the torque input in torque control
  if (FLAGS_torque_control) {
    drake::examples::kuka_iiwa_arm::KukaTorqueController<double>* torque_controller =
        dynamic_cast<drake::examples::kuka_iiwa_arm::KukaTorqueController<double>*>(controller);
    DRAKE_DEMAND(torque_controller);
    base_builder->Connect(command_receiver->get_output_port(1),
                          torque_controller->get_input_port_commanded_torque());
  }

  if (FLAGS_visualize_frames) {
    // TODO(sam.creasey) This try/catch block is here because even
    // though RigidBodyTree::FindBody returns a pointer and could return
    // null to indicate failure, it throws instead.  At any rate, warn
    // instead of dying if the links aren't named as expected.  This
    // happens (for example) when loading
    // dual_iiwa14_polytope_collision.urdf.
    try {
    // Visualizes the end effector frame and 7th body's frame.
      std::vector<RigidBodyFrame<double>> local_transforms;
      local_transforms.push_back(
          RigidBodyFrame<double>("iiwa_link_ee", tree.FindBody("iiwa_link_ee"),
                                 drake::Isometry3<double>::Identity()));
      local_transforms.push_back(
          RigidBodyFrame<double>("iiwa_link_7", tree.FindBody("iiwa_link_7"),
                                 drake::Isometry3<double>::Identity()));
      local_transforms.push_back(
          RigidBodyFrame<double>("iiwa_link_6", tree.FindBody("iiwa_link_6"),
                                 drake::Isometry3<double>::Identity()));
      local_transforms.push_back(
          RigidBodyFrame<double>("iiwa_link_5", tree.FindBody("iiwa_link_5"),
                                 drake::Isometry3<double>::Identity()));
      local_transforms.push_back(
          RigidBodyFrame<double>("iiwa_link_4", tree.FindBody("iiwa_link_4"),
                                 drake::Isometry3<double>::Identity()));
      local_transforms.push_back(
          RigidBodyFrame<double>("iiwa_link_3", tree.FindBody("iiwa_link_3"),
                                 drake::Isometry3<double>::Identity()));
      local_transforms.push_back(
          RigidBodyFrame<double>("iiwa_link_2", tree.FindBody("iiwa_link_2"),
                                 drake::Isometry3<double>::Identity()));
      local_transforms.push_back(
          RigidBodyFrame<double>("iiwa_link_1", tree.FindBody("iiwa_link_1"),
                                 drake::Isometry3<double>::Identity()));
      local_transforms.push_back(
          RigidBodyFrame<double>("iiwa_link_0", tree.FindBody("iiwa_link_0"),
                                 drake::Isometry3<double>::Identity()));
      auto frame_viz = base_builder->AddSystem<drake::systems::FrameVisualizer>(
          &tree, local_transforms, &lcm);
      base_builder->Connect(plant->get_output_port(0),
                            frame_viz->get_input_port(0));
      frame_viz->set_publish_period(drake::examples::kuka_iiwa_arm::kIiwaLcmStatusPeriod);
    } catch (std::logic_error& ex) {
      drake::log()->error("Unable to visualize end effector frames:\n{}\n"
                          "Maybe use --novisualize_frames?",
                          ex.what());
      return 1;
    }
  }

  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  // Eigen::Matrix< double , 7 , 1> InitialPositions;
  // InitialPositions << 3.14/4, -3.14/2, (3.14)/2, 3.14/2, (3.14)/2, (3.14)/2, 1*(3.14)/4.;

  // command_receiver->set_initial_position(
  //     &sys->GetMutableSubsystemContext(*command_receiver,
  //                                      &simulator.get_mutable_context()),
  //     InitialPositions);

  // Simulate for a very long time.
  simulator.StepTo(FLAGS_simulation_sec);

  lcm.StopReceiveThread();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return dairlib::examples::kuka_iiwa_arm::DoMain();
}
