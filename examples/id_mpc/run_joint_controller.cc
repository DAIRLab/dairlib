#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "systems/robot_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/controllers/id_mpc/systems/joint_pd_controller.h"

#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmInterfaceSystem;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;


int DoMain() {
  // TODO (@Brian-Acosta) yaml or gflags for these
  std::string mpc_channel = "ID_MPC";
  std::string input_channel = "CASSIE_INPUT";
  std::string state_channel = "CASSIE_STATE_SIMULATION";

  std::string urdf = "examples/Cassie/urdf/cassie_fixed_spring_conservative.urdf";

  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser parser(&plant);
  parser.AddModels(urdf);
  plant.Finalize();

  drake::systems::DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");
  auto mpc_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_timestamped_saved_traj>(
          mpc_channel, &lcm));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant);

  // TODO (@Brian-Acosta) YAML-ize this
  std::unordered_map<std::string, double> kp;
  kp.insert({"hip_roll_right", 100});
  kp.insert({"hip_yaw_right", 100});
  kp.insert({"hip_pitch_right", 300});
  kp.insert({"knee_right", 400});
  kp.insert({"toe_right", 100});
  kp.insert({"hip_roll_left", 100});
  kp.insert({"hip_yaw_left", 100});
  kp.insert({"hip_pitch_left", 300});
  kp.insert({"knee_left", 400});
  kp.insert({"toe_left", 100});

  std::unordered_map<std::string, double> kd;
  kd.insert({"hip_roll_right", 5});
  kd.insert({"hip_yaw_right", 5});
  kd.insert({"hip_pitch_right", 10});
  kd.insert({"knee_right", 15});
  kd.insert({"toe_right", 5});
  kd.insert({"hip_roll_left", 5});
  kd.insert({"hip_yaw_left", 5});
  kd.insert({"hip_pitch_left", 10});
  kd.insert({"knee_left", 15});
  kd.insert({"toe_left", 5});

  auto pd_controller = builder.AddSystem<JointPDController>(plant, kp, kd);
  auto command_sender = builder.AddSystem<RobotCommandSender>(plant);
  auto command_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_robot_input>(
          input_channel, &lcm, TriggerTypeSet({TriggerType::kForced})));

  builder.Connect(state_receiver->get_output_port(),
                  pd_controller->get_input_port_state());
  builder.Connect(mpc_sub->get_output_port(),
                  pd_controller->get_input_port_lcm_traj());
  builder.Connect(*pd_controller, *command_sender);
  builder.Connect(*command_sender, *command_pub);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  LcmDrivenLoop<lcmt_robot_output> loop(
      &lcm, std::move(diagram), state_receiver, state_channel, true);

  auto& loop_context = loop.get_diagram_mutable_context();

  LcmHandleSubscriptionsUntil(&lcm, [&]() {
    return mpc_sub->GetInternalMessageCount() > 1; });

  mpc_sub->ForcedPublish(loop.get_diagram()->
      GetMutableSubsystemContext(*mpc_sub, &loop_context));

  loop.Simulate();

  return 0;
}

}

int main(int argc, char*argv[]) {
  return dairlib::systems::controllers::id_mpc::DoMain();
}