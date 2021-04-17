#pragma once

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::TriggerTypeSet;

namespace dairlib {
namespace goldilocks_models {

template <typename MessageType>
void NetworkPublisher(const std::string& channel_in,
                      const std::string& channel_out, int n_publishes) {
  // Parameters
  //  double init_publish_period = 0.1;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  // Build the diagram
  drake::systems::DiagramBuilder<double> builder;
  auto name_pub = builder.AddSystem(LcmPublisherSystem::Make<MessageType>(
      channel_out, &lcm_network, TriggerTypeSet({TriggerType::kForced})));
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("network_publisher_for_" + channel_out));

  // Create simulator
  drake::systems::Diagram<double>* diagram_ptr = owned_diagram.get();
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();

  // Create subscriber for lcm driven loop
  drake::lcm::Subscriber<MessageType> input_sub(&lcm_local, channel_in);
  drake::lcm::Subscriber<dairlib::lcmt_timestamped_saved_traj> mpc_sub(&lcm_local,
                                                           "MPC_OUTPUT");

  // Wait for the first message
  drake::log()->info("Waiting for first lcm input message");
  LcmHandleSubscriptionsUntil(&lcm_local,
                              [&]() { return input_sub.count() > 0; });
  drake::log()->info(diagram_ptr->get_name() + " started");

  while (true) {
    // Wait for input message.
    input_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm_local,
                                [&]() { return input_sub.count() > 0; });
    // Pass output message
    MessageType msg;
    msg = input_sub.message();
    name_pub->get_input_port().FixValue(
        &(diagram_ptr->GetMutableSubsystemContext(*name_pub, &diagram_context)),
        msg);
    // Force-publish via the diagram
    diagram_ptr->Publish(diagram_context);

    // Once we have the first mpc message, enter this endless while loop
    if (true /*mpc_sub.count() > 0*/) {
      while (true) {
        // Wait for input message.
        input_sub.clear();
        std::cout << "waiting for input message\n";
        LcmHandleSubscriptionsUntil(&lcm_local,
                                    [&]() { return input_sub.count() > 0; });
        mpc_sub.clear();
        std::cout << "waiting for mpc message\n";
        LcmHandleSubscriptionsUntil(&lcm_local,
                                    [&]() { return mpc_sub.count() > 0; });
        input_sub.clear();
        std::cout << "waiting for input message\n";
        LcmHandleSubscriptionsUntil(&lcm_local,
                                    [&]() { return input_sub.count() > 0; });

        int pub_count = 0;
        while (pub_count < n_publishes) {
          // Get message time from the input channel
          // double t_current = input_sub.message().utime * 1e-6;
          // std::cout << "publish at t = " << t_current << std::endl;

          // Pass output message
          MessageType msg;
          msg = input_sub.message();
          name_pub->get_input_port().FixValue(
              &(diagram_ptr->GetMutableSubsystemContext(*name_pub,
                                                        &diagram_context)),
              msg);

          // Force-publish via the diagram
          diagram_ptr->Publish(diagram_context);

          pub_count++;
        }
      }
    }
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib
