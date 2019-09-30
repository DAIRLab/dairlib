#pragma once

#include <string>
#include "drake/multibody/rigid_body.h"
#include "drake/lcm/drake_lcm_log.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_log_playback_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "systems/primitives/vector_aggregator.h"

namespace dairlib {
namespace multibody {

/// parseLcmLog() parses lcm log files where the information can
/// be represented as a vector
///
/// Template T - lcmtype
/// Template U - class to convert lcm message to a vector (will be inferred from
/// the input `system`
///
/// Input:
///   - RigidBodyTree `tree` to be passed as argument to the class `U`
///   - string `file` with the path to the location of the log file
///   - string `channel` with the name of the channel containing the lcm
///     message of type `T`
///   - optional `duration` till which the lcm messages should be parsed
///
/// Output:
///   - VectorXd `t` to store time
///   - MatrixXd `x` to store the information in the lcm message
template <typename T, typename U>
void parseLcmLog(std::unique_ptr<U> system, std::string file,
                 std::string channel, Eigen::VectorXd* t, Eigen::MatrixXd* x,
                 double duration = 1.0e6) {
  using drake::systems::lcm::LcmSubscriberSystem;

  drake::lcm::DrakeLcmLog r_log(file, false);

  drake::systems::DiagramBuilder<double> builder;

  builder.AddSystem<drake::systems::lcm::LcmLogPlaybackSystem>(&r_log);

  auto output_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<T>(channel, &r_log));

  auto output_receiver = builder.AddSystem(std::move(system));
  builder.Connect(*output_sub, *output_receiver);

  auto output_aggregator = builder.AddSystem<systems::VectorAggregator>(
      output_receiver->get_output_port(0).size() - 1);

  builder.Connect(*output_receiver, *output_aggregator);

  auto diagram = builder.Build();

  drake::systems::Simulator<double> sim(*diagram);

  sim.AdvanceTo(r_log.GetNextMessageTime() + duration);

  *t = output_aggregator->BuildTimestampVector();

  *x = output_aggregator->BuildMatrixFromVectors();
  }
} // multibody
} //dairlib

