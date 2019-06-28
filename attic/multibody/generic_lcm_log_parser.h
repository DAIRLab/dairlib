#pragma once

#include <string>
#include "drake/multibody/rigid_body.h"
#include "attic/multibody/generic_lcm_log_parser.h"
#include "drake/lcm/drake_lcm_log.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_log_playback_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "systems/primitives/vector_aggregator.h"

namespace dairlib {
namespace multibody {

  template<typename T, typename U>
  void parseLcmLog(const RigidBodyTree<double>& tree,
      std::string file, std::string channel,
      Eigen::VectorXd* t, Eigen::MatrixXd* x, double duration = 1.0e6) {

    using drake::systems::lcm::LcmSubscriberSystem;

    drake::lcm::DrakeLcmLog r_log(file, false);

    drake::systems::DiagramBuilder<double> builder;

    builder.AddSystem<drake::systems::lcm::LcmLogPlaybackSystem>(&r_log);

    auto output_sub = builder.AddSystem(
        LcmSubscriberSystem::Make<T>(channel, &r_log));

    auto output_receiver = builder.AddSystem<U>(tree);
    builder.Connect(*output_sub, *output_receiver);

    auto output_aggregator = builder.AddSystem<systems::VectorAggregator>(
        output_receiver->get_output_port(0).size() - 1);

    builder.Connect(*output_receiver, *output_aggregator);

    auto diagram = builder.Build();

    drake::systems::Simulator<double> sim(*diagram);

    sim.StepTo(r_log.GetNextMessageTime() + duration);

    *t = output_aggregator->BuildTimestampVector();

    *x = output_aggregator->BuildMatrixFromVectors();
  }
} // multibody
} //dairlib
