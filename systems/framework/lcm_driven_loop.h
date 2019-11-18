#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/lcm/drake_lcm.h"

#include "dairlib/lcmt_controller_switch.hpp"

namespace dairlib {
namespace systems {

/// LcmDrivenLoop runs the simulation of a diagram (the whole system) of which
/// update is triggered by the incoming lcm messages.
/// It can handle single and multiple incoming lcm messages.
/// In the case of single message, the diagram is triggered by this lcm
/// trivially.
/// In the case of multiple incoming messages (we call them input messages),
/// the user has to provide one extra lcm message (we call it switch message)
/// which tells LcmDrivenLoop the channel that it should listen to for the
/// simulation update.

// We set a default value for SwitchMessageType so that we can generalize this
// to both single and multi inputs. (for template instantiation)
template <typename InputMessageType,
    typename SwitchMessageType = dairlib::lcmt_controller_switch>
class LcmDrivenLoop {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmDrivenLoop)

  /// Inputs of the single-input LcmDrivenLoop constructor:
  ///     drake_lcm: DrakeLcm
  ///     diagram: a drake diagram
  ///     first_leafsystem: the first block of the diagram
  ///     input_channel: the name of the input channel
  ///
  /// Inputs of the multi-input LcmDrivenLoop constructor:
  ///     drake_lcm: DrakeLcm
  ///     diagram: a drake diagram
  ///     first_leafsystem: the first block of the diagram
  ///     switch_channel: the name of the switch channel
  ///     input_channels: the names of the input channels

  /// Procedures to use LcmDrivenLoop:
  /// 1. construct LcmDrivenLoop
  /// 2. (if it's multi-input) the user can set the initial channel that
  ///    LcmDrivenLoop listens to by calling SetInitActiveChannel().
  /// 3. run Simulate()

  // Lcm driven loop for single input
  LcmDrivenLoop(drake::lcm::DrakeLcm* drake_lcm,
                std::unique_ptr<drake::systems::Diagram<double>> diagram,
                const drake::systems::LeafSystem<double>* first_leafsystem,
                const std::string& input_channel);

  // Lcm driven loop for multiple inputs
  LcmDrivenLoop(drake::lcm::DrakeLcm* drake_lcm,
                std::unique_ptr<drake::systems::Diagram<double>> diagram,
                const drake::systems::LeafSystem<double>* first_leafsystem,
                const std::string& switch_channel,
                std::vector<std::string> input_channels);

  // Set the initial active input channel
  void SetInitActiveChannel(const std::string& init_channel);

  // Start simulating the diagram
  void Simulate(double end_time = std::numeric_limits<double>::infinity());

  // Give a name to the diagram
  void SetName(const std::string & diagram_name);

 private:
  LcmDrivenLoop(drake::lcm::DrakeLcm* drake_lcm,
                std::unique_ptr<drake::systems::Diagram<double>> diagram,
                const drake::systems::LeafSystem<double>* first_leafsystem);
  std::string diagram_name_ = "diagram";

  drake::lcm::DrakeLcm* drake_lcm_;
  drake::systems::Diagram<double>* diagram_ptr_;
  const drake::systems::LeafSystem<double>* first_leafsystem_;

  std::unique_ptr<drake::systems::Simulator<double>> simulator_;

  std::string active_channel_;
  std::unique_ptr<drake::lcm::Subscriber<SwitchMessageType>> switch_sub_;
  std::map<std::string, drake::lcm::Subscriber<InputMessageType>>
      name_to_input_sub_map_;

  bool is_multiple_inputs_ = false;
};

}  // namespace systems
}  // namespace dairlib