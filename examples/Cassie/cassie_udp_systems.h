#pragma once

#include <atomic>
#include <memory>

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"
#include "drake/systems/framework/basic_vector.h"

#include "cassie_udp_spoofer.h"
#include "datatypes/cassie_dispatch_types.h"
#include "datatypes/cassie_names.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"

namespace dairlib {

class CassieUdpOutputPublisher : public LeafSystem<double> {
 public:
  CassieUdpOutputPublisher(std::shared_ptr<CassieUdpSpoofer> spoofer);

 private:
  void DoPublish(const Context<double>& context,
               const std::vector<const systems::PublishEvent<double>*>&) const;
  std::shared_ptr<CassieUdpSpoofer> _spoofer;
};

class CassieUdpInputSubscriber : public LeafSystem<double> {
 public:
  CassieUdpInputReceiver(std::shared_ptr<CassieUdpSpoofer> spoofer);
  //has no inputs
  void get_input_port(int) = delete;

 private:
  void Output(const Context<double>& context,
                    lcmt_robot_input* output) const;

  std::atomic<lcmt_robot_input> last_received_input;
  void ProcessInput(cassie_dispatch_robot_in_t new_input);
};

}
