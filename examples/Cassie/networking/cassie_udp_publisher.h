#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/leaf_system.h"
#include "examples/Cassie/networking/udp_serializer.h"


namespace dairlib {
namespace systems {

using UDPTriggerTypes =
    std::unordered_set<drake::systems::TriggerType, drake::DefaultHash>;


/**
 * Publishes a UDP message containing information from its input port.
 * Publishing can be set up to happen on a per-step or periodic basis.
 * Publishing "by force", through
 * `CassieUDPPublisher::Publish(const Context&)`, is also enabled.
 *
 * @ingroup message_passing
 */
class CassieUDPPublisher : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieUDPPublisher)

  /**
   * A factory method that returns an %CassieUDPPublisher
   *
   * @param address the IP address to subscribe to
   *
   * @param port the port to listen on
   *
   * @param publish_triggers A set determining which events trigger publishes
   * must be a non-empty subset of {kForced, kPeriodic, kPerStep}
   *
   * @param publish_period Period that messages will be published (optional).
   * If the publish period is zero, CassieUDPPublisher will use per-step
   * publishing instead; see LeafSystem::DeclarePerStepPublishEvent().
   *
   * @pre publish_period is non-negative.
   * @pre publish_period > iff publish_triggers contains kPeriodic
   */
  static std::unique_ptr<CassieUDPPublisher> Make(const std::string& address,
      const int port, const UDPTriggerTypes& publish_triggers,
      double publish_period = 0.0) {
    return std::make_unique<CassieUDPPublisher>(address, port, publish_triggers,
        publish_period);
  }

/**
   * A factory method that returns an %CassieUDPPublisher
   * Instantates the default publish triggers: kForced and either (kPeriodic
   * if publish_period > 0) or (kPerStep if publish_period = 0)
   *
   * @param address the IP address to subscribe to
   *
   * @param port the port to listen on
   *
   * @param publish_period Period that messages will be published (optional).
   * If the publish period is zero, CassieUDPPublisher will use per-step
   * publishing instead; see LeafSystem::DeclarePerStepPublishEvent().
   *
   * @pre publish_period is non-negative.
   */
  static std::unique_ptr<CassieUDPPublisher> Make(const std::string& address,
      const int port, double publish_period = 0.0) {
    return std::make_unique<CassieUDPPublisher>(address, port, publish_period);
  }

  /**
   * A constructor for a %CassieUDPPublisher
   *
   * @param address the IP address to subscribe to
   *
   * @param port the port to listen on
   *
   * @param publish_triggers A set determining which events trigger publishes
   * must be a non-empty subset of {kForced, kPeriodic, kPerStep}
   *
   * @param publish_period Period that messages will be published (optional).
   * If the publish period is zero, CassieUDPPublisher will use per-step
   * publishing instead; see LeafSystem::DeclarePerStepPublishEvent().
   *
   * @pre publish_period is non-negative.
   */
  CassieUDPPublisher(const std::string& address, const int port,
      const UDPTriggerTypes& publish_triggers, double publish_period = 0.0);


  /**
   * A constructor for a %CassieUDPPublisher
   * Instantates the default publish triggers: kForced and either (kPeriodic
   * if publish_period > 0) or (kPerStep if publish_period = 0)
   *
   * @param address the IP address to subscribe to
   *
   * @param port the port to listen on
   *
   * @param publish_period Period that messages will be published (optional).
   * If the publish period is zero, CassieUDPPublisher will use per-step
   * publishing instead; see LeafSystem::DeclarePerStepPublishEvent().
   *
   * @pre publish_period is non-negative.
   */
  CassieUDPPublisher(const std::string& address, const int port,
      double publish_period = 0.0);

  ~CassieUDPPublisher() override;
  /**
   * Returns the default name for a system that publishes @p address: @p port.
   */
  std::string make_name(const std::string& address, const int port);

  /**
   * Returns the sole input port.
   */
  const drake::systems::InputPort<double>& get_input_port() const {
    DRAKE_THROW_UNLESS(this->num_input_ports() == 1);
    return drake::systems::LeafSystem<double>::get_input_port(0);
  }

  // Don't use the indexed overload; use the no-arg overload.
  void get_input_port(int index) = delete;

  // This system has no output ports.
  void get_output_port(int) = delete;

 private:
  drake::systems::EventStatus PublishInputAsUDPMessage(
      const drake::systems::Context<double>& context) const;

  // The IP address to which to publish UDP messages.
  const std::string address_;
  const int port_;
  struct sockaddr_in server_address_;
  int socket_;

  // Converts Value<cassie_user_in_t> objects into UDP message bytes.
  std::unique_ptr<CassieUDPInSerializer> serializer_;
};

}  // namespace systems
}  // namespace dairlib
