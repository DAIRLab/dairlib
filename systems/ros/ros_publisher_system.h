#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/leaf_system.h"

#include "ros/ros.h"

namespace drake_ros_systems {

using namespace drake;

/**
 * Publishes an ROS message containing information from its input port.
 *
 * @ingroup message_passing
 */
template <typename RosMessage>
class RosPublisherSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RosPublisherSystem)

  /**
   * A factory method that returns an %RosPublisherSystem that takes
   * Value<RosMessage> message objects on its sole abstract-valued input port.
   *
   * @tparam RosMessage message type to serialize.
   *
   * @param[in] topic The ROS topic on which to publish.
   *
   * @param node_handle The ROS context.
   */
  static std::unique_ptr<RosPublisherSystem<RosMessage>> Make(
      const std::string& topic, ros::NodeHandle* node_handle) {
    return std::make_unique<RosPublisherSystem<RosMessage>>(topic, node_handle);
  }

  // TODO(gizatt): add multiple DrakeRosInterface, so you can publish to a log
  // and real lcm at the same time. (TODO cloned from LCM publisher system,
  // originally attributed to Siyuan.)

  /**
   * A constructor for an %RosPublisherSystem that takes ROS message objects on
   * its sole abstract-valued input port. The ROS message type is determined by
   * the template type.
   *
   * @param[in] topic The ROS topic on which to publish.
   *
   * @param node_handle The ROS context.
   */
  RosPublisherSystem(const std::string& topic, ros::NodeHandle* node_handle)
      : topic_(topic), node_handle_(node_handle) {
    DRAKE_DEMAND(node_handle_ != nullptr);

    publisher_ = node_handle->advertise<RosMessage>(topic, 5);

    DeclareAbstractInputPort("ros_message", Value<RosMessage>());
    set_name(make_name(topic_));
  }

  ~RosPublisherSystem() override{};

  const std::string& get_topic_name() const { return topic_; }

  /// Returns the default name for a system that publishes @p topic.
  static std::string make_name(const std::string& topic) {
    return "RosPublisherSystem(" + topic + ")";
  }

  /**
   * Sets the publishing period of this system. See
   * LeafSystem::DeclarePublishPeriodSec() for details about the semantics of
   * parameter `period`.
   */
  void set_publish_period(double period) {
    LeafSystem<double>::DeclarePeriodicPublish(period);
  }

  /**
   * Takes the VectorBase from the input port of the context and publishes
   * it onto an ROS topic.
   */
  void DoPublish(
      const systems::Context<double>& context,
      const std::vector<const systems::PublishEvent<double>*>&) const override {
    SPDLOG_TRACE(drake::log(), "Publishing ROS {} message", topic_);

    const AbstractValue* const input_value =
        this->EvalAbstractInput(context, kPortIndex);
    DRAKE_ASSERT(input_value != nullptr);

    publisher_.publish(input_value->get_value<RosMessage>());
  }

 private:
  // The topic on which to publish ROS messages.
  const std::string topic_;

  ros::NodeHandle* const node_handle_{};
  ros::Publisher publisher_;

  const int kPortIndex = 0;
};

}  // namespace drake_ros_systems
