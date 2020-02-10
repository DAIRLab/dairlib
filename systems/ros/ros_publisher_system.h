#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_set>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

#include "ros/ros.h"

namespace dairlib {
namespace systems {

using TriggerTypeSet =
    std::unordered_set<drake::systems::TriggerType, drake::DefaultHash>;

/**
 * Publishes an ROS message containing information from its input port.
 *
 * @ingroup message_passing
 */
template <typename RosMessage>
class RosPublisherSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RosPublisherSystem)

  /**
   * A factory method that returns an %RosPublisherSystem that takes
   * Value<RosMessage> message objects on its sole abstract-valued input port.
   *
   * @param[in] topic The ROS topic on which to publish.
   *
   * @param node_handle The ROS context.
   *
   * @param publish_triggers Set of triggers that determine when messages will
   * be published. Supported TriggerTypes are {kForced, kPeriodic, kPerStep}.
   * Will throw an error if empty or if unsupported types are provided.
   *
   * @param publish_period Period that messages will be published (optional).
   * publish_period should only be non-zero if one of the publish_triggers is
   * kPeriodic.
   */
  static std::unique_ptr<RosPublisherSystem<RosMessage>> Make(
      const std::string& topic, ros::NodeHandle* node_handle,
      double publish_period = 0.0) {
    return std::make_unique<RosPublisherSystem<RosMessage>>(topic, node_handle,
      publish_period);
  }

  /**
   * A factory method that returns an %RosPublisherSystem that takes
   * Value<RosMessage> message objects on its sole abstract-valued input port.
   *
   * @param[in] topic The ROS topic on which to publish.
   *
   * @param node_handle The ROS context.
   *
   * @param publish_period Period that messages will be published (optional).
   * If the publish period is zero, RosPublisherSystem will use per-step
   * publishing instead; see LeafSystem::DeclarePerStepPublishEvent().
   */
  static std::unique_ptr<RosPublisherSystem<RosMessage>> Make(
      const std::string& topic, ros::NodeHandle* node_handle,
      const TriggerTypeSet& publish_triggers, double publish_period = 0.0) {
    return std::make_unique<RosPublisherSystem<RosMessage>>(topic, node_handle,
      publish_triggers, publish_period);
  }

  /**
   * A constructor for an %RosPublisherSystem that takes ROS message objects on
   * its sole abstract-valued input port. The ROS message type is determined by
   * the template type.
   *
   * @param[in] topic The ROS topic on which to publish.
   *
   * @param node_handle The ROS context.
   *
   * @param publish_triggers Set of triggers that determine when messages will
   * be published. Supported TriggerTypes are {kForced, kPeriodic, kPerStep}.
   * Will throw an error if empty or if unsupported types are provided.
   *
   * @param publish_period Period that messages will be published (optional).
   * publish_period should only be non-zero if one of the publish_triggers is
   * kPeriodic.
   */
  RosPublisherSystem(const std::string& topic, ros::NodeHandle* node_handle,
      const TriggerTypeSet& publish_triggers, double publish_period = 0.0)
      : topic_(topic), node_handle_(node_handle) {
    DRAKE_DEMAND(node_handle_ != nullptr);
    DRAKE_DEMAND(publish_period >= 0.0);
    DRAKE_DEMAND(!publish_triggers.empty());

    using drake::systems::TriggerType;

    // Check that publish_triggers does not contain an unsupported trigger.
    for (const auto& trigger : publish_triggers) {
      DRAKE_THROW_UNLESS((trigger == TriggerType::kForced) ||
        (trigger == TriggerType::kPeriodic) ||
        (trigger == TriggerType::kPerStep));
    }

    // Outgoing queue size chosen to be small, but 5 is arbitrary
    publisher_ = node_handle->advertise<RosMessage>(topic, 5);
    // check that publisher is not empty
    DRAKE_THROW_UNLESS(publisher_);

    DeclareAbstractInputPort("ros_message", drake::Value<RosMessage>());
    set_name(make_name(topic_));

    // Declare a forced publish so that any time Publish(.) is called on this
    // system (or a Diagram containing it), a message is emitted.
    if (publish_triggers.find(TriggerType::kForced) != publish_triggers.end()) {
      this->DeclareForcedPublishEvent(&RosPublisherSystem::PublishToRosTopic);
    }

    if (publish_triggers.find(TriggerType::kPeriodic) !=
        publish_triggers.end()) {
      DRAKE_THROW_UNLESS(publish_period > 0.0);
      const double offset = 0.0;
      this->DeclarePeriodicPublishEvent(publish_period, offset,
          &RosPublisherSystem::PublishToRosTopic);
    } else {
      // publish_period > 0 without TriggerType::kPeriodic has no meaning and is
      // likely a mistake.
      DRAKE_THROW_UNLESS(publish_period == 0.0);
    }

    if (publish_triggers.find(TriggerType::kPerStep) !=
        publish_triggers.end()) {
      this->DeclarePerStepEvent(
      drake::systems::PublishEvent<double>([this](
          const drake::systems::Context<double>& context,
          const drake::systems::PublishEvent<double>&) {
        this->PublishToRosTopic(context);
      }));
    }
  }

  /**
   * A constructor for an %RosPublisherSystem that takes ROS message objects on
   * its sole abstract-valued input port. The ROS message type is determined by
   * the template type.
   *
   * @param[in] topic The ROS topic on which to publish.
   *
   * @param node_handle The ROS context.
   *
   * @param publish_period Period that messages will be published (optional).
   * If the publish period is zero, RosPublisherSystem will use per-step
   * publishing instead; see LeafSystem::DeclarePerStepPublishEvent().
   */
  RosPublisherSystem(const std::string& topic, ros::NodeHandle* node_handle,
      double publish_period = 0.0)
      : RosPublisherSystem(topic, node_handle,
          (publish_period > 0.0) ?
          TriggerTypeSet({drake::systems::TriggerType::kForced,
                          drake::systems::TriggerType::kPeriodic}) :
          TriggerTypeSet({drake::systems::TriggerType::kForced,
                          drake::systems::TriggerType::kPerStep}),
          publish_period) {}

  ~RosPublisherSystem() override{};

  const std::string& get_topic_name() const { return topic_; }

  /// Returns the default name for a system that publishes @p topic.
  static std::string make_name(const std::string& topic) {
    return "RosPublisherSystem(" + topic + ")";
  }

  /**
   * Takes the VectorBase from the input port of the context and publishes
   * it onto an ROS topic.
   */
  drake::systems::EventStatus PublishToRosTopic(
      const drake::systems::Context<double>& context) const {
    SPDLOG_TRACE(drake::log(), "Publishing ROS {} message", topic_);

    const drake::AbstractValue* const input_value =
        this->EvalAbstractInput(context, kPortIndex);
    DRAKE_ASSERT(input_value != nullptr);

    publisher_.publish(input_value->get_value<RosMessage>());

    return drake::systems::EventStatus::Succeeded();
  }

 private:
  // The topic on which to publish ROS messages.
  const std::string topic_;

  ros::NodeHandle* const node_handle_{};
  ros::Publisher publisher_;

  const int kPortIndex = 0;
};

}  // namespace systems
}  // namespace dairlib
