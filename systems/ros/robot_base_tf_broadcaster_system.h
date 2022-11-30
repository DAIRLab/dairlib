#pragma once

// ROS includes
#include "ros/ros.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_broadcaster.h"

// drake includes
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

// Dairlib includes
#include "multibody/multibody_utils.h"

namespace dairlib{
namespace systems{

using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;

class RobotBaseTfBroadcasterSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotBaseTfBroadcasterSystem)

  /**
   * Constructor for a system which publishes a ROS tfMessgage for the robot's
   * base link and any other frames which are fixed in the base frame
   * given by the user
   * @param plant MultibodyPlant representing the robot
   * @param context plant context
   * @param base_frame name of the frame for which the tf should be constructed
   * @param world_frame name of the world frame (parent of the base frame
   * for the ROS  header
   * @param body_fixed_frames list of body fixed frames and their
   * pose in the body frame
   */
  RobotBaseTfBroadcasterSystem(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const std::string& base_frame,
      const std::string& world_frame,
      const std::vector<
            std::pair<std::string,
                      drake::math::RigidTransformd>>&body_fixed_frames,
      const TriggerTypeSet& publish_triggers,
      double publish_period = 0.0) :
        plant_(plant),
        context_(context),
        base_frame_(base_frame),
        world_frame_(world_frame) {

    this->set_name("tf braodcaster");
    DeclareVectorInputPort("x", plant.num_positions() + plant.num_velocities());

    for (const auto& frame : body_fixed_frames) {
      geometry_msgs::TransformStamped tf =
          tf2::eigenToTransform(frame.second.GetAsIsometry3());
      tf.header.frame_id = base_frame_;
      tf.child_frame_id = frame.first;
      body_fixed_frames_.push_back(tf);
    }

    // Declare a forced publish so that any time Publish(.) is called on this
    // system (or a Diagram containing it), a message is emitted.
    if (publish_triggers.find(TriggerType::kForced) != publish_triggers.end()) {
      this->DeclareForcedPublishEvent(
          &RobotBaseTfBroadcasterSystem::PublishTransforms);
    }

    if (publish_triggers.find(TriggerType::kPeriodic) !=
        publish_triggers.end()) {
      DRAKE_THROW_UNLESS(publish_period > 0.0);
      const double offset = 0.0;
      this->DeclarePeriodicPublishEvent(
          publish_period,
          offset,
          &RobotBaseTfBroadcasterSystem::PublishTransforms
      );
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
            this->PublishTransforms(context);
          }));
    }
  }

 private:

  drake::systems::EventStatus PublishTransforms(
      const drake::systems::Context<double>& context) const {

    const auto& state = EvalVectorInput(context, 0)->get_value();
    multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, state, context_);

    const auto X_WB = plant_.GetBodyByName(base_frame_).EvalPoseInWorld(*context_);
    const auto tnow = ros::Time::now();

    geometry_msgs::TransformStamped tbase = tf2::eigenToTransform(X_WB.GetAsIsometry3());

    tbase.header.stamp = tnow;
    tbase.header.frame_id = world_frame_;
    tbase.child_frame_id = base_frame_;

    broadcaster_.sendTransform(tbase);
    for (auto tf : body_fixed_frames_) {
      tf.header.stamp = tnow;
      broadcaster_.sendTransform(tf);
    }
    return drake::systems::EventStatus::Succeeded();
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  const std::string base_frame_;
  const std::string world_frame_;
  std::vector<geometry_msgs::TransformStamped> body_fixed_frames_{};

  mutable tf2_ros::TransformBroadcaster broadcaster_;

};

}
}




