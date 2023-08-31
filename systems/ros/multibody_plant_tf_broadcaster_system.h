#pragma once

// ROS includes
#include <vector>
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

class MultibodyPlantTfBroadcasterSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantTfBroadcasterSystem)

  /**
   * Constructor for a system which publishes a ROS tfMessgage for the robot's
   * base link and any other frames which are fixed in the base frame
   * given by the user
   * @param plant MultibodyPlant representing the robot
   * @param context plant context
   * @param frames names of the frame for which the map to frame tf should be broadcast
   * @param base_frame name of the floating base of the robot
   * @param world_frame_name name of the world frame (parent of the base frame)
   * for the ROS  header
   * @param base_fixed_frames list of body fixed frames and their
   * pose in the body frame
   */
  MultibodyPlantTfBroadcasterSystem(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const std::vector<std::string> frames,
      const std::string& base_frame,
      const std::string& world_frame_name,
      const std::vector<
            std::pair<std::string,
                      drake::math::RigidTransformd>>& base_fixed_frames,
      const TriggerTypeSet& publish_triggers,
      double publish_period = 0.0) :
        plant_(plant),
        context_(context),
        base_frame_(base_frame),
        world_frame_(world_frame_name),
        frames_(frames) {

    this->set_name("tf broadcaster for " + plant_.get_name());
    DeclareVectorInputPort("x", plant.num_positions() + plant.num_velocities());

    for (const auto& frame : base_fixed_frames) {
      geometry_msgs::TransformStamped tf =
          tf2::eigenToTransform(frame.second.GetAsIsometry3());
      tf.header.frame_id = base_frame_;
      tf.child_frame_id = frame.first;
      body_fixed_frames_.push_back(tf);
    }

    if (publish_triggers.find(TriggerType::kForced) != publish_triggers.end()) {
      this->DeclareForcedPublishEvent(
          &MultibodyPlantTfBroadcasterSystem::PublishTransforms);
    }

    if (publish_triggers.find(TriggerType::kPeriodic) !=
        publish_triggers.end()) {
      DRAKE_THROW_UNLESS(publish_period > 0.0);
      const double offset = 0.0;
      this->DeclarePeriodicPublishEvent(
          publish_period,
          offset,
          &MultibodyPlantTfBroadcasterSystem::PublishTransforms
      );
    } else {
      // publish_period > 0 without TriggerType::kPeriodic has no meaning and is
      // likely a mistake.
      DRAKE_THROW_UNLESS(publish_period == 0.0);
    }

    if (publish_triggers.find(TriggerType::kPerStep) !=
        publish_triggers.end()) {
      this->DeclarePerStepPublishEvent(
          &MultibodyPlantTfBroadcasterSystem::PublishTransforms
      );
    }
  }

 private:

  drake::systems::EventStatus PublishTransforms(
      const drake::systems::Context<double>& context) const {

    const auto& state = EvalVectorInput(context, 0)->get_value();
    multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, state, context_);

    const auto tnow = ros::Time::now();
    for (const auto& frame : frames_) {
      const auto X_WB = plant_.GetBodyByName(frame).EvalPoseInWorld(*context_);
      geometry_msgs::TransformStamped tbase = tf2::eigenToTransform(X_WB.GetAsIsometry3());
      tbase.header.stamp = tnow;
      tbase.header.frame_id = world_frame_;
      tbase.child_frame_id = frame;
      broadcaster_.sendTransform(tbase);
    }
    for (auto tf : body_fixed_frames_) {
      tf.header.stamp = tnow;
      broadcaster_.sendTransform(tf);
    }

    return drake::systems::EventStatus::Succeeded();
  }

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  const std::string base_frame_;
  const std::vector<std::string> frames_;
  const std::string world_frame_;
  std::vector<geometry_msgs::TransformStamped> body_fixed_frames_{};

  mutable tf2_ros::TransformBroadcaster broadcaster_;

};

}
}




