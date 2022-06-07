#include "examples/Cassie/systems/cassie_encoder.h"

#include <drake/common/eigen_types.h>

#include "multibody/multibody_utils.h"

namespace dairlib {

using Eigen::VectorXd;

CassieEncoder::CassieEncoder(
    const drake::multibody::MultibodyPlant<double>& plant)
    : num_positions_(plant.num_positions()),
      num_velocities_(plant.num_velocities()) {
  auto pos_map = multibody::MakeNameToPositionsMap(plant);
  auto vel_map = multibody::MakeNameToVelocitiesMap(plant);

  for (int i = 0; i < plant.num_joints(); ++i) {
    auto& joint = plant.get_joint(drake::multibody::JointIndex(i));
    if (joint_encoder_resolutions.count(joint.name())) {
      auto filter = std::make_shared<JointFilter>();
      filter->joint_pos_index = pos_map[joint.name()];
      filter->joint_vel_index = vel_map[joint.name() + "dot"];
      filter->joint_encoder_resolution =
          joint_encoder_resolutions.at(joint.name());
      joint_filters_.push_back(filter);
    }
    if (drive_encoder_resolutions.count(joint.name())) {
      auto filter = std::make_shared<DriveFilter>();
      filter->drive_pos_index = pos_map[joint.name()];
      filter->drive_vel_index = vel_map[joint.name() + "dot"];
      filter->drive_encoder_resolution =
          drive_encoder_resolutions.at(joint.name());
      filter->gear_ratio = drive_gear_ratios.at(joint.name());
      drive_filters_.push_back(filter);
    }
  }
  this->DeclareVectorInputPort(
      "robot_state",
      systems::BasicVector<double>(num_positions_ + num_velocities_));
  this->DeclareVectorOutputPort(
      "filtered_state",
      systems::BasicVector<double>(num_positions_ + num_velocities_),
      &CassieEncoder::UpdateFilter);
}

void CassieEncoder::UpdateFilter(const drake::systems::Context<double>& context,
                                 systems::BasicVector<double>* output) const {
  const systems::BasicVector<double>& x =
      *this->template EvalVectorInput<systems::BasicVector>(context, 0);

  VectorXd q = x.value().head(num_positions_);
  VectorXd v = x.value().tail(num_velocities_);

  VectorXd q_filtered = q;
  VectorXd v_filtered = v;

  // drive encoders
  for (const auto& filter : drive_filters_) {
    // Position
    using std::floor;
    double ratio = filter->gear_ratio;
    double scale = (2.0 * M_PI) / filter->drive_encoder_resolution / ratio;
    const int encoder_value = q[filter->drive_pos_index] * ratio /
                              (2.0 * M_PI) * filter->drive_encoder_resolution;
    q_filtered[filter->drive_pos_index] = encoder_value * scale;

    // Velocity
    // Initialize unfiltered signal array to prevent bad transients
    bool allzero = true;
    for (int i = 0; i < CASSIE_JOINT_FILTER_NB; ++i) {
      allzero &= filter->x[i] == 0;
    }
    if (allzero) {
      // If all filter values are zero, initialize the signal array
      // with the current encoder value
      for (int i = 0; i < CASSIE_JOINT_FILTER_NB; ++i) {
        filter->x[i] = encoder_value;
      }
    }

    // Shift and update unfiltered signal array
    for (int i = DRIVE_FILTER_NB - 1; i > 0; --i) {
      filter->x[i] = filter->x[i - 1];
    }
    filter->x[0] = encoder_value;
    // Compute filter value
    int y = 0;
    for (int i = 0; i < DRIVE_FILTER_NB; ++i) {
      y += filter->x[i] * drive_filter_b[i];
    }
    v_filtered[filter->drive_vel_index] = y * scale / M_PI;
  }

  // joint encoders
  for (const auto& filter : joint_filters_) {
    // Position
    using std::floor;
    const double ticks_per_radian =
        filter->joint_encoder_resolution / (2.0 * M_PI);
    q_filtered[filter->joint_pos_index] =
        floor(q[filter->joint_pos_index] * ticks_per_radian) / ticks_per_radian;

    // Velocity
    // Initialize unfiltered signal array to prevent bad transients
    bool allzero = true;
    for (int i = 0; i < CASSIE_JOINT_FILTER_NB; ++i) {
      allzero &= filter->x[i] == 0;
    }
    if (allzero) {
      // If all filter values are zero, initialize the signal array
      // with the current encoder value
      for (int i = 0; i < CASSIE_JOINT_FILTER_NB; ++i) {
        filter->x[i] = q_filtered[filter->joint_pos_index];
      }
    }

    // Shift and update signal arrays
    for (int i = CASSIE_JOINT_FILTER_NB - 1; i > 0; --i) {
      filter->x[i] = filter->x[i - 1];
    }
    filter->x[0] = q_filtered[filter->joint_pos_index];
    for (int i = CASSIE_JOINT_FILTER_NA - 1; i > 0; --i) {
      filter->y[i] = filter->y[i - 1];
    }

    // Compute filter value
    filter->y[0] = 0;
    for (int i = 0; i < CASSIE_JOINT_FILTER_NB; ++i) {
      filter->y[0] += filter->x[i] * joint_filter_b[i];
    }
    for (int i = 1; i < CASSIE_JOINT_FILTER_NA; ++i) {
      filter->y[0] -= filter->y[i] * joint_filter_a[i];
    }
    v_filtered[filter->joint_vel_index] = filter->y[0];
  }
  VectorXd x_filtered = VectorXd::Zero(num_positions_ + num_velocities_);
  x_filtered << q_filtered, v_filtered;
  output->set_value(x_filtered);
}

}  // namespace dairlib
