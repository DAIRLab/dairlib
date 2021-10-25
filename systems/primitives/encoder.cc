#include "systems/primitives/encoder.h"

#include <drake/common/eigen_types.h>

namespace dairlib {
namespace systems {

using Eigen::VectorXd;

Encoder::Encoder(const drake::multibody::MultibodyPlant<double>& plant,
                 std::vector<int>& joint_indices,
                 std::vector<int>& ticks_per_revolution)
    : num_positions_(plant.num_positions()),
      num_velocities_(plant.num_velocities()),
      joint_indices_(joint_indices),
      ticks_per_revolution_(ticks_per_revolution),
      joint_filters_(std::vector<JointFilter>(joint_indices_.size())) {
  this->DeclareVectorInputPort("q, v",
      systems::BasicVector<double>(num_positions_ + num_velocities_));
  this->DeclareVectorOutputPort("q, v",
      systems::BasicVector<double>(num_positions_ + num_velocities_),
      &Encoder::UpdateFilter);
}

void Encoder::UpdateFilter(const drake::systems::Context<double>& context,
                           systems::BasicVector<double>* output) const {
  const systems::BasicVector<double>& input =
      *this->template EvalVectorInput<BasicVector>(context, 0);

  VectorXd q = input.get_value().head(num_positions_);
  VectorXd v = input.get_value().head(num_velocities_);

  VectorXd q_filtered = q;
  VectorXd v_filtered = v;

  // joint_encoders
  for (int joint_index = 0; joint_index < joint_indices_.size();
       ++joint_index) {
    JointFilter filter = joint_filters_[joint_index];

    // Position
    using std::floor;
    const double ticks_per_radian =
        ticks_per_revolution_[joint_index] / (2.0 * M_PI);
    q_filtered[joint_indices_[joint_index]] =
        floor(q[joint_indices_[joint_index]] * ticks_per_radian) /
        ticks_per_radian;

    // Velocity
    // Initialize unfiltered signal array to prevent bad transients
    bool allzero = true;
    for (int i = 0; i < JOINT_FILTER_NB; ++i) {
      allzero &= filter.x[i] == 0;
    }
    if (allzero) {
      // If all filter values are zero, initialize the signal array
      // with the current encoder value
      for (int i = 0; i < JOINT_FILTER_NB; ++i) {
        filter.x[i] = q_filtered[joint_indices_[joint_index]];
      }
    }

    // Shift and update signal arrays
    for (int i = JOINT_FILTER_NB - 1; i > 0; --i) {
      filter.x[i] = filter.x[i - 1];
    }
    filter.x[0] = q[joint_indices_[joint_index]];
    for (int i = JOINT_FILTER_NA - 1; i > 0; --i) {
      filter.y[i] = filter.y[i - 1];
    }

    // Compute filter value
    filter.y[0] = 0;
    for (int i = 0; i < JOINT_FILTER_NB; ++i) {
      filter.y[0] += filter.x[i] * joint_filter_b[i];
    }
    for (int i = 1; i < JOINT_FILTER_NA; ++i) {
      filter.y[0] -= filter.y[i] * joint_filter_a[i];
    }
    v_filtered[joint_indices_[joint_index]] = filter.y[0];
  }

  VectorXd x_filtered = VectorXd::Zero(num_positions_ + num_velocities_);
  x_filtered << q_filtered, v_filtered;
  output->set_value(x_filtered);
}

}  // namespace systems
}  // namespace dairlib
