#pragma once

#include "systems/sensors/gaussian_noise_pass_through.h"
#include <memory>
#include <utility>
#include <drake/common/eigen_types.h>

namespace dairlib {
namespace systems {

GaussianNoisePassThrough::GaussianNoisePassThrough(
    int num_positions, int num_velocities, int num_inputs,
    const drake::MatrixX<double>& pos_variance,
    const drake::MatrixX<double>& vel_variance)
    : num_positions_(num_positions),
      num_velocities_(num_velocities),
      num_inputs_(num_inputs),
      pos_variance_(pos_variance),
      vel_variance_(vel_variance) {
  systems::OutputVector<double> input(num_positions, num_velocities, num_inputs);
  systems::OutputVector<double> output(num_positions, num_velocities, num_inputs);
  this->DeclareVectorInputPort(input);
  this->DeclareVectorOutputPort(output,
      &GaussianNoisePassThrough::DoCalcVectorOutput);
}

void GaussianNoisePassThrough::DoCalcVectorOutput(
    const drake::systems::Context<double>& context,
    systems::OutputVector<double>* output) const {
  const systems::OutputVector<double>& input =
      *this->template EvalVectorInput<OutputVector>(context, 0);
  output->SetPositions(input.GetPositions() +
      pos_variance_ * Eigen::VectorXd::Random
          (num_positions_));
  output->SetVelocities(input.GetVelocities() +
      vel_variance_ * Eigen::VectorXd::Random(num_velocities_));
  output->SetEfforts(input.GetEfforts());
  output->set_timestamp(input.get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
