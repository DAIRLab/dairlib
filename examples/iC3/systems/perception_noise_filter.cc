#include "perception_noise_filter.h"

#include <iostream>
#include <utility>
#include <random>

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;

namespace dairlib {

using systems::StateVector;

PerceptionNoiseFilter::PerceptionNoiseFilter(bool add_noise)
 : add_noise_(add_noise) { 

  // ASSUMES SINGLE OBJECT
  this->set_name("perception noise filter");
  object_state_input_port_ =
      this->DeclareVectorInputPort(
              "x_object", StateVector<double>(7, 6))
          .get_index();

  object_state_output_port_ =
      this->DeclareVectorOutputPort(
              "x_object_filtered",
              StateVector<double>(7, 6),
              &PerceptionNoiseFilter::OutputNoisyObjectState)
          .get_index();
}

void PerceptionNoiseFilter::OutputNoisyObjectState(
  const drake::systems::Context<double>& context,
  StateVector<double>* output_state) const {
  
	const StateVector<double>* object_state =
			(StateVector<double>*)this->EvalVectorInput(context, object_state_input_port_);

  VectorXd q_object = object_state->GetPositions();
  VectorXd v_object = object_state->GetVelocities();
  DRAKE_DEMAND(q_object.size() == 7);

  if (!add_noise_) {
    output_state->SetPositions(q_object);
    output_state->SetVelocities(v_object);
    return;
  }

  double xy_std = 0.003;
  double z_std = 0.001;
  double angular_std = 0.5;
  double angular_std_rad = angular_std * (M_PI / 180.0);

  std::random_device rd;
  std::mt19937 gen(rd());

  std::normal_distribution<double> dist_xy(0.0, xy_std);
  std::normal_distribution<double> dist_z(0.0, z_std);
  std::normal_distribution<double> dist_rot(0.0, angular_std_rad);

  double x_noise = dist_xy(gen);
  double y_noise = dist_xy(gen);
  double z_noise = dist_z(gen);

  double roll_noise  = dist_rot(gen);
  double pitch_noise = dist_rot(gen);
  double yaw_noise   = dist_rot(gen);

  // Apply rotational noise
  AngleAxisd rollAngle(roll_noise, Vector3d::UnitX());
  AngleAxisd pitchAngle(pitch_noise, Vector3d::UnitY());
  AngleAxisd yawAngle(yaw_noise, Vector3d::UnitZ());
  Quaterniond q_noise = yawAngle * pitchAngle * rollAngle;
  Quaterniond q_curr(q_object(0), q_object(1), q_object(2), q_object(3));

  Quaterniond q_noisy = q_noise * q_curr;

  q_object(0) = q_noisy.w();
  q_object(1) = q_noisy.x();
  q_object(2) = q_noisy.y();
  q_object(3) = q_noisy.z();
  q_object(4) += x_noise;
  q_object(5) += y_noise;
  q_object(6) += z_noise;

  output_state->SetPositions(q_object);
  output_state->SetVelocities(v_object);

}


} // namespace dairlib