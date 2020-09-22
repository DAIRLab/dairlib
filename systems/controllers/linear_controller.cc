#include "systems/controllers/linear_controller.h"

namespace dairlib {
namespace systems {

LinearController::LinearController(
    int num_positions, int num_velocities, int num_inputs,
    const drake::multibody::MultibodyPlant<double>& plant) {
  output_input_port_ =
      this->DeclareVectorInputPort(
              OutputVector<double>(num_positions, num_velocities, num_inputs))
          .get_index();

  config_input_port_ =
      this->DeclareVectorInputPort(
              LinearConfig(num_positions + num_velocities, num_inputs))
          .get_index();

  this->DeclareVectorOutputPort(TimestampedVector<double>(num_inputs),
                                &LinearController::CalcControl);

  lcm_output_port_ =
      this->DeclareAbstractOutputPort(&LinearController::AssignLcmOutput)
          .get_index();

  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();
  num_efforts_ = plant.num_actuators();

  positionIndexMap_ = multibody::makeNameToPositionsMap(plant);
  velocityIndexMap_ = multibody::makeNameToVelocitiesMap(plant);
  effortIndexMap_ = multibody::makeNameToActuatorsMap(plant);

  // Loop through the maps to extract ordered names
  for (int i = 0; i < num_positions_; ++i) {
    for (auto& x : positionIndexMap_) {
      if (x.second == i) {
        ordered_position_names_.push_back(x.first);
        break;
      }
    }
  }

  for (int i = 0; i < num_velocities_; ++i) {
    for (auto& x : velocityIndexMap_) {
      if (x.second == i) {
        ordered_velocity_names_.push_back(x.first);
        break;
      }
    }
  }

  for (int i = 0; i < num_efforts_; i++) {
    for (auto& x : effortIndexMap_) {
      if (x.second == i) {
        ordered_effort_names_.push_back(x.first);
        break;
      }
    }
  }
}

void LinearController::CalcControl(const Context<double>& context,
                                   TimestampedVector<double>* control) const {
  const OutputVector<double>* output =
      (OutputVector<double>*)this->EvalVectorInput(context, output_input_port_);

  const LinearConfig* config = dynamic_cast<const LinearConfig*>(
      this->EvalVectorInput(context, config_input_port_));
  VectorXd u =
      config->GetK() * (config->GetDesiredState() - output->GetState());

//  std::cout << "xd = " << config->GetDesiredState().transpose() << std::endl;
//  std::cout << "x = " << output->GetState().transpose() << std::endl;
//  std::cout << "u = " << u.transpose() << std::endl;
  //    std::cout << "u_fb = " << output->GetEfforts().transpose() << std::endl;

  control->SetDataVector(u);
  control->set_timestamp(output->get_timestamp());
}

void LinearController::AssignLcmOutput(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_pd_control* msg) const {
  const OutputVector<double>* state =
      (OutputVector<double>*)this->EvalVectorInput(context, output_input_port_);

  const LinearConfig* config = dynamic_cast<const LinearConfig*>(
      this->EvalVectorInput(context, config_input_port_));
  VectorXd u =
      config->GetK() * (config->GetDesiredState() - state->GetState());

  // using the time from the context
  msg->utime = context.get_time() * 1e6;

  msg->num_positions = num_positions_;
  msg->num_velocities = num_velocities_;
  msg->position_names.resize(num_positions_);
  msg->velocity_names.resize(num_velocities_);
  msg->position_feedback.resize(num_positions_);
  msg->position_desired.resize(num_positions_);
  msg->velocity_feedback.resize(num_velocities_);
  msg->velocity_desired.resize(num_velocities_);

  for (int i = 0; i < num_positions_; i++) {
    msg->position_feedback[i] = state->GetAtIndex(i);
    msg->position_desired[i] = config->GetDesiredState()(i);
    msg->position_names[i] = ordered_position_names_[i];
  }
  for (int i = 0; i < num_velocities_; i++) {
    msg->velocity_feedback[i] = state->GetAtIndex(num_positions_ + i);
    msg->velocity_desired[i] = config->GetDesiredState()(num_positions_ + i);
    msg->velocity_names[i] = ordered_velocity_names_[i];
  }

  msg->num_efforts = num_efforts_;
  msg->effort_names.resize(num_efforts_);
  msg->effort.resize(num_efforts_);

  for (int i = 0; i < num_efforts_; i++) {
    msg->effort[i] = u(i);
    msg->effort_names[i] = ordered_effort_names_[i];
  }
}

}  // namespace systems
}  // namespace dairlib