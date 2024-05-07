#include "track_target_generator.h"

#include <iostream>

using dairlib::systems::OutputVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::State;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

TargetGenerator::TargetGenerator(
    const MultibodyPlant<double>& lcs_plant,
    const SimulateFrankaParams& sim_param,
    const BallRollingTrajectoryParams& traj_param) {
  // INPUT PORTS
  plant_state_port_ =
      this->DeclareVectorInputPort("lcs_state", TimestampedVector<double>(
                                                    lcs_plant.num_positions() +
                                                    lcs_plant.num_velocities()))
          .get_index();
  // OUTPUT PORTS
  // TODO:: make dimension not hardcoded
  target_port_ = this->DeclareVectorOutputPort(
                         "object_position_target", BasicVector<double>(7),
                         &TargetGenerator::CalcTrackTarget)
                     .get_index();

  // Set Trajectory Patameters
  SetTrajectoryParameters(sim_param, traj_param);

  first_message_time_idx_ = this->DeclareAbstractState(drake::Value<double>(0));
  received_first_message_idx_ =
      this->DeclareAbstractState(drake::Value<bool>(false));

  this->DeclarePerStepUnrestrictedUpdateEvent(
      &TargetGenerator::UpdateFirstMessageTime);

  n_q = lcs_plant.num_positions();
  n_v = lcs_plant.num_velocities();
  n_x = n_q + n_v;
}

EventStatus TargetGenerator::UpdateFirstMessageTime(
    const Context<double>& context, State<double>* state) const {
  auto& received_first_message =
      state->get_mutable_abstract_state<bool>(received_first_message_idx_);
  auto& first_message_time =
      state->get_mutable_abstract_state<double>(first_message_time_idx_);

  if (!received_first_message) {
    auto plant_state = (TimestampedVector<double>*)this->EvalVectorInput(
        context, plant_state_port_);
    double timestamp = plant_state->get_timestamp();
    received_first_message = true;
    first_message_time = timestamp;
    return EventStatus::Succeeded();
  }
  return EventStatus::Succeeded();
}

void TargetGenerator::SetTrajectoryParameters(
    const SimulateFrankaParams& sim_param,
    const BallRollingTrajectoryParams& traj_param) {
  // Set the target parameters
  // Create class variables for each parameter

  trajectory_type_ = traj_param.trajectory_type;

  /// circle trajectory parameters (general)
  traj_radius_ = traj_param.traj_radius;
  x_c_ = traj_param.x_c;
  y_c_ = traj_param.y_c;
  initial_phase_ = sim_param.phase;
  // state based circular specific setting
  lead_angle_ = traj_param.lead_angle;
  // time based circular specific setting, angular velocity
  velocity_circle_ = traj_param.velocity_circle;

  /// line trajectory parameters (general)
  start_x_ = traj_param.start_x;
  start_y_ = traj_param.start_y;
  end_x_ = traj_param.end_x;
  end_y_ = traj_param.end_y;
  // state based line specific setting
  lead_step_ = traj_param.lead_step;
  // time based line specific setting, velocity
  velocity_line_ = traj_param.velocity_line;

  /// object height should be fixed since it is on the table
  object_height_ = sim_param.ball_radius + sim_param.ground_offset_frame(2);
}

void TargetGenerator::CalcTrackTarget(const Context<double>& context,
                                      BasicVector<double>* target) const {
  // Evaluate input port for object state
  auto plant_state = (TimestampedVector<double>*)this->EvalVectorInput(
      context, plant_state_port_);

  // Get timestamp
  VectorXd lcs_state = plant_state->get_data();
  double timestamp = plant_state->get_timestamp();
  auto first_message_time =
      context.get_abstract_state<double>(first_message_time_idx_);
  double curr_time = timestamp - first_message_time;

  // Get ball position
  VectorXd obj_curr_position = lcs_state.head(n_q).tail(3);

  // Initialize target pose
  VectorXd target_obj_state = VectorXd::Zero(7);
  VectorXd target_obj_position = VectorXd::Zero(3);

  /// Different trajectory types
  if (trajectory_type_ == 0) {
    //  0: time based circle trajectory, velocity_circle_ is angular velocity in
    //  degrees/s
    double theta = PI * (curr_time * velocity_circle_ + initial_phase_) / 180;
    target_obj_position(0) = x_c_ + traj_radius_ * sin(theta);
    target_obj_position(1) = y_c_ + traj_radius_ * cos(theta);
    target_obj_position(2) = object_height_;
  } else if (trajectory_type_ == 1) {
    //  1: state based circle trajectory, assign target angle on the circle
    //  using current angle

    // note that the x and y arguments are intentionally flipped
    // since we want to get the angle from the y-axis, not the x-axis
    double x = obj_curr_position(0) - x_c_;
    double y = obj_curr_position(1) - y_c_;
    double curr_angle = atan2(x, y);
    double theta = curr_angle + lead_angle_ * PI / 180;

    target_obj_position(0) = x_c_ + traj_radius_ * sin(theta);
    target_obj_position(1) = y_c_ + traj_radius_ * cos(theta);
    target_obj_position(2) = object_height_;
  } else if (trajectory_type_ == 2) {
    //  2: time based line trajectory, velocity_circle_ is angular velocity in
    //  degrees/s

    // calculate the time period from start to end
    double period = abs((start_y_ - end_y_) / velocity_line_);
    int sgn = 0;
    if ((end_y_ - start_y_) > 0) {
      sgn = 1;
    } else if ((end_y_ - start_y_) < 0) {
      sgn = -1;
    }
    int period_count = int(floor((curr_time / period)));
    double period_time = curr_time - period * period_count;

    // reverse the velocity sign every period then end to start
    if (period_count % 2 == 0) {
      // start to end
      double velocity = sgn * velocity_line_;
      target_obj_position(0) = start_x_;
      target_obj_position(1) = start_y_ + sgn * velocity * period_time;
      target_obj_position(2) = object_height_;
    } else {
      // end to start
      double velocity = sgn * velocity_line_;
      target_obj_position(0) = start_x_;
      target_obj_position(1) = end_y_ - sgn * velocity * period_time;
      target_obj_position(2) = object_height_;
    }
  } else if (trajectory_type_ == 3) {
    //  3: state based line trajectory, velocity_circle_ is angular velocity in
    //  degrees/s

    // reverse the direction when it exceeds the target
    if (obj_curr_position(1) < end_y_) {
      target_obj_position(0) = start_x_;
      target_obj_position(1) = obj_curr_position(1) + lead_step_;
      target_obj_position(2) = object_height_;
    }
  } else if (trajectory_type_ == 4) {
    // Throw an error.
    std::cerr << ("Trajectory type 4 - n-shaped Path : Currently unimplemented")
              << std::endl;
    DRAKE_THROW_UNLESS(false);
  } else {
    // Throw an error.
    std::cerr << ("Unknown path type") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }
  target_obj_state << 1, 0, 0, 0, target_obj_position;
  target->SetFromVector(target_obj_state);
}
}  // namespace systems
}  // namespace dairlib