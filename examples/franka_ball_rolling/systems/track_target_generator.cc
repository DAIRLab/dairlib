#include "track_target_generator.h"
#include <iostream>


using dairlib::systems::OutputVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::EventStatus;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

TargetGenerator::TargetGenerator(
    const MultibodyPlant<double>& robot_plant,
    const SimulateFrankaParams& sim_param,
    const BallRollingTrajectoryParams& traj_param) {
    // INPUT PORTS
    plant_state_port_ =
    this->DeclareVectorInputPort(
                    "plant_state", OutputVector<double>(robot_plant.num_positions(),
                                                        robot_plant.num_velocities(),
                                                        robot_plant.num_actuators()))
            .get_index();
    // OUTPUT PORTS
    target_port_ =
      this->DeclareVectorOutputPort(
              "track_target", BasicVector<double>(3),
              &TargetGenerator::CalcTrackTarget)
          .get_index();

    // Set Trajectory Patameters
    SetTrajectoryParameters(traj_param);
}

void TargetGenerator::SetTrajectoryParameters(
        const BallRollingTrajectoryParams& traj_param) {
  // Set the target parameters
  // Create class variables for each parameter

  trajectory_type_ = traj_param.trajectory_type;

  /// circle trajectory parameters (general)
  traj_radius_ = traj_param.traj_radius;
  x_c_ = traj_param.x_c;
  y_c_ = traj_param.y_c;
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
}

void TargetGenerator::CalcTrackTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {

  // Evaluate input port for object state
  auto plant_state = (OutputVector<double>*)this->EvalVectorInput(context, plant_state_port_);

  // Get ball position and timestamp
  VectorXd obj_curr_position = plant_state->GetPositions().tail(3);
  double timestamp = plant_state->get_timestamp();
  SimulateFrankaParams sim_param = drake::yaml::LoadYamlFile<SimulateFrankaParams>(
            "examples/franka_ball_rolling/parameters/simulate_franka_params.yaml");
  double object_height = sim_param.ball_radius - sim_param.ground_offset_frame(2);

  // Initialize target pose
  VectorXd target_obj_state = VectorXd::Zero(7);
  VectorXd target_obj_position = VectorXd::Zero(3);

  /// Different trajectory types
  if (trajectory_type_ == 0){
      //  0: time based circle trajectory, velocity_circle_ is angular velocity in degrees/s
      double theta = PI * (timestamp * velocity_circle_ + sim_param.phase) / 180;
      target_obj_position(0) = x_c_ + traj_radius_ * sin(theta);
      target_obj_position(1) = y_c_ + traj_radius_ * cos(theta);
      target_obj_position(2) = object_height;
  }
  else if (trajectory_type_ == 1){
      //  1: state based circle trajectory, assign target angle on the circle using current angle

      // note that the x and y arguments are intentionally flipped
      // since we want to get the angle from the y-axis, not the x-axis
      double x = obj_curr_position(0) - x_c_;
      double y = obj_curr_position(1) - y_c_;
      double curr_angle = atan2(x,y);
      double theta = curr_angle + lead_angle_ * PI / 180;

      target_obj_position(0) = x_c_ + traj_radius_ * sin(theta);
      target_obj_position(1) = y_c_ + traj_radius_ * cos(theta);
      target_obj_position(2) = object_height;
  }
  else if (trajectory_type_ == 2){
      //  2: time based line trajectory, velocity_circle_ is angular velocity in degrees/s

      // calculate the time period from start to end
      double period = abs((start_y_ - end_y_) / velocity_line_);
      int sgn = 0;
      if ((end_y_ - start_y_) > 0){
          sgn = 1;
      }
      else if ((end_y_ - start_y_) < 0) {
          sgn = -1;
      }
      int period_count = int(floor((timestamp / period)));
      double period_time = timestamp - period * period_count;

      // reverse the velocity sign every period then end to start
      if (period_count % 2 == 0){
          // start to end
          double velocity = sgn * velocity_line_;
          target_obj_position(0) = start_x_;
          target_obj_position(1) = start_y_ + sgn * velocity * period_time;
          target_obj_position(2) = object_height;
      }
      else{
          // end to start
          double velocity = sgn * velocity_line_;
          target_obj_position(0) = start_x_;
          target_obj_position(1) = end_y_ - sgn * velocity * period_time;
          target_obj_position(2) = object_height;
      }
  }
  else if (trajectory_type_ == 3){
      //  3: state based line trajectory, velocity_circle_ is angular velocity in degrees/s

      // reverse the direction when it exceeds the target
      if (obj_curr_position(1) < end_y_){
          target_obj_position(0) = start_x_;
          target_obj_position(1) = obj_curr_position(1) + lead_step_;
          target_obj_position(2) = object_height;
      }
  }
  else if(trajectory_type_ == 4){
      // Throw an error.
      std::cerr << ("Trajectory type 4 - n-shaped Path : Currently unimplemented") << std::endl;
      DRAKE_THROW_UNLESS(false);
  }
  else{
      // Throw an error.
      std::cerr << ("Unknown path type") << std::endl;
      DRAKE_THROW_UNLESS(false);
  }
  target_obj_state << 1, 0, 0, 0, target_obj_position;
  target->SetFromVector(target_obj_state);
}
}  // namespace systems
}  // namespace dairlib