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
    const MultibodyPlant<double>& object_plant) {
    // INPUT PORTS
    plant_state_port_ =
    this->DeclareVectorInputPort(
                    "plant_state", OutputVector<double>(object_plant.num_positions(),
                                                        object_plant.num_velocities(),
                                                        object_plant.num_actuators()))
            .get_index();
    // OUTPUT PORTS
    target_port_ =
      this->DeclareVectorOutputPort(
              "track_target", BasicVector<double>(3),
              &TargetGenerator::CalcTrackTarget)
          .get_index();
}

void TargetGenerator::SetTrajectoryParameters(
        const int& trajectory_type, const double& traj_radius, const double& x_c, const double& y_c,
        const double& lead_angle, const double& velocity_circle,
        const double& start_x, const double& start_y, const double& end_x, const double& end_y,
        const double& lead_step, const double& velocity_line) {
  // Set the target parameters
  // Create class variables for each parameter

  trajectory_type_ = trajectory_type;

  /// circle trajectory parameters (general)
  traj_radius_ = traj_radius;
  x_c_ = x_c;
  y_c_ = y_c;
  // state based circular specific setting
  lead_angle_ = lead_angle;
  // time based circular specific setting, angular velocity
  velocity_circle_ = velocity_circle;

  /// line trajectory parameters (general)
  start_x_ = start_x;
  start_y_ = start_y;
  end_x_ = end_x;
  end_y_ = end_y;
  // state based line specific setting
  lead_step_ = lead_step;
  // time based line specific setting, velocity
  velocity_line_ = velocity_line;

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
  double ball_height = sim_param.ball_radius - sim_param.ground_offset_frame(2);

  // Initialize target pose
  VectorXd target_obj_state = VectorXd::Zero(7);
  VectorXd target_obj_position = VectorXd::Zero(3);

  /// Different trajectory types
  // 0: time based circle trajectory
//  if (trajectory_type_ == 0){
//      target_obj_position(0) = x_c_ + traj_radius_ * sin(timestamp);
//      target_obj_position(1) = y_c_ + traj_radius_ * cos(theta);
//      target_obj_position(2) = object_half_width_;      // TODO: @bibit Should this be the same as the current position z or some fixed half width?
//  }
//  // 1: state based circle trajectory
//  if (trajectory_type_ == 1){
//      double x = obj_curr_position(0) - x_c_;
//      double y = obj_curr_position(1) - y_c_;
//
//      // note that the x and y arguments are intentionally flipped
//      // since we want to get the angle from the y-axis, not the x-axis
//      double angle = atan2(x,y);
//      double theta = angle + lead_angle_ * PI / 180;
//
//      target_obj_position(0) = x_c_ + traj_radius_ * sin(theta);
//      target_obj_position(1) = y_c_ + traj_radius_ * cos(theta);
//      target_obj_position(2) = object_half_width_;      // TODO: @bibit Should this be the same as the current position z or some fixed half width?
//  }
//  else if (trajectory_type_ == 2){
//      // initializing fixed goal vector that remains constant.
//      VectorXd fixed_goal = VectorXd::Zero(3);
//      fixed_goal(0) = start_x_;
//      fixed_goal(1) = start_y_;
//      fixed_goal(2) = object_half_width_;
//
//      // compute and set next target location for jack to be one step_size in the direction of the fixed goal.
//      VectorXd next_target = obj_curr_position + end_x_ * (fixed_goal - obj_curr_position);
//      target_obj_position(0) = next_target[0];
//      target_obj_position(1) = next_target[1];
//      target_obj_position(2) = next_target[2];
//  }

}

//void TargetGenerator::CalcObjectTarget(
//    const drake::systems::Context<double>& context,
//    BasicVector<double>* target) const {
//  const StateVector<double>* object_state =
//      (StateVector<double>*)this->EvalVectorInput(context, robot_state_port_);
//
//  // Get object position
//  VectorXd obj_curr_position = object_state->GetPositions().tail(3);
//
//  VectorXd target_obj_state = VectorXd::Zero(7);
//  VectorXd target_obj_position = VectorXd::Zero(3);
//
//  // Adaptive circular trajectory for traj_type = 1
//  if (trajectory_type_ == 1){
//    double x = obj_curr_position(0) - x_c_;
//    double y = obj_curr_position(1) - y_c_;
//
//    // note that the x and y arguments are intentionally flipped
//    // since we want to get the angle from the y-axis, not the x-axis
//    double angle = atan2(x,y);
//    double theta = angle + lead_angle_ * PI / 180;
//
//    target_obj_position(0) = x_c_ + traj_radius_ * sin(theta);
//    target_obj_position(1) = y_c_ + traj_radius_ * cos(theta);
//    target_obj_position(2) = object_half_width_;      // TODO: @bibit Should this be the same as the current position z or some fixed half width?
//  }
//  // Use a fixed goal if trajectory_type is 2.
//  else if (trajectory_type_ == 2){
//    // initializing fixed goal vector that remains constant.
//    VectorXd fixed_goal = VectorXd::Zero(3);
//    fixed_goal(0) = start_x_;
//    fixed_goal(1) = start_y_;
//    fixed_goal(2) = object_half_width_;
//
//    // compute and set next target location for jack to be one step_size in the direction of the fixed goal.
//    VectorXd next_target = obj_curr_position + end_x_ * (fixed_goal - obj_curr_position);
//    target_obj_position(0) = next_target[0];
//    target_obj_position(1) = next_target[1];
//    target_obj_position(2) = next_target[2];
//  }
//   // Use a straight line trajectory with adaptive next goal if trajectory type is 3.
//  else if(trajectory_type_ == 3){
//    VectorXd start_point = VectorXd::Zero(3);
//    VectorXd end_point = VectorXd::Zero(3);
//    // define the start and end points for the straight line.
//    start_point[0] = end_y_;
//    start_point[1] = start_point_y_;
//    start_point[2] = object_half_width_;
//
//    end_point[0] = end_point_x_;
//    end_point[1] = end_point_y_;
//    end_point[2] = object_half_width_;
//
//    // compute vector from start point to end point
//    VectorXd distance_vector = end_point - start_point;
//    // project current jack location onto straight line vector.
//    double projection_length = (obj_curr_position - start_point).dot(distance_vector)/distance_vector.norm();
//    VectorXd projection_point = start_point + projection_length*(distance_vector/distance_vector.norm());
//
//    // Step ahead along the path to the goal from the projection point, without overshooting past the goal.
//    VectorXd target_on_line_with_lookahead;
//    if ((obj_curr_position - end_point).norm() < lookahead_step_size_) {
//      target_on_line_with_lookahead = end_point;
//    }
//    else {
//      VectorXd step_vector =lookahead_step_size_ * distance_vector/distance_vector.norm();
//      target_on_line_with_lookahead = projection_point + step_vector;
//    }
//
//    // compute error vector between projection point and current jack location
//    VectorXd error_vector = target_on_line_with_lookahead - obj_curr_position;
//
//    // declaring next_target vector
//    VectorXd next_target;
//    if (error_vector.norm() >= max_step_size_){
//        // if jack is max step size or further from the projection point, the next target will be one step size in that direction.
//        next_target = obj_curr_position + max_step_size_*(error_vector/error_vector.norm());
//    }
//    else{
//        // else set the next target to be the projection point.
//        next_target = target_on_line_with_lookahead;
//    }
//    // set next target location for jack
//    target_obj_position(0) = next_target[0];
//    target_obj_position(1) = next_target[1];
//    target_obj_position(2) = next_target[2];
//  }
//
//  else if(trajectory_type_ == 0){
//        // Throw an error.
//    std::cerr << ("Trajectory type 0 - Time Based Path : Currently unimplemented") << std::endl;
//    DRAKE_THROW_UNLESS(false);
//  }
//  else{
//    // Throw an error.
//    std::cerr << ("Unknown path type") << std::endl;
//    DRAKE_THROW_UNLESS(false);
//  }

//  target_obj_state << 1, 0, 0, 0, target_obj_position;  // The first four used to be the quaternion for a flat tray. Does this need to change @Bibit?
//  target->SetFromVector(target_obj_state);
//}

}  // namespace systems
}  // namespace dairlib