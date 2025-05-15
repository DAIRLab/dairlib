#include "control_target_generator.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include <drake/common/yaml/yaml_io.h>
#include "examples/sampling_c3/parameter_headers/franka_c3_controller_params.h"

#include "lcm/lcm_trajectory.h"

using Eigen::VectorXd;

namespace dairlib {
namespace systems {

    TargetGenerator::TargetGenerator(
        const drake::multibody::MultibodyPlant<double>& object_plant) {
      // INPUT PORTS
      radio_port_ =
          this->DeclareAbstractInputPort("lcmt_radio_out",
                                         drake::Value<dairlib::lcmt_radio_out>{})
              .get_index();
    
      object_state_port_ =
          this->DeclareVectorInputPort(
                  "x_object", StateVector<double>(object_plant.num_positions(),
                                                  object_plant.num_velocities()))
              .get_index();
    
      // OUTPUT PORTS
      end_effector_target_port_ = this->DeclareVectorOutputPort(
                                  "end_effector_target", BasicVector<double>(3),
                                  &TargetGenerator::CalcEndEffectorTarget)
                              .get_index();
      object_target_port_ = this->DeclareVectorOutputPort(
                                  "object_target", BasicVector<double>(7),
                                  &TargetGenerator::CalcObjectTarget)
                              .get_index();
      object_velocity_target_port_ = this->DeclareVectorOutputPort(
                                  "object_velocity_target", BasicVector<double>(6),
                                  &TargetGenerator::CalcObjectVelocityTarget)
                              .get_index();
      object_final_target_port_ = this->DeclareVectorOutputPort(
                                  "object_final_target", BasicVector<double>(7),
                                  &TargetGenerator::OutputObjectFinalTarget)
                              .get_index();
      target_gen_info_port_ = this->DeclareAbstractOutputPort(
                                "target_generator_info",
                                dairlib::lcmt_timestamped_saved_traj(),
                                &TargetGenerator::OutputTargetGeneratorInfo)
                                  .get_index();
    
      C3Options c3_options;
      FrankaC3ControllerParams controller_params =
          drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(
            "examples/sampling_c3/jacktoy/parameters/franka_c3_controller_params.yaml");
    
      c3_options = drake::yaml::LoadYamlFile<C3Options>(
                    controller_params.c3_options_file);
    }
    
    void TargetGenerator::SetRemoteControlParameters(
        const int& trajectory_type,
        const bool& use_changing_final_goal,
        const int& changing_final_goal_type,
        const bool& prevent_three_topples_for_random_goal_gen,
        const double& traj_radius,
        const double& x_c, const double& y_c, const double& lead_angle,
        const Eigen::VectorXd& target_object_position,
        const Eigen::VectorXd& target_object_orientation,
        const double& step_size, const double& start_point_x,
        const double& start_point_y,
        const double& end_point_x,
        const double& end_point_y,
        const double& lookahead_step_size,
        const double& lookahead_angle,
        const double& angle_hysteresis,
        const double& angle_err_to_vel_factor,
        const double& max_step_size,
        const double& ee_goal_height,
        const double& object_half_width,
        const double& position_success_threshold,
        const double& orientation_success_threshold,
        const Eigen::VectorXd& random_goal_x_limits,
        const Eigen::VectorXd& random_goal_y_limits,
        const Eigen::VectorXd& random_goal_radius_limits,
        const double& resting_object_height) {
      // Set the target parameters
      // Create class variables for each parameter
      trajectory_type_ = trajectory_type;
      use_changing_final_goal_ = use_changing_final_goal;
      changing_goal_type_ = static_cast<ChangingGoalType>(changing_final_goal_type);
      prevent_three_topples_ = prevent_three_topples_for_random_goal_gen;
      traj_radius_ = traj_radius;
      x_c_ = x_c;
      y_c_ = y_c;
      lead_angle_ = lead_angle;
      target_final_object_position_ = target_object_position;
      target_final_object_orientation_ = target_object_orientation;
      step_size_ = step_size;
      start_point_x_ = start_point_x;
      start_point_y_ = start_point_y;
      end_point_x_ = end_point_x;
      end_point_y_ = end_point_y;
      lookahead_step_size_ = lookahead_step_size;
      lookahead_angle_ = lookahead_angle;
      angle_hysteresis_ = angle_hysteresis;
      angle_err_to_vel_factor_ = angle_err_to_vel_factor;
      max_step_size_ = max_step_size;
      ee_goal_height_ = ee_goal_height;
      object_half_width_ = object_half_width;
      position_success_threshold_ = position_success_threshold;
      orientation_success_threshold_ = orientation_success_threshold;
      random_goal_x_limits_ = random_goal_x_limits;
      random_goal_y_limits_ = random_goal_y_limits;
      random_goal_radius_limits_ = random_goal_radius_limits;
      resting_object_height_ = resting_object_height;
    }
    
    void TargetGenerator::CalcEndEffectorTarget(
        const drake::systems::Context<double>& context,
        drake::systems::BasicVector<double>* target) const {
      // Evaluate input port for object state
      const StateVector<double>* object_state =
        (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);
    
      // End effector position needs to be right above the object
      VectorXd end_effector_position = object_state->GetPositions().tail(3);
      // Fixing ee target z to be above the object at a fixed height
      end_effector_position[2] += ee_goal_height_;
    
      target->SetFromVector(end_effector_position);
    }

    void TargetGenerator::CalcObjectTarget(
        const drake::systems::Context<double>& context,
        BasicVector<double>* target) const {
      const StateVector<double>* object_state =
          (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);
    
      // Get current object states.
      VectorXd obj_curr_position = object_state->GetPositions().tail(3);
      VectorXd obj_curr_quat = object_state->GetPositions().head(4);
    
      // Prepare to set target object states.
      VectorXd target_obj_state = VectorXd::Zero(7);
      VectorXd target_obj_position = VectorXd::Zero(3);
      VectorXd target_obj_orientation = VectorXd::Zero(4);
    
      // Define quaternion objects for current and desired orientations.
      Eigen::Quaterniond curr_quat(obj_curr_quat(0), obj_curr_quat(1),
                                   obj_curr_quat(2), obj_curr_quat(3));
      Eigen::Quaterniond des_quat(target_final_object_orientation_[0],
                                  target_final_object_orientation_[1],
                                  target_final_object_orientation_[2],
                                  target_final_object_orientation_[3]);
      // Compute the orientation difference.
      Eigen::AngleAxis<double> angle_axis_diff(des_quat * curr_quat.inverse());
    
      // Handle changing goal if success criteria are met for both position and
      // orientation.
      if (use_changing_final_goal_) {
        // Check if success has been met.
        double object_position_error = (obj_curr_position -
          target_final_object_position_).norm();
        double object_angular_error = angle_axis_diff.angle();
    
        if ((object_position_error < position_success_threshold_) &&
            (object_angular_error < orientation_success_threshold_)) {
          std::cout << "\nMet pose goal!\n" << std::endl;
    
          // Reset the target object orientation and position.
          if (changing_goal_type_ == CHANGING_GOAL_RANDOM) {
            SetRandomizedTargetFinalObjectPosition();
            SetRandomizedTargetFinalObjectOrientation();
          }
          else if (changing_goal_type_ == CHANGING_GOAL_ORIENTATION_SEQUENCE) {
            // Set the next orientation in the sequence.
            this->CycleThroughOrientationSequence();
          }
          else {
            std::cerr << "Invalid changing goal type." << std::endl;
          }
          goal_counter_++;
        }
    
      }
    
      target_obj_orientation = target_final_object_orientation_;
    
      // Adaptive circular trajectory for traj_type = 1
      if (trajectory_type_ == 1){
        double x = obj_curr_position(0) - x_c_;
        double y = obj_curr_position(1) - y_c_;
    
        // note that the x and y arguments are intentionally flipped
        // since we want to get the angle from the y-axis, not the x-axis
        double angle = atan2(x,y);
        double theta = angle + lead_angle_ * PI / 180;
    
        target_obj_position(0) = x_c_ + traj_radius_ * sin(theta);
        target_obj_position(1) = y_c_ + traj_radius_ * cos(theta);
        target_obj_position(2) = object_half_width_;
      }
      // Use a fixed goal if trajectory_type is 2.
      else if (trajectory_type_ == 2){
        // initializing fixed goal vector that remains constant.
        if ((target_final_object_position_ - obj_curr_position).norm() < step_size_)
        {
          // if the jack is within one step size of the fixed goal, set the target
          // to be the fixed goal.
          target_obj_position(0) = target_final_object_position_[0];
          target_obj_position(1) = target_final_object_position_[1];
          target_obj_position(2) = target_final_object_position_[2];
        }
        else{
          // compute and set next target location for jack to be one step_size in
          // the direction of the fixed goal.
          VectorXd next_target = obj_curr_position + step_size_ * (
            target_final_object_position_ - obj_curr_position);
          target_obj_position(0) = next_target[0];
          target_obj_position(1) = next_target[1];
          target_obj_position(2) = next_target[2];
        }
      }
       // Use a straight line trajectory with adaptive next goal if trajectory type
       // is 3.
      else if(trajectory_type_ == 3){
        VectorXd start_point = VectorXd::Zero(3);
        VectorXd end_point = VectorXd::Zero(3);
        // define the start and end points for the straight line.
        start_point[0] = start_point_x_;
        start_point[1] = start_point_y_;
        start_point[2] = object_half_width_;
    
        end_point[0] = end_point_x_;
        end_point[1] = end_point_y_;
        end_point[2] = object_half_width_;
    
        // compute vector from start point to end point
        VectorXd distance_vector = end_point - start_point;
        // project current jack location onto straight line vector.
        double projection_length = (obj_curr_position - start_point).dot(
          distance_vector)/distance_vector.norm();
        VectorXd projection_point = start_point + projection_length*(
          distance_vector/distance_vector.norm());
    
        // Step ahead along the path to the goal from the projection point, without
        // overshooting past the goal.
        VectorXd target_on_line_with_lookahead;
        if ((obj_curr_position - end_point).norm() < lookahead_step_size_) {
          target_on_line_with_lookahead = end_point;
        }
        else {
          VectorXd step_vector = lookahead_step_size_ * (
            distance_vector/distance_vector.norm());
          target_on_line_with_lookahead = projection_point + step_vector;
        }
    
        // compute error vector between projection point and current jack location
        VectorXd error_vector = target_on_line_with_lookahead - obj_curr_position;
    
        // declaring next_target vector
        VectorXd next_target;
        if (error_vector.norm() >= max_step_size_){
            // if jack is max step size or further from the projection point, the
            // next target will be one step size in that direction.
            next_target = obj_curr_position + max_step_size_*(
              error_vector/error_vector.norm());
        }
        else{
            // else set the next target to be the projection point.
            next_target = target_on_line_with_lookahead;
        }
        // set next target location for jack
        target_obj_position(0) = next_target[0];
        target_obj_position(1) = next_target[1];
        target_obj_position(2) = next_target[2];
      }
      else if(trajectory_type_ == 4){
        // First handle position lookahead.
        VectorXd start_point = obj_curr_position;
        VectorXd end_point = target_final_object_position_;
    
        // compute vector from start point to end point
        VectorXd distance_vector = end_point - start_point;
    
        if(distance_vector.norm() < lookahead_step_size_){
          target_obj_position(0) = end_point[0];
          target_obj_position(1) = end_point[1];
          target_obj_position(2) = end_point[2];
        }
        else{
          VectorXd step_vector = lookahead_step_size_ * (
            distance_vector/distance_vector.norm());
          VectorXd target_on_line_with_lookahead = start_point + step_vector;
          target_obj_position(0) = target_on_line_with_lookahead[0];
          target_obj_position(1) = target_on_line_with_lookahead[1];
          target_obj_position(2) = target_on_line_with_lookahead[2];
        }
    
        // Second handle orientation lookahead.
        // Get target orientation
        Eigen::Quaterniond y_quat_des(target_final_object_orientation_[0],
                                      target_final_object_orientation_[1],
                                      target_final_object_orientation_[2],
                                      target_final_object_orientation_[3]);
    
        // Get current orientation.
        const VectorX<double> &q = object_state->GetPositions().head(4);
        Eigen::Quaterniond y_quat(q(0), q(1), q(2), q(3));
    
        // Compute the error.
        Eigen::AngleAxis<double> angle_axis_diff(y_quat_des * y_quat.inverse());
    
        // Get the axis and angle of rotation.
        double angle = angle_axis_diff.angle();
        Eigen::Vector3d axis = angle_axis_diff.axis();
    
        // Enforce consistency near 180 degrees.
        if ((axis.dot(last_rotation_axis_) < 0) &&
            (PI - angle < angle_hysteresis_)) {
          angle = 2*PI - angle;
          axis = -axis;
        }
        last_rotation_axis_ = axis;
    
        // Enforce the lookahead.
        angle = std::min(angle, lookahead_angle_);
    
        // Apply the rotation.
        Eigen::AngleAxis<double> angle_axis_relative(angle, axis);
        Eigen::Quaterniond quat_relative = Eigen::Quaterniond(angle_axis_relative);
        Eigen::Quaterniond y_quat_lookahead_quat = quat_relative * y_quat;
        Eigen::MatrixXd y_quat_lookahead_wxyz(4, 1);
        y_quat_lookahead_wxyz << y_quat_lookahead_quat.w(),
          y_quat_lookahead_quat.x(), y_quat_lookahead_quat.y(),
          y_quat_lookahead_quat.z();
    
        target_obj_orientation = y_quat_lookahead_wxyz;
      }
      else if(trajectory_type_ == 0){
            // Throw an error.
        std::cerr <<
          ("Trajectory type 0 - Time Based Path : Currently unimplemented")
          << std::endl;
        DRAKE_THROW_UNLESS(false);
      }
      else{
        // Throw an error.
        std::cerr << ("Unknown path type") << std::endl;
        DRAKE_THROW_UNLESS(false);
      }
    
      target_obj_state << target_obj_orientation, target_obj_position;
      target->SetFromVector(target_obj_state);
    }

    void TargetGenerator::CalcObjectVelocityTarget(
        const drake::systems::Context<double>& context,
        BasicVector<double>* target) const {
      const StateVector<double>* object_state =
          (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);
      // Get target orientation
      Eigen::Quaterniond y_quat_des(target_final_object_orientation_[0],
                                    target_final_object_orientation_[1],
                                    target_final_object_orientation_[2],
                                    target_final_object_orientation_[3]);
    
      // Get current orientation
      const VectorX<double> &q = object_state->GetPositions().head(4);
      Eigen::VectorXd normalized_q = q / q.norm();
      Eigen::Quaterniond y_quat(normalized_q(0), normalized_q(1), normalized_q(2),
                                normalized_q(3));
    
      // Compute the error.
      Eigen::AngleAxis<double> angle_axis_diff(y_quat_des * y_quat.inverse());
    
      // Generate spherically interpolated trajectory.
        auto orientation_trajectory = PiecewiseQuaternionSlerp<double>(
        {0, 1}, {y_quat, y_quat_des});
    
      // Evaluate the trajectory at the lookahead time.
      // Scale time based on lookahead angle.
      double lookahead_fraction = std::min(
        lookahead_angle_ / angle_axis_diff.angle(), 1.0);
      Eigen::MatrixXd y_quat_lookahead = orientation_trajectory.value(
        lookahead_fraction);
      Eigen::Quaterniond y_quat_lookahead_quat(
        y_quat_lookahead(0), y_quat_lookahead(1), y_quat_lookahead(2),
        y_quat_lookahead(3));
    
      Eigen::AngleAxis<double> angle_axis_diff_to_lookahead(
        y_quat_lookahead_quat * y_quat.inverse());
      VectorXd angle_error = angle_axis_diff_to_lookahead.angle() *
        angle_axis_diff_to_lookahead.axis();
      angle_error *= angle_err_to_vel_factor_;
    
      VectorXd target_obj_velocity = VectorXd::Zero(6);
      target_obj_velocity << angle_error, VectorXd::Zero(3);
      target->SetFromVector(target_obj_velocity);
    }

    void TargetGenerator::OutputObjectFinalTarget(
        const drake::systems::Context<double>& context,
        BasicVector<double>* target) const {

    VectorXd target_final_obj_state = VectorXd::Zero(7);
    target_final_obj_state << target_final_object_orientation_,
        target_final_object_position_;
    target->SetFromVector(target_final_obj_state);
    }

    void TargetGenerator::SetRandomizedTargetFinalObjectPosition() const {
        // Obeys x and y limits (to stay on the table) as well as radius limits (to
        // more closely match robot limits).
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> x_dis(
        random_goal_x_limits_[0], random_goal_x_limits_[1]);
        std::uniform_real_distribution<double> y_dis(
        random_goal_y_limits_[0], random_goal_y_limits_[1]);
    
        double x = x_dis(gen);
        double y = y_dis(gen);
        while ((sqrt(x*x + y*y) > random_goal_radius_limits_[1]) ||
            (sqrt(x*x + y*y) < random_goal_radius_limits_[0])) {
        x = x_dis(gen);
        y = y_dis(gen);
        }
    
        Eigen::VectorXd target_final_object_position(3);
        target_final_object_position_ << x, y, resting_object_height_;
    }

    void TargetGenerator::SetRandomizedTargetFinalObjectOrientation() const {
        const auto& valid_orientations = ValidOrientations();

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(0, valid_orientations.size()-1);
        int random_index = dis(gen);
      
        if (prevent_three_topples_) {
          while (this->three_topples_required(random_index)) {
            std::cout << "Rejected orientation: " << random_index <<
              " is 3 topples from " << orientation_index_ << std::endl;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(0, valid_orientations.size()-1);
            random_index = dis(gen);
          }
        }
      
        Eigen::Quaterniond quat_nominal = valid_orientations.at(random_index);
      
        // Add random yaw in world frame.  Ensure at least 90 degrees away if no
        // topple is required.
        double min_yaw = 0;
        double max_yaw = 2*PI;
        if (random_index == orientation_index_) {
          min_yaw = PI/2;
          max_yaw = 3*PI/2;
          quat_nominal = Eigen::Quaterniond(target_final_object_orientation_[0],
                                            target_final_object_orientation_[1],
                                            target_final_object_orientation_[2],
                                            target_final_object_orientation_[3]);
          std::cout << "No topple required -- will use ";
        }
        std::uniform_real_distribution<double> yaw_dis(min_yaw, max_yaw);
        double yaw = yaw_dis(gen);
        if (random_index == orientation_index_) {
          std::cout << yaw << " radians of yaw." << std::endl;
        }
        Eigen::Quaterniond quat_world_yaw(
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond quat_final = quat_world_yaw * quat_nominal;
      
        target_final_object_orientation_ <<
          quat_final.w(), quat_final.x(), quat_final.y(), quat_final.z();
      
        orientation_index_ = random_index;
      }

    
    void TargetGenerator::OutputTargetGeneratorInfo(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* target) const {
        // NOTE: This is a placeholder for the boolean value so we can output
        // it as an existing lcm message type. It is unconventional to be
        // using this message type for this purpose.
        Eigen::MatrixXd orientation_index_data = orientation_index_ *
        Eigen::MatrixXd::Ones(1, 1);
        Eigen::VectorXd timestamp = context.get_time() * Eigen::VectorXd::Ones(1);
    
        LcmTrajectory::Trajectory orientation_index_traj;
        orientation_index_traj.traj_name = "orientation_index";
        orientation_index_traj.datatypes = std::vector<std::string>(1, "int");
        orientation_index_traj.datapoints = orientation_index_data;
        orientation_index_traj.time_vector = timestamp.cast<double>();
    
        LcmTrajectory orientation_index_lcm_traj(
        {orientation_index_traj}, {"orientation_index"}, "orientation_index",
        "orientation_index", false);
    
        // Output the mode as an lcm message
        target->saved_traj = orientation_index_lcm_traj.GenerateLcmObject();
        target->utime = context.get_time() * 1e6;
    }
    
    // JACKTOY SPECIFIC
    // Helper function to check if three topples are required.
    bool TargetGeneratorJacktoy::three_topples_required(const int new_orientation_index)
    const {
    switch (orientation_index_) {
    case 0:
        if (new_orientation_index == 3) {return true;} else {return false;}
    case 1:
        if (new_orientation_index == 6) {return true;} else {return false;}
    case 2:
        if (new_orientation_index == 5) {return true;} else {return false;}
    case 3:
        if (new_orientation_index == 0) {return true;} else {return false;}
    case 4:
        if (new_orientation_index == 7) {return true;} else {return false;}
    case 5:
        if (new_orientation_index == 2) {return true;} else {return false;}
    case 6:
        if (new_orientation_index == 1) {return true;} else {return false;}
    case 7:
        if (new_orientation_index == 4) {return true;} else {return false;}
    default:
        return false;
    }
    }

    void TargetGeneratorJacktoy::CycleThroughOrientationSequence() const {
        const auto& valid_orientations = ValidOrientations();
        Eigen::Quaterniond next_quat = valid_orientations.at(goal_counter_ % 8);
        target_final_object_orientation_ <<
          next_quat.w(), next_quat.x(), next_quat.y(), next_quat.z();
      
        orientation_index_ = goal_counter_ % 8;
      }

}  // namespace systems
}  // namespace dairlib