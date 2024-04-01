#include "heuristic_generator.h"
#include <iostream>


using dairlib::systems::OutputVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::EventStatus;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

HeuristicGenerator::HeuristicGenerator(
    const MultibodyPlant<double>& lcs_plant,
    const SimulateFrankaParams& sim_param,
    const HeuristicGaitParams& heuristic_param,
    const BallRollingTrajectoryParams& trajectory_param,
    const C3Options& c3_param) {
    // INPUT PORTS 1, get current simplified plant (i.e. plant to generate lcs) state
    plant_state_port_ =
            this->DeclareVectorInputPort(
                            "lcs_plant_state", StateVector<double>(lcs_plant.num_positions(),
                                                                   lcs_plant.num_velocities()))
                    .get_index();
    // INPUT PORTS 2, get object desired target trajectory
    // TODO:: make dimension not hardcoded
    input_target_port_ =
            this->DeclareVectorInputPort(
                    "object_target", BasicVector<double>(7))
                    .get_index();

    // OUTPUT PORTS 1: send out simple model desired state (y_des)
    output_target_port_ =
            this->DeclareVectorOutputPort(
                    "lcs_state_target", BasicVector<double>(lcs_plant.num_positions() + lcs_plant.num_velocities()),
                    &HeuristicGenerator::CalcHeuristicTarget)
                    .get_index();

    // OUTPUT PORTS 2: send out heuristic end-effector tilt
    orientation_port_ =
            this->DeclareVectorOutputPort(
                    "orientation_target", BasicVector<double>(3),
                    &HeuristicGenerator::CalcHeuristicTilt)
                    .get_index();

    gain_port_ =
            this->DeclareAbstractOutputPort("gain_target",&HeuristicGenerator::CalcHeuristicGain)
                    .get_index();

    // Set Trajectory Patameters
    SetHeuristicParameters(sim_param, heuristic_param, trajectory_param, c3_param);
}

void HeuristicGenerator::SetHeuristicParameters( const SimulateFrankaParams& sim_param,
                                                 const HeuristicGaitParams& heuristic_param,
                                                 const BallRollingTrajectoryParams& trajectory_param,
                                                 const C3Options& c3_param) {
    roll_phase_ = heuristic_param.roll_phase;
    return_phase_ = heuristic_param.return_phase;
    rolling_period_ = roll_phase_ + return_phase_;
    gait_parameters_ = heuristic_param.gait_parameters;
    table_offset_ = sim_param.ground_offset_frame(2);
    axis_option_ = heuristic_param.axis_option;
    tilt_degrees_ = heuristic_param.tilt_degrees;
    x_c_ = trajectory_param.x_c;
    y_c_ = trajectory_param.y_c;
    q_new_vector_ = heuristic_param.q_new_vector;

    settling_time_ = sim_param.stabilize_time1 + sim_param.move_time + sim_param.stabilize_time2;
    ee_default_height = sim_param.ee_radius + 2 * sim_param.ball_radius + table_offset_;

    c3_param_ = c3_param;
}

void HeuristicGenerator::CalcHeuristicTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {

  // Evaluate input port for plant state (object state) and object target
  auto lcs_plant_state = (OutputVector<double>*)this->EvalVectorInput(context, plant_state_port_);
  auto object_target = (BasicVector<double>*)this->EvalVectorInput(context, input_target_port_);

  double timestamp = lcs_plant_state->get_timestamp();
  VectorXd object_cur_position = lcs_plant_state->GetPositions().tail(3);
  VectorXd object_des_position = object_target->value().tail(3);
  VectorXd object_position_error = object_des_position - object_cur_position;
  VectorXd error_normed = object_position_error / object_position_error.norm();

  // compute rolling phase and period time (i.e. time in a single period)
  double shifted_time = timestamp - settling_time_ -  return_phase_;
  if (shifted_time < 0) shifted_time += rolling_period_;
  double time_in_period = shifted_time - rolling_period_ * floor((shifted_time / rolling_period_));
  double back_dist = gait_parameters_(1);

  // assigning heuristic determined end-effector position
  VectorXd ee_des_position = VectorXd::Zero(3);
  VectorXd ee_cur_position = lcs_plant_state->GetPositions().head(3);

  // rolling phase
  if ( time_in_period < roll_phase_ ) {
      ee_des_position[0] = object_cur_position[0];
      ee_des_position[1] = object_cur_position[1];
      ee_des_position[2] = ee_default_height + gait_parameters_[0];
  }
  // upward phase
  else if (time_in_period < roll_phase_ + return_phase_ / 3){
      ee_des_position[0] = ee_cur_position[0];
      ee_des_position[1] = ee_cur_position[1];
      ee_des_position[2] = gait_parameters_[2] + table_offset_;
  }
  // sideway phase
  else if( time_in_period < roll_phase_ + 2 * return_phase_ / 3 ) {
      ee_des_position[0] = object_cur_position[0] - back_dist * error_normed(0);
      ee_des_position[1] = object_cur_position[1] - back_dist * error_normed(1);
      ee_des_position[2] = gait_parameters_[3] + table_offset_;
  }
  // position finger phase
  else{
      ee_des_position[0] = object_cur_position[0] - back_dist * error_normed(0);
      ee_des_position[1] = object_cur_position[1] - back_dist * error_normed(1);
      ee_des_position[2] = gait_parameters_[4] + table_offset_;
  }

  VectorXd velocity_des = VectorXd::Zero(lcs_plant_state->GetVelocities().size());
  VectorXd target_lcs_state = VectorXd::Zero(lcs_plant_state->GetPositions().size() + lcs_plant_state->GetVelocities().size());
  target_lcs_state << ee_des_position, 1, 0, 0, 0, object_des_position, velocity_des;
  target->SetFromVector(target_lcs_state);
}


void HeuristicGenerator::CalcHeuristicTilt(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* orientation_target) const {

    // Evaluate input port for plant state (object state) and object target
    auto lcs_plant_state = (OutputVector<double>*)this->EvalVectorInput(context, plant_state_port_);
    auto object_target = (BasicVector<double>*)this->EvalVectorInput(context, input_target_port_);

    VectorXd object_cur_position = lcs_plant_state->GetPositions().tail(3);
    VectorXd object_des_position = object_target->value().tail(3);
    VectorXd object_position_error = object_des_position - object_cur_position;
    VectorXd error_normed = object_position_error / object_position_error.norm();

    VectorXd axis = VectorXd::Zero(3);
    if (axis_option_ == 1){
        // OPTION 1: tilt EE away from the desired direction of the ball
        axis << error_normed(1), -error_normed(0), 0;
    }
    else if (axis_option_ == 2){
        // OPTION 2: tilt EE toward the center of the circle trajectory (only works for circle trajectory)
        axis << object_cur_position(1)-y_c_, -(object_cur_position(0)-x_c_), 0;
        axis = axis / axis.norm();
    }
    else{
        // Throw an error.
        std::cerr << ("Unknown axis option") << std::endl;
        DRAKE_THROW_UNLESS(false);
    }
    orientation_target->SetFromVector(axis);
}

void HeuristicGenerator::CalcHeuristicGain(
    const drake::systems::Context<double>& context,
    solvers::C3::CostMatrices* Cost_target) const {

    // Evaluate input port for plant state (object state) and object target
    auto lcs_plant_state = (OutputVector<double>*)this->EvalVectorInput(context, plant_state_port_);
    auto object_target = (BasicVector<double>*)this->EvalVectorInput(context, input_target_port_);

    double timestamp = lcs_plant_state->get_timestamp();

    // compute rolling phase and period time (i.e. time in a single period)
    double shifted_time = timestamp - settling_time_ -  return_phase_;
    if (shifted_time < 0) shifted_time += rolling_period_;
    double time_in_period = shifted_time - rolling_period_ * floor((shifted_time / rolling_period_));

    MatrixXd Q = c3_param_.Q;
    MatrixXd R = c3_param_.R;
    MatrixXd G = c3_param_.G;
    MatrixXd U = c3_param_.U;

    if (time_in_period > roll_phase_){
        Q = q_new_vector_.asDiagonal();
    }

    std::vector<MatrixXd> Q_c3(c3_param_.N + 1, Q);
    std::vector<MatrixXd> R_c3(c3_param_.N, R);
    std::vector<MatrixXd> G_c3(c3_param_.N, G);
    std::vector<MatrixXd> U_c3(c3_param_.N, U);

    solvers::C3::CostMatrices Cost(Q_c3, R_c3, G_c3, U_c3);
    *Cost_target = Cost;
}

}  // namespace systems
}  // namespace dairlib