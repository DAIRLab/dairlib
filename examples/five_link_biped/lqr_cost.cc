//
// Created by yangwill on 11/24/19.
//

#include "lqr_cost.h"

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::Trajectory;
using Eigen::VectorXd;
using std::shared_ptr;
using std::vector;
namespace dairlib {

using systems::BasicVector;
using systems::OutputVector;
using systems::TimestampedVector;

LQRCost::LQRCost(const MultibodyPlant<double>& plant, const Eigen::MatrixXd& q,
                 const Eigen::MatrixXd& r,
                 const vector<shared_ptr<Trajectory<double>>>& state_traj,
                 const vector<shared_ptr<Trajectory<double>>>& input_traj)
    : plant_(plant),
      Q_(q),
      R_(r),
      state_traj_(state_traj),
      input_traj_(input_traj) {
  DRAKE_DEMAND(Q_.cols() == (plant.num_positions() + plant.num_velocities()));
  DRAKE_DEMAND(Q_.rows() == (plant.num_positions() + plant.num_velocities()));
  DRAKE_DEMAND(R_.cols() == (plant.num_actuators()));

  this
      ->DeclareVectorInputPort(OutputVector<double>(
          plant.num_positions(), plant.num_velocities(), plant.num_actuators()))
      .get_index();

  this->DeclareVectorOutputPort(BasicVector<double>(2), &LQRCost::CalcCost)
      .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  DeclarePerStepDiscreteUpdateEvent(&LQRCost::DiscreteVariableUpdate);

  cost_idx_ = this->DeclareDiscreteState(1);
  time_idx_ = this->DeclareDiscreteState(1);
}

EventStatus LQRCost::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto cost = discrete_state->get_mutable_vector(cost_idx_).get_mutable_value();
  auto prev_time =
      discrete_state->get_mutable_vector(time_idx_).get_mutable_value();
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, 0);
  const BasicVector<double>* fsm_state =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);

  int mode = (int)fsm_state->get_value()(0);
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);

  if (current_time < 1e-5) {  // Simulator has restarted
    prev_time << current_time;
    cost << 0.0;
  }

  double dt = current_time - prev_time(0);
  if (dt > 1e-10) {
    prev_time << current_time;

    // First order approximation
    VectorXd state_error =
        robot_output->GetState() - state_traj_[mode]->value(current_time);
    VectorXd input_error =
        robot_output->GetEfforts() - input_traj_[mode]->value(current_time);
    //  VectorXd additional_cost = (state_error.transpose() * Q_ * state_error +
    //                             input_error.transpose() * R_ * input_error) *
    //                             dt;
    VectorXd additional_cost =
        (state_error.transpose() * Q_ * state_error +
         robot_output->GetEfforts() * R_ * robot_output->GetEfforts()) *
        dt;
    cost << cost + additional_cost;
    //    std::cout << "Cumulative cost: " << cost << std::endl;
  }
  return EventStatus::Succeeded();
}

void LQRCost::CalcCost(const drake::systems::Context<double>& context,
                       systems::BasicVector<double>* cost) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, 0);
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);
  cost->get_mutable_value() << current_time,
      context.get_discrete_state().get_vector(cost_idx_).get_value();
}

}  // namespace dairlib