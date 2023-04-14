#include "end_effector_trajectory.h"

#include <iostream>

#include "dairlib/lcmt_radio_out.hpp"
#include "multibody/multibody_utils.h"

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::string;

using dairlib::systems::OutputVector;
using drake::multibody::BodyFrame;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib {

EndEffectorTrajectoryGenerator::EndEffectorTrajectoryGenerator(
    const MultibodyPlant<double>& plant, Context<double>* context)
    : plant_(plant), context_(context), world_(plant.world_frame()) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        "x", OutputVector<double>(plant_.num_positions(),
                                                  plant_.num_velocities(),
                                                  plant_.num_actuators()))
                    .get_index();
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();

  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "trajectory",
              drake::Value<drake::trajectories::Trajectory<double>>(pp))
          .get_index();
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("end_effector_trajectory", traj_inst,
                                  &EndEffectorTrajectoryGenerator::CalcTraj);
}

PiecewisePolynomial<double> EndEffectorTrajectoryGenerator::GenerateCircle(
    const drake::systems::Context<double>& context) const {
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  double t = robot_output->get_timestamp();
  double dt = 0.1;
  double frequency = 0.5 * (1 + radio_out->channel[2]) * M_PI;
  double radius = radio_out->channel[0] * 0.2;
  int index = 1 - std::max(0.0, radio_out->channel[15]);
  VectorXd y0 = VectorXd::Zero(3);
  y0(1 - index) = 0.7;
  y0(index) = radius * cos(frequency * t);
  y0(2) = radius * sin(frequency * t) + 0.3;
  VectorXd ydot0 = VectorXd::Zero(3);
  ydot0(1 - index) = 0;
  ydot0(index) = -radius * frequency * sin(frequency * t);
  ydot0(2) = -radius * frequency * cos(frequency * t);
  std::vector<double> breaks = {t, t + dt};
  std::vector<MatrixXd> samples = {y0, y0 + dt * ydot0};
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      breaks, samples);
}

PiecewisePolynomial<double> EndEffectorTrajectoryGenerator::GeneratePose(
    const drake::systems::Context<double>& context) const {
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  double t = robot_output->get_timestamp();
  double dt = 0.1;
  VectorXd y0 = VectorXd::Zero(3);
  y0(0) = 0.65 + radio_out->channel[0] * 0.2;
  y0(0) = std::clamp(y0(0), 0.0, 0.75);
  y0(1) = radio_out->channel[1] * 0.2;
  y0(2) = 0.3 + radio_out->channel[2] * 0.2;
  VectorXd ydot0 = VectorXd::Zero(3);
  std::vector<double> breaks = {t, t + dt};
  std::vector<MatrixXd> samples = {y0, y0 + dt * ydot0};
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      breaks, samples);
}

PiecewisePolynomial<double> EndEffectorTrajectoryGenerator::GenerateLine(
    const drake::systems::Context<double>& context) const {
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  double t = robot_output->get_timestamp();
  double dt = 0.1;
  VectorXd y0 = VectorXd::Zero(3);
  y0(0) = 0;
  y0(1) = 0.8;
  y0(2) = 0.2 * sin(M_PI * t) + 0.5;
  VectorXd ydot0 = VectorXd::Zero(3);
  ydot0(0) = 0;
  ydot0(1) = 0;
  ydot0(2) = -0.2 * M_PI * cos(M_PI * t);
  std::vector<double> breaks = {t, t + dt};
  std::vector<MatrixXd> samples = {y0, y0 + dt * ydot0};
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      breaks, samples);
}

void EndEffectorTrajectoryGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  //  // Read in finite state machine
  const auto& trajectory_input =
      this->EvalAbstractInput(context, trajectory_port_)
          ->get_value<drake::trajectories::Trajectory<double>>();
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (radio_out->channel[14]) {
    *casted_traj = *(PiecewisePolynomial<double>*)dynamic_cast<
        const PiecewisePolynomial<double>*>(&trajectory_input);
  } else {
    *casted_traj = GeneratePose(context);
    //  *casted_traj = GenerateCircle(context);
    //  *casted_traj = GenerateLine(context);
  }
}

}  // namespace dairlib
