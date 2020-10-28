#include "flight_foot_traj_generator.h"

#include "multibody/multibody_utils.h"

using std::cout;
using std::endl;

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;

using dairlib::multibody::createContext;
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
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc {

FlightFootTrajGenerator::FlightFootTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, double period, bool is_left_ft)
    : plant_(plant),
      context_(context),
      world_(plant.world_frame()),
      period_(period),
      is_left_ft_(is_left_ft){
  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  if(is_left_ft_){
    center_line_offset_ = 0.12;
    this->set_name("left_ft_traj");
    this->DeclareAbstractOutputPort("left_ft_traj", traj_inst,
                                    &FlightFootTrajGenerator::CalcTraj);
  }
  else{
    center_line_offset_ = -0.12;
    this->set_name("right_ft_traj");
    this->DeclareAbstractOutputPort("right_ft_traj", traj_inst,
                                    &FlightFootTrajGenerator::CalcTraj);
  }

//  this->set_name("swing_ft_traj");
//  this->DeclareAbstractOutputPort("swing_ft_traj", traj_inst,
//                                  &FlightFootTrajGenerator::CalcTraj);

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
}

/*
  Move the feet relative to the COM
  The trajectory of the COM cannot be altered, so must solve for
  foot positions as a function of COM.
*/
PiecewisePolynomial<double> FlightFootTrajGenerator::generateSwingTraj(
    const VectorXd& x, double t) const {
  VectorXd zero_input = VectorXd::Zero(plant_.num_actuators());
  plant_.SetPositionsAndVelocities(context_, x);

  double start_time = t - fmod(t, period_);
  double end_time = start_time + period_;
  std::vector<double> T_waypoint = {
      start_time, start_time + 0.5 * (end_time - start_time), end_time};
  std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));

  // x
  Y[0](0, 0) = -0.2;
  Y[1](0, 0) = 0.0;
  Y[2](0, 0) = 0.2;
  // y
  Y[0](1, 0) = center_line_offset_;
  Y[1](1, 0) = center_line_offset_;
  Y[2](1, 0) = center_line_offset_;
  // z
  /// We added stance_foot_height because we want the desired trajectory to be
  /// relative to the stance foot in case the floating base state estimation
  /// drifts.
  Y[0](2, 0) = -0.8;
  Y[1](2, 0) = -0.7;
  Y[2](2, 0) = -0.8;

  std::vector<MatrixXd> Y_dot(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y_dot[0](0, 0) = 0;
  Y_dot[1](0, 0) = 0.2 / period_;
  Y_dot[2](0, 0) = 0;
  // y
  Y_dot[0](1, 0) = 0;
  Y_dot[1](1, 0) = 0;
  Y_dot[2](1, 0) = 0;
  // z
  Y_dot[0](2, 0) = 0;
  Y_dot[1](2, 0) = 0;
  Y_dot[2](2, 0) = -0.01;

  //  const PiecewisePolynomial<double>& foot_traj_segment =
  //      foot_traj_.slice(foot_traj_.get_segment_index(t), 1);

  return PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
      T_waypoint, Y, Y_dot.at(0), Y_dot.at(2));
}

PiecewisePolynomial<double> FlightFootTrajGenerator::generateStanceTraj(
    const VectorXd& x, double t) const {
  VectorXd zero_input = VectorXd::Zero(plant_.num_actuators());
  plant_.SetPositionsAndVelocities(context_, x);

  double start_time = t - fmod(t, period_);
  double end_time = start_time + period_;
  std::vector<double> T_waypoint = {
      start_time, start_time + 0.5 * (end_time - start_time), end_time};
  std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));

  // x
  Y[0](0, 0) = 0.2;
  Y[1](0, 0) = 0.0;
  Y[2](0, 0) = -0.2;
  // y
  Y[0](1, 0) = center_line_offset_;
  Y[1](1, 0) = center_line_offset_;
  Y[2](1, 0) = center_line_offset_;
  // z
  /// We added stance_foot_height because we want the desired trajectory to be
  /// relative to the stance foot in case the floating base state estimation
  /// drifts.
  Y[0](2, 0) = -0.8;
  Y[1](2, 0) = -0.8;
  Y[2](2, 0) = -0.8;

  std::vector<MatrixXd> Y_dot(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y_dot[0](0, 0) = 0;
  Y_dot[1](0, 0) = 0.2 / period_;
  Y_dot[2](0, 0) = 0;
  // y
  Y_dot[0](1, 0) = 0;
  Y_dot[1](1, 0) = 0;
  Y_dot[2](1, 0) = 0;
  // z
  Y_dot[0](2, 0) = -0.01;
  Y_dot[1](2, 0) = 0;
  Y_dot[2](2, 0) = 0;

  //  const PiecewisePolynomial<double>& foot_traj_segment =
  //      foot_traj_.slice(foot_traj_.get_segment_index(t), 1);

  return PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
      T_waypoint, Y, Y_dot.at(0), Y_dot.at(2));
}

void FlightFootTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  VectorXd x = robot_output->GetState();
  double timestamp = robot_output->get_timestamp();

  // Read in finite state machine
  const auto fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value();

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if(fsm_state[0] != is_left_ft_){
    *casted_traj =
        generateSwingTraj(robot_output->GetState(), timestamp);
  }
  else{
    *casted_traj =
        generateStanceTraj(robot_output->GetState(), timestamp);
  }
}

}  // namespace dairlib::examples::osc
