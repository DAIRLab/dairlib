#include "com_traj_generator.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

using dairlib::multibody::createContext;
using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::Cassie::osc_jump {

COMTrajGenerator::COMTrajGenerator(const MultibodyPlant<double>& plant,
                                   int pelvis_idx, Vector3d front_contact_disp,
                                   Vector3d rear_contact_disp,
                                   PiecewisePolynomial<double> crouch_traj,
                                   double height,
                                   double time_offset)
    : plant_(plant),
      hip_idx_(pelvis_idx),
      front_contact_disp_(front_contact_disp),
      rear_contact_disp_(rear_contact_disp),
      crouch_traj_(crouch_traj),
      height_(height) {
  this->set_name("com_traj");
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("com_traj", traj_inst,
                                  &COMTrajGenerator::CalcTraj);
  time_idx_ = this->DeclareDiscreteState(1);
  com_x_offset_idx_ = this->DeclareDiscreteState(1);
  fsm_idx_ = this->DeclareDiscreteState(1);

  DeclarePerStepDiscreteUpdateEvent(&COMTrajGenerator::DiscreteVariableUpdate);

  crouch_traj_.shiftRight(time_offset);
}

EventStatus COMTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto prev_fsm_state =
      discrete_state->get_mutable_vector(fsm_idx_).get_mutable_value();
  auto prev_time =
      discrete_state->get_mutable_vector(time_idx_).get_mutable_value();
  auto com_x_offset =
      discrete_state->get_mutable_vector(com_x_offset_idx_).get_mutable_value();

  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);

  if (prev_fsm_state(0) != fsm_state(0)) {  // When to reset the clock
    prev_fsm_state(0) = fsm_state(0);
    prev_time(0) = current_time;
    VectorXd zero_input = VectorXd::Zero(plant_.num_actuators());

    auto plant_context =
        createContext(plant_, robot_output->GetState(), zero_input);
    com_x_offset(0) = 0.05;
  }
  return EventStatus::Succeeded();
}

drake::trajectories::PiecewisePolynomial<double> COMTrajGenerator::generateBalanceTraj(
    const drake::systems::Context<
        double>& context,
    const Eigen::VectorXd& x,
    double time) const {
  VectorXd zero_input = VectorXd::Zero(plant_.num_actuators());
  auto plant_context = createContext(plant_, x, zero_input);
  const drake::multibody::BodyFrame<double>& world = plant_.world_frame();
  const drake::multibody::BodyFrame<double>& l_toe_frame =
      plant_.GetBodyByName("toe_left").body_frame();
  const drake::multibody::BodyFrame<double>& r_toe_frame =
      plant_.GetBodyByName("toe_right").body_frame();

  Vector3d l_toe_front;
  Vector3d l_toe_rear;
  Vector3d r_toe_front;
  Vector3d r_toe_rear;

  plant_.CalcPointsPositions(*plant_context, l_toe_frame, front_contact_disp_,
                             world, &l_toe_front);
  plant_.CalcPointsPositions(*plant_context, l_toe_frame, rear_contact_disp_,
                             world, &l_toe_rear);
  plant_.CalcPointsPositions(*plant_context, r_toe_frame, front_contact_disp_,
                             world, &r_toe_front);
  plant_.CalcPointsPositions(*plant_context, r_toe_frame, rear_contact_disp_,
                             world, &r_toe_rear);
  Vector3d currCoM = plant_.CalcCenterOfMassPosition(*plant_context);

  Vector3d targetCoM =
      (l_toe_front + l_toe_rear + r_toe_front + r_toe_rear) / 4;
  targetCoM(2) = height_;
  targetCoM(0) += 0.05;
  MatrixXd centerOfMassPoints(3, 2);
  centerOfMassPoints << currCoM, targetCoM;
  VectorXd breaks_vector(2);
  breaks_vector << 0, 20.0*(targetCoM - currCoM).norm();

  // Slowly move toward the desired initial configuration
  return PiecewisePolynomial<double>::FirstOrderHold(breaks_vector,
      centerOfMassPoints);
}

drake::trajectories::PiecewisePolynomial<double> COMTrajGenerator::generateCrouchTraj(
    const drake::systems::Context<
        double>& context,
    const Eigen::VectorXd& x,
    double time) const {

  // This assumes that the crouch is starting at the exact position as the
  // start of the target trajectory which should be handled by balance
  // trajectory
  const PiecewisePolynomial<double>& com_traj =
      crouch_traj_.slice(crouch_traj_.get_segment_index(time), 1);

  return com_traj;
}

drake::trajectories::PiecewisePolynomial<double> COMTrajGenerator::generateLandingTraj(
    const drake::systems::Context<
        double>& context,
    const Eigen::VectorXd& x,
    double time) const {

  const VectorXd com_x_offset =
      context.get_discrete_state().get_vector(com_x_offset_idx_).get_value();

  // Only offset the x-position
  Vector3d offset;
  offset << com_x_offset(0), 0, 0;

  auto traj_segment = crouch_traj_.slice(crouch_traj_.get_segment_index(time), 1);
  std::vector<double> breaks = traj_segment.get_segment_times();
  MatrixXd offset_matrix = offset.replicate(1, breaks.size());
  VectorXd breaks_vector = Eigen::Map<VectorXd>(breaks.data(), breaks.size());
  PiecewisePolynomial<double> com_offset =
      PiecewisePolynomial<double>::FirstOrderHold(breaks_vector, offset_matrix);
  return traj_segment + com_offset;
}

void COMTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  double time = robot_output->get_timestamp();

  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  const drake::VectorX<double>& x = robot_output->GetState();

  switch (static_cast<int>(fsm_state(0))) {
    case (BALANCE):  //  BALANCE
      *casted_traj = generateBalanceTraj(context, x, time);
      break;
    case (CROUCH):  //  CROUCH
      *casted_traj = generateCrouchTraj(context, x, time);
      break;
    case (LAND):  //  LAND
      *casted_traj = generateLandingTraj(context, x, time);
      break;
  }
}

}  // namespace dairlib::examples::Cassie::osc_jump