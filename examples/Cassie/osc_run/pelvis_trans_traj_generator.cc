#include "examples/Cassie/osc_run/pelvis_trans_traj_generator.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc {

PelvisTransTrajGenerator::PelvisTransTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::trajectories::PiecewisePolynomial<double>& traj,
    const std::unordered_map<
        int, std::vector<std::pair<const Eigen::Vector3d,
                                   const drake::multibody::Frame<double>&>>>&
        feet_contact_points)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      traj_(traj),
      feet_contact_points_(feet_contact_points) {
  this->set_name("pelvis_trans_traj");
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  clock_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("pelvis_trans_traj", traj_inst,
                                  &PelvisTransTrajGenerator::CalcTraj);

  DeclarePerStepDiscreteUpdateEvent(
      &PelvisTransTrajGenerator::DiscreteVariableUpdate);
}

EventStatus PelvisTransTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  return EventStatus::Succeeded();
}

PiecewisePolynomial<double> PelvisTransTrajGenerator::GeneratePelvisTraj(
    const VectorXd& x, double t, int fsm_state) const {
  plant_.SetPositionsAndVelocities(context_, x);
  Vector3d contact_pos_sum = Vector3d::Zero();
  Vector3d position;
  for (const auto& point_and_frame : feet_contact_points_.at(fsm_state)) {
    plant_.CalcPointsPositions(*context_, point_and_frame.second,
                               VectorXd::Zero(3), world_, &position);
    contact_pos_sum += position;
  }
  contact_pos_sum(2) = 0;

  std::vector<double> breaks = traj_.get_segment_times();
  VectorXd breaks_vector = Eigen::Map<VectorXd>(breaks.data(), breaks.size());
  MatrixXd offset_matrix = 0.5 * contact_pos_sum.replicate(1, breaks.size());

  PiecewisePolynomial<double> com_offset =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, offset_matrix);

//  return traj_ + com_offset;
  return traj_;
}

void PelvisTransTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  // Read in finite state machine
  const auto& fsm_state =
      this->EvalVectorInput(context, fsm_port_)->get_value()(0);
  const auto& clock =
      this->EvalVectorInput(context, clock_port_)->get_value()(0);

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
//  const drake::VectorX<double>& x = robot_output->GetState();
  if (fsm_state == 0 || fsm_state == 1) {
    *casted_traj =
        GeneratePelvisTraj(robot_output->GetState(), clock, fsm_state);
  }
}

}  // namespace dairlib::examples::osc