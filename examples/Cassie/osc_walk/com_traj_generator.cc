#include "examples/Cassie/osc_walk/com_traj_generator.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

using dairlib::multibody::createContext;
using std::cout;
using std::endl;
using std::pair;
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
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc_walk {

COMTrajGenerator::COMTrajGenerator(const MultibodyPlant<double>& plant,
                                   PiecewisePolynomial<double> com_traj,
                                   double time_offset)
    : plant_(plant), world_(plant_.world_frame()), com_traj_(com_traj) {
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
  x_offset_idx_ = this->DeclareDiscreteState(1);
  time_shift_idx_ = this->DeclareDiscreteState(1);
  fsm_idx_ = this->DeclareDiscreteState(1);

  DeclarePerStepDiscreteUpdateEvent(&COMTrajGenerator::DiscreteVariableUpdate);
  com_traj_.shiftRight(time_offset);
  context_ = plant_.CreateDefaultContext();
}

EventStatus COMTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  auto prev_fsm_state =
      discrete_state->get_mutable_vector(fsm_idx_).get_mutable_value();
  auto x_offset =
      discrete_state->get_mutable_vector(x_offset_idx_).get_mutable_value();
  auto time_shift =
      discrete_state->get_mutable_vector(x_offset_idx_).get_mutable_value();

  const BasicVector<double>* fsm_output =
      this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  double timestamp = robot_output->get_timestamp();

  if (prev_fsm_state(0) != fsm_state(0)) {  // When to reset the clock
    prev_fsm_state(0) = fsm_state(0);

    if (fsm_state(0) == LEFT) {
      VectorXd q = robot_output->GetPositions();
      plant_.SetPositions(context_.get(), q);
      VectorXd center_of_mass = plant_.CalcCenterOfMassPosition(*context_);
      x_offset(0) = (center_of_mass(0) - com_traj_.value(0)(0));
      time_shift(0) = timestamp;
    }
  }
  return EventStatus::Succeeded();
}

void COMTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  auto time_shift = context.get_discrete_state(time_shift_idx_).get_value();

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);

  *casted_traj = com_traj_;
  casted_traj->shiftRight(time_shift(0));
}

}  // namespace dairlib::examples::osc_walk