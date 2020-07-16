#include "swing_foot_traj_generator.h"

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

namespace dairlib::examples::Cassie::osc_walk {

SwingFootTrajGenerator::SwingFootTrajGenerator(
    const MultibodyPlant<double>& plant, const string& hip_name,
    bool isLeftFoot, const PiecewisePolynomial<double>& foot_traj,
    double time_offset)
    : plant_(plant),
      world_(plant.world_frame()),
      stance_foot_frame_(plant.GetFrameByName(hip_name)),
      foot_traj_(foot_traj) {
  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  if (isLeftFoot) {
    this->set_name("l_foot_traj");
    this->DeclareAbstractOutputPort("l_foot_traj", traj_inst,
                                    &SwingFootTrajGenerator::CalcTraj);
    active_state_ = LEFT;
  } else {
    this->set_name("r_foot_traj");
    this->DeclareAbstractOutputPort("r_foot_traj", traj_inst,
                                    &SwingFootTrajGenerator::CalcTraj);
    active_state_ = RIGHT;
  }

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  // Shift trajectory by time_offset
  // TODO(yangwill) add this shift elsewhere to make the gait periodic
  foot_traj_.shiftRight(time_offset);
}

/*
  Move the swing foot relative to the stance foot. The stance foot is fixed so
  should be a good reference point
*/
PiecewisePolynomial<double> SwingFootTrajGenerator::generateFootTraj(
    const drake::systems::Context<double>& context, const VectorXd& x,
    double t) const {
  VectorXd zero_input = VectorXd::Zero(plant_.num_actuators());
  auto plant_context = createContext(plant_, x, zero_input);

  Vector3d zero_offset = Vector3d::Zero();
  Vector3d stance_foot_pos = Vector3d::Zero();
  plant_.CalcPointsPositions(*plant_context, stance_foot_frame_, zero_offset,
                             world_, &stance_foot_pos);

  const PiecewisePolynomial<double>& foot_traj_segment =
      foot_traj_.slice(foot_traj_.get_segment_index(t), 1);

  // Hip offset stuff
  std::vector<double> breaks = foot_traj_segment.get_segment_times();
  VectorXd breaks_vector = Map<VectorXd>(breaks.data(), breaks.size());
  MatrixXd stance_foot(3, 2);
  // Velocity estimates are generally bad
  stance_foot << stance_foot_pos, stance_foot_pos;
  PiecewisePolynomial<double> hip_offset =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, stance_foot);

  return foot_traj_segment + hip_offset;
}

void SwingFootTrajGenerator::CalcTraj(
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
  if (fsm_state(0) == active_state_) {
    *casted_traj =
        generateFootTraj(context, robot_output->GetState(), timestamp);
  } else {
    // Do nothing to avoid bugs, maybe return a zero trajectory?
  }
}

}  // namespace dairlib::examples::Cassie::osc_walk
