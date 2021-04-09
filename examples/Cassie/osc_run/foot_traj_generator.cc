#include "foot_traj_generator.h"
#include "multibody/multibody_utils.h"

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
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

namespace dairlib::examples::osc_run {

FootTrajGenerator::FootTrajGenerator(
    const MultibodyPlant<double>& plant, Context<double>* context,
    const string& hip_name, bool isLeftFoot,
    const PiecewisePolynomial<double>& foot_traj, bool relative_feet,
    double time_offset)
    : plant_(plant),
      context_(context),
      world_(plant.world_frame()),
      hip_frame_(plant.GetFrameByName(hip_name)),
      foot_traj_(foot_traj),
      relative_feet_(relative_feet) {
  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  if (isLeftFoot) {
    this->set_name("left_ft_traj");
    this->DeclareAbstractOutputPort("left_ft_traj", traj_inst,
                                    &FootTrajGenerator::CalcTraj);
  } else {
    this->set_name("right_ft_traj");
    this->DeclareAbstractOutputPort("right_ft_traj", traj_inst,
                                    &FootTrajGenerator::CalcTraj);
  }

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  // Shift trajectory by time_offset
  foot_traj_.shiftRight(time_offset);
}

PiecewisePolynomial<double> FootTrajGenerator::generateFlightTraj(
    const VectorXd& x, double t) const {
  if (relative_feet_) {
    plant_.SetPositionsAndVelocities(context_, x);
    // Hip offset
    Vector3d zero_offset = Vector3d::Zero();
    Vector3d hip_pos = Vector3d::Zero();
    plant_.CalcPointsPositions(*context_, hip_frame_, zero_offset, world_,
                               &hip_pos);

    const PiecewisePolynomial<double>& foot_traj_segment =
        foot_traj_.slice(foot_traj_.get_segment_index(t), 1);

    std::vector<double> breaks = foot_traj_segment.get_segment_times();
    VectorXd breaks_vector = Map<VectorXd>(breaks.data(), breaks.size());
    MatrixXd hip_points(3, 2);
    // Velocity estimates are generally bad
    hip_points << hip_pos, hip_pos;
    PiecewisePolynomial<double> hip_offset =
        PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, hip_points);

    return foot_traj_segment + hip_offset;
  } else {
    return foot_traj_;
  }
}

void FootTrajGenerator::CalcTraj(
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
  //  if (fsm_state[0] == FLIGHT) {
  *casted_traj = generateFlightTraj(robot_output->GetState(), timestamp);
  //  }
}

}  // namespace dairlib::examples::osc_run
