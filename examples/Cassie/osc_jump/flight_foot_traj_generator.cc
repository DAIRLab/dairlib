#include "flight_foot_traj_generator.h"

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

namespace dairlib::examples::osc_jump {

FlightFootTrajGenerator::FlightFootTrajGenerator(
    const MultibodyPlant<double>& plant, Context<double>* context,
    const string& hip_name, bool isLeftFoot,
    const PiecewisePolynomial<double>& foot_traj,
    const PiecewisePolynomial<double>& hip_traj,
    double time_offset)
    : plant_(plant),
      context_(context),
      world_(plant.world_frame()),
      hip_frame_(plant.GetFrameByName(hip_name)),
      foot_traj_(foot_traj),
      hip_traj_(hip_traj){
  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  if (isLeftFoot) {
    this->set_name("left_ft_traj");
    this->DeclareAbstractOutputPort("left_ft_traj", traj_inst,
                                    &FlightFootTrajGenerator::CalcTraj);
  } else {
    this->set_name("right_ft_traj");
    this->DeclareAbstractOutputPort("right_ft_traj", traj_inst,
                                    &FlightFootTrajGenerator::CalcTraj);
  }

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant_.num_positions(),
                                      plant_.num_velocities(),
                                      plant_.num_actuators()))
      .get_index();
  fsm_port_ =
      this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();

  // Shift trajectory by time_offset
  foot_traj_.shiftRight(time_offset);
  hip_traj_.shiftRight(time_offset);
}

PiecewisePolynomial<double> FlightFootTrajGenerator::GenerateRelativeTraj()
const {
  return foot_traj_ - hip_traj_;
}

void FlightFootTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  VectorXd x = robot_output->GetState();
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *casted_traj = GenerateRelativeTraj();
}

}  // namespace dairlib::examples::osc_jump
