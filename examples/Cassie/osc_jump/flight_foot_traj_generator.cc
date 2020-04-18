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
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using drake::multibody::JacobianWrtVariable;

namespace dairlib::examples::Cassie::osc_jump {

FlightFootTrajGenerator::FlightFootTrajGenerator(
    const MultibodyPlant<double>& plant, const string& hip_name,
    bool isLeftFoot, const PiecewisePolynomial<double>& foot_traj, double
    time_offset)
    : plant_(plant),
      hip_name_(hip_name),
      foot_traj_(foot_traj){
  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  if (isLeftFoot) {
    this->set_name("l_foot_traj");
    this->DeclareAbstractOutputPort("l_foot_traj", traj_inst,
        &FlightFootTrajGenerator::CalcTraj);
  } else {
    this->set_name("r_foot_traj");
    this->DeclareAbstractOutputPort("r_foot_traj", traj_inst,
        &FlightFootTrajGenerator::CalcTraj);
  }

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant_.num_positions(),
              plant_.num_velocities(),
              plant_.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  //Shift trajectory by time_offset
  foot_traj_.shiftRight(time_offset);
}

/*
  Move the feet relative to the COM
  The trajectory of the COM cannot be altered, so must solve for
  foot positions as a function of COM.
*/
PiecewisePolynomial<double> FlightFootTrajGenerator::generateFlightTraj(
    const drake::systems::Context<double>& context, const VectorXd& x,
    double t) const {
  VectorXd zero_input = VectorXd::Zero(plant_.num_actuators());
  auto plant_context = createContext(plant_, x, zero_input);

  const drake::multibody::BodyFrame<double>& world = plant_.world_frame();
  const drake::multibody::BodyFrame<double>& hip_frame =
      plant_.GetBodyByName(hip_name_).body_frame();

  Vector3d zero_offset = Vector3d::Zero();
  Vector3d hip_pos = Vector3d::Zero();
  plant_.CalcPointsPositions(*plant_context, hip_frame, zero_offset, world,
      &hip_pos);

  const PiecewisePolynomial<double>& foot_traj_segment =
      foot_traj_.slice(foot_traj_.get_segment_index(t), 1);

  // Hip offset stuff
  std::vector<double> breaks = foot_traj_segment.get_segment_times();
  VectorXd breaks_vector = Map<VectorXd>(breaks.data(), breaks.size());
//  MatrixXd J_hip(3, plant_.num_velocities());
//  plant_.CalcJacobianTranslationalVelocity(*plant_context,
//      JacobianWrtVariable::kV,
//      hip_frame, zero_offset,
//      world, world, &J_hip);
//  double dt = breaks_vector[1] - breaks_vector[0];
  MatrixXd hip_points(3, 2);
//  hip_points << hip_pos, hip_pos + J_hip*x.tail(plant_.num_velocities()) * dt;
  // Velocity estimates are generally bad
  hip_points << hip_pos, hip_pos;
  PiecewisePolynomial<double> hip_offset =
      PiecewisePolynomial<double>::FirstOrderHold(breaks_vector, hip_points);

  return foot_traj_segment + hip_offset;
//  return foot_traj_segment;
}

void FlightFootTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*) this->EvalVectorInput(context, state_port_);
  VectorXd x = robot_output->GetState();
  double timestamp = robot_output->get_timestamp();

  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*) this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  auto* casted_traj =
      (PiecewisePolynomial<double>*) dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  switch ((int) fsm_state(0)) {
    case (FLIGHT):  // FLIGHT
      *casted_traj = generateFlightTraj(context, robot_output->GetState(),
          timestamp);
      break;
    default:break;
  }
}

}  // namespace dairlib::examples::Cassie::osc_jump
