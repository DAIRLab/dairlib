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

namespace dairlib::examples::Cassie::osc_jump {

FlightFootTrajGenerator::FlightFootTrajGenerator(const MultibodyPlant<double>& plant,
                               const string& hip_name, bool isLeftFoot,
                               const PiecewisePolynomial<double>& foot_traj,
                               double height)
    : plant_(plant),
      hip_name_(hip_name),
      foot_traj_(foot_traj),
      height_(height) {
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

  // DeclarePerStepDiscreteUpdateEvent(&FlightFootTrajGenerator::DiscreteVariableUpdate);
}

/*
  Move the feet relative to the COM
  The trajectory of the COM cannot be altered, so must solve for
  foot positions as a function of COM.
*/
PiecewisePolynomial<double> FlightFootTrajGenerator::generateFlightTraj(
    const drake::systems::Context<double>& context, VectorXd* q, VectorXd* v,
    double t) const {
  // Always remember to check 0-norm quaternion when using doKinematics
  auto plant_context = createContext(plant_, *q, *v);
  // Vector3d pt_on_foot = Eigen::VectorXd::Zero(3);

  const drake::multibody::BodyFrame<double>& world = plant_.world_frame();
  const drake::multibody::BodyFrame<double>& hip_frame =
      plant_.GetBodyByName(hip_name_).body_frame();
  Vector3d pt_on_hip = Vector3d::Zero();
  Vector3d hip_pos;
  plant_.CalcPointsPositions(*plant_context, hip_frame, pt_on_hip, world,
                             &hip_pos);

  const PiecewisePolynomial<double>& foot_traj_segment =
      foot_traj_.slice(foot_traj_.get_segment_index(t), 1);
  std::vector<double> breaks = foot_traj_segment.get_segment_times();
  VectorXd breaks_vector = Map<VectorXd>(breaks.data(), breaks.size());

  MatrixXd hip_points(3, 2);
  hip_points << hip_pos, hip_pos;
  //  hip_pos << hip, hip;
  PiecewisePolynomial<double> hip_offset =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, hip_points);
  return foot_traj_segment + hip_offset;
}

void FlightFootTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();
  double timestamp = robot_output->get_timestamp();

  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  switch ((int)fsm_state(0)) {
    case (2):  // FLIGHT
      *casted_traj = generateFlightTraj(context, &q, &v, timestamp);
      break;
    default:
      break;
  }
}

}  // namespace dairlib::examples::Cassie::osc_jump
}  // namespace dairlib::examples::Cassie::osc_jump
}  // namespace examples
}  // namespace dairlib