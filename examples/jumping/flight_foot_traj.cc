#include "examples/jumping/flight_foot_traj.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib {
namespace examples {
namespace jumping {
namespace osc {

FlightFootTraj::FlightFootTraj(const RigidBodyTree<double>& tree, int hip_idx,
                               int left_foot_idx, int right_foot_idx,
                               bool isLeftFoot,
                               PiecewisePolynomial<double> foot_traj,
                               double height, double foot_offset)
    : tree_(tree),
      hip_idx_(hip_idx),
      left_foot_idx_(left_foot_idx),
      right_foot_idx_(right_foot_idx),
      isLeftFoot_(isLeftFoot),
      foot_traj_(foot_traj),
      height_(height) {
  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  if (isLeftFoot) {
    this->set_name("l_foot_traj");
    this->DeclareAbstractOutputPort("l_foot_traj", traj_inst,
                                    &FlightFootTraj::CalcTraj);
    foot_offset_ = -foot_offset;
  } else {
    this->set_name("r_foot_traj");
    this->DeclareAbstractOutputPort("r_foot_traj", traj_inst,
                                    &FlightFootTraj::CalcTraj);
    foot_offset_ = foot_offset;
  }

  // Input/Output Setup
  state_port_ = this
                    ->DeclareVectorInputPort(OutputVector<double>(
                        tree.get_num_positions(), tree.get_num_velocities(),
                        tree.get_num_actuators()))
                    .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  // DeclarePerStepDiscreteUpdateEvent(&FlightFootTraj::DiscreteVariableUpdate);
}

/*
  Move the feet relative to the COM
  The trajectory of the COM cannot be altered, so must solve for
  foot positions as a function of COM.
*/
PiecewisePolynomial<double> FlightFootTraj::generateFlightTraj(
    const drake::systems::Context<double>& context, VectorXd* q, VectorXd* v,
    double t) const {
  // Kinematics cache and indices
  KinematicsCache<double> cache = tree_.CreateKinematicsCache();
  // Modify the quaternion in the begining when the state is not received from
  // the robot yet
  // Always remember to check 0-norm quaternion when using doKinematics
  multibody::SetZeroQuaternionToIdentity(q);
  cache.initialize(*q, *v);
  tree_.doKinematics(cache);

  // Vector3d pt_on_foot = Eigen::VectorXd::Zero(3);
  // find a function that calculates the center of mass for a rigidbodytree

//  Vector3d center_of_mass = tree_.centerOfMass(cache);
//  Vector3d com_velocity = tree_.centerOfMassJacobian(cache) * (*v);

  Vector3d torso_offset;
  torso_offset << 0, 0, -0.2;
  Vector3d hip =
      tree_.transformPoints(cache, torso_offset, hip_idx_, 0);
  Vector3d hip_vel =
      tree_.transformPointsJacobian(cache, torso_offset, hip_idx_, 0,
          false) * (*v);

  int segment_idx = -1;  // because times start at 0
  for (double t0 : foot_traj_.get_segment_times()) {
    if (t0 > t) {
      break;
    }
    ++segment_idx;
  }
  if(segment_idx == foot_traj_.get_number_of_segments()){
    segment_idx--;
  }

  const PiecewisePolynomial<double>& foot_traj_segment =
      foot_traj_.slice(segment_idx, 1);
  std::vector<double> breaks = foot_traj_segment.get_segment_times();
  VectorXd breaks_vector = Map<VectorXd>(breaks.data(), breaks.size());
  //  MatrixXd com_pos(3, breaks.size());
  double dt = breaks[1] - breaks[0];
//  MatrixXd com_pos(3, 2);
//  com_pos << center_of_mass, center_of_mass + com_velocity * dt;
//  PiecewisePolynomial<double> com_offset =
//      PiecewisePolynomial<double>::FirstOrderHold(breaks_vector, com_pos);
  MatrixXd hip_pos(3, 2);
  hip_pos << hip, hip + hip_vel * dt;
//  hip_pos << hip, hip;
  PiecewisePolynomial<double> hip_offset =
      PiecewisePolynomial<double>::FirstOrderHold(breaks_vector, hip_pos);
  return foot_traj_segment + hip_offset;
}

void FlightFootTraj::CalcTraj(
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

  PiecewisePolynomial<double>* casted_traj =
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

}  // namespace osc
}  // namespace jumping
}  // namespace examples
}  // namespace dairlib
