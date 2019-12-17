#include "examples/jumping/torso_traj.h"

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

TorsoTraj::TorsoTraj(const RigidBodyTree<double>& tree,
                     PiecewisePolynomial<double> torso_traj)
    : tree_(tree), torso_angle_traj_(torso_traj) {
  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  this->set_name("pelvis_rot_traj");
  this->DeclareAbstractOutputPort("pelvis_rot_traj", traj_inst,
                                  &TorsoTraj::CalcTraj);

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
                        tree.get_num_positions(), tree.get_num_velocities(),
                        tree.get_num_actuators()))
                    .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  // DeclarePerStepDiscreteUpdateEvent(&FlightFootTraj::DiscreteVariableUpdate);
}

void TorsoTraj::CalcTraj(const drake::systems::Context<double>& context,
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

  *casted_traj = torso_angle_traj_;
}

}  // namespace osc
}  // namespace jumping
}  // namespace examples
}  // namespace dairlib
