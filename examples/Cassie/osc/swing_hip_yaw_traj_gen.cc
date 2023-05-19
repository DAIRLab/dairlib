#include "examples/Cassie/osc/swing_hip_yaw_traj_gen.h"

#include <math.h>

#include <string>

#include "multibody/multibody_utils.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;

using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace cassie {
namespace osc {

SwingHipYawTrajGenerator::SwingHipYawTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::unordered_map<int, int>& fsm_to_joint_idx_map,
    double phase_duration)
    : plant_(plant),
      fsm_to_joint_idx_map_(fsm_to_joint_idx_map),
      phase_duration_(phase_duration) {
  // Input/Output Setup
  fsm_port_ =
      this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();
  state_port_ = this->DeclareVectorInputPort(
                        "x, u, t", OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
                    .get_index();
  des_yaw_port_ =
      this->DeclareVectorInputPort("pelvis_yaw", BasicVector<double>(1))
          .get_index();

  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(VectorXd(0));
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("des_swing_hip_yaw", traj_inst,
                                  &SwingHipYawTrajGenerator::CalcDesiredTraj);
}

void SwingHipYawTrajGenerator::CalcDesiredTraj(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // The desired trajectory creates a position error jump, so probably not good
  // for the hardware.
  // TODO: will need to modify this des traj profile if we want to use it on
  //  hardware

  // Finite state
  int fsm_state = static_cast<int>(
      this->EvalVectorInput(context, fsm_port_)->get_value()(0));

  // Read in desired yaw angle
  double des_yaw_vel = dynamic_cast<const BasicVector<double>*>(
                           this->EvalVectorInput(context, des_yaw_port_))
                           ->get_value()(0);
  double des_yaw_pos = des_yaw_vel * phase_duration_ / 2;

  // Read in current state
  double current_yaw =
      dynamic_cast<const OutputVector<double>*>(
          this->EvalVectorInput(context, state_port_))
          ->GetPositions()(fsm_to_joint_idx_map_.at(fsm_state));

  // Construct the PiecewisePolynomial.
  const std::vector<double> breaks = {context.get_time(),
                                      context.get_time() + phase_duration_};
  std::vector<MatrixXd> knots(breaks.size(), MatrixXd::Zero(1, 1));
  //  knots[0](0) = current_yaw;
  knots[0](0) = des_yaw_pos;
  knots[1](0) = des_yaw_pos;
  const auto pp = PiecewisePolynomial<double>::FirstOrderHold(breaks, knots);

  // Assign traj
  auto* pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *pp_traj = pp;
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
