#include "examples/Cassie/osc/heading_traj_generator.h"

#include <math.h>

#include <string>
#include <drake/common/trajectories/piecewise_quaternion.h>

#include "multibody/multibody_utils.h"

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
using drake::trajectories::PiecewiseQuaternionSlerp;

namespace dairlib {
namespace cassie {
namespace osc {

HeadingTrajGenerator::HeadingTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName("pelvis")) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        "x, u, t", OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
                    .get_index();
  des_yaw_port_ =
      this->DeclareVectorInputPort("pelvis_yaw", BasicVector<double>(1))
          .get_index();
  // Provide an instance to allocate the memory first (for the output)
  PiecewiseQuaternionSlerp<double> empty_slerp_traj;
  drake::trajectories::Trajectory<double>& traj_inst = empty_slerp_traj;
  this->DeclareAbstractOutputPort("pelvis_quat", traj_inst,
                                  &HeadingTrajGenerator::CalcHeadingTraj);
}

void HeadingTrajGenerator::CalcHeadingTraj(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in desired yaw angle
  const BasicVector<double>* des_yaw_output =
      (BasicVector<double>*)this->EvalVectorInput(context, des_yaw_port_);
  VectorXd des_yaw_vel = des_yaw_output->get_value();

  // Read in current state
  const OutputVector<double>* robotOutput =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q = robotOutput->GetPositions();

  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  Quaterniond quat(
      plant_.EvalBodyPoseInWorld(*context_, pelvis_).rotation().matrix());
  quat.normalize();


  double dt = 0.1;
  double des_delta_yaw = des_yaw_vel(0) * dt;
  Eigen::Vector4d pelvis_rotation_i;
  pelvis_rotation_i << quat.w(), 0, 0, quat.z();
  pelvis_rotation_i.normalize();
  Quaterniond init_quat(pelvis_rotation_i(0), pelvis_rotation_i(1),
                        pelvis_rotation_i(2), pelvis_rotation_i(3));
  init_quat.normalize();
  Quaterniond relative_quat(cos(des_delta_yaw / 2), 0, 0,
                            sin(des_delta_yaw / 2));
  Quaterniond final_quat = relative_quat * init_quat;

  const std::vector<double> breaks = {context.get_time(),
                                      context.get_time() + dt};
  const auto pp = drake::trajectories::PiecewiseQuaternionSlerp<double>(
      breaks, {quat, final_quat});

  // Assign traj
  auto* pp_traj =
      (PiecewiseQuaternionSlerp<double>*)dynamic_cast<PiecewiseQuaternionSlerp<double>*>(
          traj);
  *pp_traj = pp;
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
