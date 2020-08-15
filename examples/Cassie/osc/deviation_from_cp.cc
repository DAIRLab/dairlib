#include "examples/Cassie/osc/deviation_from_cp.h"

#include <math.h>
#include <string>

#include "multibody/multibody_utils.h"

#include "drake/math/quaternion.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;

using drake::multibody::JacobianWrtVariable;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;

namespace dairlib {
namespace cassie {
namespace osc {

DeviationFromCapturePoint::DeviationFromCapturePoint(
    const drake::multibody::MultibodyPlant<double>& plant,
    Context<double>* context)
    : plant_(plant),
    context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName("pelvis")) {
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  xy_port_ = this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();
  this->DeclareVectorOutputPort(BasicVector<double>(2),
                                &DeviationFromCapturePoint::CalcFootPlacement);

}

void DeviationFromCapturePoint::CalcFootPlacement(
    const Context<double>& context, BasicVector<double>* output) const {
  // Read in finite state machine
  const BasicVector<double>* des_hor_vel_output =
      (BasicVector<double>*)this->EvalVectorInput(context, xy_port_);
  VectorXd des_hor_vel = des_hor_vel_output->get_value();

  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();

  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  // Get center of mass position and velocity
  MatrixXd J(3, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      *context_, JacobianWrtVariable::kV, world_, world_, &J);
  Vector3d com_vel = J * v;

  // Extract quaternion from floating base position
  Quaterniond Quat(q(0), q(1), q(2), q(3));
  Quaterniond Quat_conj = Quat.conjugate();
  Vector4d quat(q.head(4));
  Vector4d quad_conj(Quat_conj.w(), Quat_conj.x(), Quat_conj.y(),
                     Quat_conj.z());

  // Calculate local velocity
  Vector3d local_com_vel = drake::math::quatRotateVec(quad_conj, com_vel);

  //////////////////// Sagital ////////////////////
  Vector3d delta_CP_sagital_3D_global(0, 0, 0);

  // Position Control
  double com_vel_sagital = local_com_vel(0);
  double des_sagital_vel = des_hor_vel(0);

  // Velocity control
  double delta_CP_sagital =
      -k_fp_ff_sagital_ * des_sagital_vel -
      k_fp_fb_sagital_ * (des_sagital_vel - com_vel_sagital);
  Vector3d delta_CP_sagital_3D_local(delta_CP_sagital, 0, 0);
  delta_CP_sagital_3D_global =
      drake::math::quatRotateVec(quat, delta_CP_sagital_3D_local);

  //////////////////// Lateral ////////////////////  TODO(yminchen): tune this
  Vector3d delta_CP_lateral_3D_global(0, 0, 0);

  // Position Control
  double com_vel_lateral = local_com_vel(1);
  double des_lateral_vel = des_hor_vel(1);

  // Velocity control
  double delta_CP_lateral =
      -k_fp_ff_lateral_ * des_lateral_vel -
      k_fp_fb_lateral_ * (des_lateral_vel - com_vel_lateral);
  Vector3d delta_CP_lateral_3D_local(0, delta_CP_lateral, 0);
  delta_CP_lateral_3D_global =
      drake::math::quatRotateVec(quat, delta_CP_lateral_3D_local);

  // Assign foot placement
  output->get_mutable_value() =
      (delta_CP_sagital_3D_global + delta_CP_lateral_3D_global).head(2);
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
