#include "examples/Cassie/osc_walk/heading_control.h"

#include <math.h>
#include <string>

#include "examples/Cassie/osc_walk/cp_control_common_func.h"
#include "attic/multibody/rigidbody_utils.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using dairlib::systems::OutputVector;

using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::BasicVector;

using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace cassie {
namespace osc_walk {

HeadingControl::HeadingControl(const RigidBodyTree<double>& tree,
    int pelvis_idx,
    Vector2d global_target_position, Eigen::Vector2d params_of_no_turning) :
  tree_(tree),
  pelvis_idx_(pelvis_idx),
  global_target_position_(global_target_position),
  params_of_no_turning_(params_of_no_turning) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
                  tree.get_num_positions(),
                  tree.get_num_velocities(),
                  tree.get_num_actuators())).get_index();
  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(VectorXd(0));
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("heading_traj", traj_inst,
      &HeadingControl::CalcHeadingAngle);
}

void HeadingControl::CalcHeadingAngle(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robotOutput = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  VectorXd q = robotOutput->GetPositions();
  VectorXd v = robotOutput->GetVelocities();

  // Kinematics cache and indices
  KinematicsCache<double> cache = tree_.CreateKinematicsCache();
  // Modify the quaternion in the begining when the state is not received from
  // the robot yet
  // Always remember to check 0-norm quaternion when using doKinematics
  multibody::SetZeroQuaternionToIdentity(&q);
  cache.initialize(q);
  tree_.doKinematics(cache);

  // Get center of mass position and velocity
  Vector3d CoM = tree_.centerOfMass(cache);

  // Get proximated heading angle of pelvis
  Vector3d pelvis_heading_vec = tree_.CalcBodyPoseInWorldFrame(
      cache, tree_.get_body(pelvis_idx_)).linear().col(0);
  double approx_pelvis_yaw = atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));

  // Get desried heading direction
  Vector2d global_CoM_to_target_pos =
      global_target_position_ - CoM.segment(0, 2);
  double desried_heading_pos = GetDesiredYawAngle(approx_pelvis_yaw,
      global_CoM_to_target_pos, params_of_no_turning_);

  // Get quaternion
  Eigen::Vector4d desired_pelvis_rotation(cos(desried_heading_pos/2),
                                          0,
                                          0,
                                          sin(desried_heading_pos/2));

  // Assign traj
  PiecewisePolynomial<double>* casted_traj = (PiecewisePolynomial<double>*)
      dynamic_cast<PiecewisePolynomial<double>*> (traj);
  *casted_traj = PiecewisePolynomial<double>(desired_pelvis_rotation);
}

}  // namespace osc_walk
}  // namespace cassie
}  // namespace dairlib


