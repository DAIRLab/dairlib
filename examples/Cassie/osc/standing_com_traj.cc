#include "examples/Cassie/osc/standing_com_traj.h"

#include <math.h>
#include <string>

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
namespace osc {

StandingComTraj::StandingComTraj(const RigidBodyTree<double>& tree,
    int pelvis_idx, int left_foot_idx, int right_foot_idx, double height) :
  tree_(tree),
  pelvis_idx_(pelvis_idx),
  left_foot_idx_(left_foot_idx),
  right_foot_idx_(right_foot_idx),
  height_(height) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
                  tree.get_num_positions(),
                  tree.get_num_velocities(),
                  tree.get_num_actuators())).get_index();
  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(VectorXd(0));
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("com_traj", traj_inst,
      &StandingComTraj::CalcDesiredTraj);

  // Testing
  first_msg_time_ = std::make_unique<double>();
  *first_msg_time_ = -1;
}

void StandingComTraj::CalcDesiredTraj(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();

  // Kinematics cache and indices
  KinematicsCache<double> cache = tree_.CreateKinematicsCache();
  // Modify the quaternion in the begining when the state is not received from
  // the robot yet
  // Always remember to check 0-norm quaternion when using doKinematics
  multibody::SetZeroQuaternionToIdentity(&q);
  cache.initialize(q);
  tree_.doKinematics(cache);

  // Get center of left/right feet contact points positions
  Vector3d left_front = tree_.transformPoints(cache,
      front_contact_disp_, left_foot_idx_, 0);
  Vector3d left_rear = tree_.transformPoints(cache,
      rear_contact_disp_, left_foot_idx_, 0);
  Vector3d right_front = tree_.transformPoints(cache,
      front_contact_disp_, right_foot_idx_, 0);
  Vector3d right_rear = tree_.transformPoints(cache,
      rear_contact_disp_, right_foot_idx_, 0);
  Vector3d feet_center = (left_front + left_rear + right_front + right_rear)/4;


  Vector3d desired_com_pos(feet_center(0), feet_center(1), feet_center(2) + height_);
  // cout << "desired_com_pos = " << desired_com_pos.transpose() << endl;

  // Assign traj
  PiecewisePolynomial<double>* casted_traj = (PiecewisePolynomial<double>*)
      dynamic_cast<PiecewisePolynomial<double>*> (traj);
  *casted_traj = PiecewisePolynomial<double>(desired_com_pos);
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib


