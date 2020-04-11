#include "examples/Cassie/osc/heading_traj_generator.h"

#include <math.h>
#include <string>

#include "attic/multibody/rigidbody_utils.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
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

HeadingTrajGenerator::HeadingTrajGenerator(const RigidBodyTree<double>& tree,
                                           int pelvis_idx)
    : tree_(tree), pelvis_idx_(pelvis_idx) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
                        tree.get_num_positions(), tree.get_num_velocities(),
                        tree.get_num_actuators()))
                    .get_index();
  des_yaw_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(VectorXd(0));
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("heading_traj", traj_inst,
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

  // Kinematics cache and indices
  KinematicsCache<double> cache = tree_.CreateKinematicsCache();
  // Modify the quaternion in the begining when the state is not received from
  // the robot yet
  // Always remember to check 0-norm quaternion when using doKinematics
  multibody::SetZeroQuaternionToIdentity(&q);
  cache.initialize(q);
  tree_.doKinematics(cache);

  // Get approximated heading angle of pelvis
  Vector3d pelvis_heading_vec =
      tree_.CalcBodyPoseInWorldFrame(cache, tree_.get_body(pelvis_idx_))
          .linear()
          .col(0);
  double approx_pelvis_yaw_i =
      atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));

  // Construct the PiecewisePolynomial.
  /// Given yaw position p_i and velocity v_i, we want to generate affine
  /// functions, p_i + v_i*t, for the desired trajectory. We use
  /// FirstOrderHold() to approximately generate the function, so we need to
  /// have the endpoint of the trajectory. We generate the endpoint by
  /// looking ahead what the position is in 0.1 second with fixed velocity v_i.
  /// Note that we construct trajectories in R^4 (quaternion space), so we need
  /// to transform the yaw trajectory into quaternion representation.
  double approx_pelvis_yaw_f = approx_pelvis_yaw_i + des_yaw_vel(0) * 0.1;
  Eigen::Vector4d pelvis_rotation_i(q(3), q(4), q(5), q(6));
  Eigen::Vector4d pelvis_rotation_f(cos(approx_pelvis_yaw_f / 2), 0, 0,
                                    sin(approx_pelvis_yaw_f / 2));

  const std::vector<double> breaks = {context.get_time(),
                                      context.get_time() + 0.1};
  std::vector<MatrixXd> knots(breaks.size(), MatrixXd::Zero(4, 1));
  knots[0] = pelvis_rotation_i;
  knots[1] = pelvis_rotation_f;
  const auto pp = PiecewisePolynomial<double>::FirstOrderHold(breaks, knots);

  // Assign traj
  PiecewisePolynomial<double>* pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *pp_traj = pp;
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
