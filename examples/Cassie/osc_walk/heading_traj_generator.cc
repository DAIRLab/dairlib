#include "examples/Cassie/osc_walk/heading_traj_generator.h"

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
namespace osc_walk {

HeadingTrajGenerator::HeadingTrajGenerator() {
  // Input/Output Setup
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
  VectorXd des_yaw = des_yaw_output->get_value();

  // Get quaternion
  Eigen::Vector4d desired_pelvis_rotation(cos(des_yaw(0) / 2), 0, 0,
                                          sin(des_yaw(0) / 2));

  // Assign traj
  PiecewisePolynomial<double>* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *casted_traj = PiecewisePolynomial<double>(desired_pelvis_rotation);
}

}  // namespace osc_walk
}  // namespace cassie
}  // namespace dairlib
