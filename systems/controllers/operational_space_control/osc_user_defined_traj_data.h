// TODO(yminchen):
// Do this after you finish OSC. This function is for tracking rom traj
// I think this is how you will use the class
// - pass ChildOfOscUserDefinedTraj's address into OscUserDefinedTrajData
//   (it could be into either OscUserDefinedTrajData's method or constructor)
// - the argument of such method/construct is OscUserDefinedTraj* obj_ptr
// - then you can call obj_ptr->Position

#pragma once

#include <Eigen/Dense>

namespace dairlib {
namespace systems {
namespace controllers {

// OscUserDefinedTrajData can be used when users want to explicitly define the
// position `r` that they want to track.
// The position `r` is a function of the configuration of the robot `q`.
class OscUserDefinedTrajData {
 public:
  OscUserDefinedTrajData(OscUserDefinedTraj* user_defined_traj);

  OscUserDefinedTrajData() {}  // Default constructor

 protected:

 private:
  // In CalcPosition(), it calls Position() which is implmented by the children.
  Eigen::VectorXd CalcPosition(const Eigen::VectorXd& q);
  // Numerically calculating jacobian.
  Eigen::MatrixXd CalcJacobian(const Eigen::VectorXd& q);
  // Numerically calculating dJ/dt * v.
  Eigen::VectorXd CalcJacobianDotTimesV(const Eigen::VectorXd& q,
                                        const Eigen::VectorXd& v);

  // int n_r_;
  // int n_q_;

  OscUserDefinedTraj* user_defined_traj_;
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
