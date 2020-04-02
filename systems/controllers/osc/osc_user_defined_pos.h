// TODO(yminchen):
// Finish writing this class. This class is for tracking rom traj
// I think this is how you will use the class
// - pass ChildOfOscUserDefinedPos's address into OscUserDefinedPosData
//   (it could be into either OscUserDefinedPosData's method or constructor)
// - the argument of such method/construct is OscUserDefinedPos* obj_ptr
// - then you can call obj_ptr->Position

#pragma once

#include <Eigen/Dense>

namespace dairlib {
namespace systems {
namespace controllers {

// OscUserDefinedPos is designed for the case when users want to explicitly
// define the position `r` which can not be obtained from Drake's API.
// The position `r` is a function of the configuration of the robot `q`.

// Requirement:
// - Users should template their position functions in derived classes.
class OscUserDefinedPos {
 public:
  // OscUserDefinedPos(/*int n_r, int n_q*/);

  OscUserDefinedPos() {}  // Default constructor

  // Users define their own position class in the derived class.
  virtual Eigen::VectorXd Position(const Eigen::VectorXd& q) const = 0;

 protected:
  // int n_r_;
  // int n_q_;

 private:
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
