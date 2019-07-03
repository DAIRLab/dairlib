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

// OscUserDefinedTraj can be used when users want to explicitly define the
// position `r` that they want to track.
// The position `r` is a function of the configuration of the robot `q`.

// Users should template their position functions in derived classes.
class OscUserDefinedTraj {
 public:
  OscUserDefinedTraj(/*int n_r, int n_q*/);

  OscUserDefinedTraj() {}  // Default constructor

  // Users define their own position class in teh derived class.
  virtual Eigen::VectorXd Position(const Eigen::VectorXd& q) = 0;

 protected:

  // int n_r_;
  // int n_q_;

 private:
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
