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
  OscUserDefinedPos() {}  // Default constructor

  // Users define their own position class in the derived class.
  virtual Eigen::VectorXd Position(const Eigen::VectorXd& q) const = 0;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
