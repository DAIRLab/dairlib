#include <Eigen/Core>
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;


namespace dairlib{
namespace systems{

Eigen::MatrixXd hessian_of_squared_quaternion_angle_difference(
    const Eigen::VectorXd& quat,
    const Eigen::VectorXd& quat_desired);

} // namespace systems
} // namespace dairlib
