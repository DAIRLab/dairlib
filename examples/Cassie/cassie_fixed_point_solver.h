#include "examples/Cassie/cassie_utils.h"

namespace dairlib {
/// Utility method to solve for a fixed point for Cassie
/// This is a very narrow method, but could be useful across different
/// Cassie examples
/// @param plant 
/// @param height The pelvis height to solve for
/// @param mu Coefficient of friction
/// @param min_normal_force Minimum normal force at each contact point
/// @param linear_friction_cone use linear approximation of friction cone
/// @param toe_spread y-position of the toes (constrained)
/// @param q Pointer to the resulting position
/// @param u Pointer to the resulting actuation input
/// @param lambda Pointer to the constraint force, though not that useful
///    without exposing the underlying constraint set/plant, it is included
///    here for debugging purposes
/// @param visualize_model_urdf Draw the resulting pose via DrakeVisualizer
///    Requires a model file location to draw. Default is "" (no draw)
void CassieFixedPointSolver(
    const drake::multibody::MultibodyPlant<double>& plant,
    double height, double mu, double min_normal_force,
    bool linear_friction_cone, double toe_spread, Eigen::VectorXd* q_result,
    Eigen::VectorXd* u_result, Eigen::VectorXd* lambda_result,
    std::string visualize_model_urdf = "");

/// Utility method to solve for loop constraints for Cassie for a neutral
/// position
/// @param plant 
/// @param q Pointer to the resulting position
/// @param u Pointer to the resulting actuation input
/// @param lambda Pointer to the constraint force, though not that useful
///    without exposing the underlying constraint set/plant, it is included
///    here for debugging purposes
/// @param visualize_model_urdf Draw the resulting pose via DrakeVisualizer
///    Requires a model file location to draw. Default is "" (no draw)
void CassieFixedBaseFixedPointSolver(
    const drake::multibody::MultibodyPlant<double>& plant,
    Eigen::VectorXd* q_result, Eigen::VectorXd* u_result,
    Eigen::VectorXd* lambda_result, std::string visualize_model_urdf = "");

}  // namespace dairlib
