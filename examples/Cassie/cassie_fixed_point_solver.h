#include "examples/Cassie/cassie_utils.h"

namespace dairlib {
/// Utility method to solve for a fixed point for Casie
/// This is a very narrow method, but could be useful across different
/// Cassie examples
/// @param filename The URDF file
/// @param height The pelvis height to solve for
/// @param mu Coefficient of friction
/// @param min_normal_force Minimum normal force at each contact point
/// @param linear_friction_cone use linear approximation of friction cone
/// @param q Pointer to the resulting position
/// @param u Pointer to the resulting actuation input
/// @param lambda Pointer to the constraint force, though not that useful
///    without exposing the underlying constraint set/plant, it is included
///    here for debugging purposes
/// @param draw_pose Draw the resulting pose via DrakeVisualizer. Default=false
void CassieFixedPointSolver(std::string filename, double height, double mu,
    double min_normal_force, bool linear_friction_cone,
    Eigen::VectorXd* q_result, Eigen::VectorXd* u_result,
    Eigen::VectorXd* lambda_result, bool draw_pose = false);

}  // namespace dairlib
