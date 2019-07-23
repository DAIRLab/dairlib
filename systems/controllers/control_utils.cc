#include "systems/controllers/control_utils.h"
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

using Eigen::Vector2d;
using Eigen::Vector3d;

namespace dairlib {
namespace systems {

Vector2d ImposeHalfplaneGaurd(Vector2d foot_placement_pos,
    int left_stance_state, int right_stance_state, int state,
    double yaw, Vector2d CoM, Vector2d stance_foot_pos,
    double center_line_offset){
  // Get the shifting direction of the center line
  Vector2d shift_foothold_dir;
  if (state == right_stance_state) {
    shift_foothold_dir << cos(yaw + M_PI * 1 / 2),
                       sin(yaw + M_PI * 1 / 2);
  } else {
    shift_foothold_dir << cos(yaw + M_PI * 3 / 2),
                       sin(yaw + M_PI * 3 / 2);
  }

  Vector3d yaw_heading(cos(yaw), sin(yaw), 0);
  Vector3d CoM_to_stance_foot(stance_foot_pos(0) - CoM(0),
                              stance_foot_pos(1) - CoM(1),
                              0);
  Vector3d heading_cross_CoM_to_stance_foot =
      yaw_heading.cross(CoM_to_stance_foot);

  // Select the point which lies on the line
  Vector2d CoM_or_stance_foot;
  if ( ((state == right_stance_state) &&
        (heading_cross_CoM_to_stance_foot(2) > 0)) ||
       ((state == left_stance_state) &&
        (heading_cross_CoM_to_stance_foot(2) < 0)) ) {
    CoM_or_stance_foot << stance_foot_pos(0), stance_foot_pos(1);
  } else {
    CoM_or_stance_foot << CoM(0), CoM(1);
  }

  Vector2d shifted_CoM_or_stance_foot(
      CoM_or_stance_foot(0) + shift_foothold_dir(0) * center_line_offset,
      CoM_or_stance_foot(1) + shift_foothold_dir(1) * center_line_offset);
  Vector3d CoM_or_stance_foot_to_fp(
      foot_placement_pos(0) - shifted_CoM_or_stance_foot(0),
      foot_placement_pos(1) - shifted_CoM_or_stance_foot(1),
      0);

  // Check if the foot placement position is in the halfplace. If not, we
  // project it onto the line.
  Vector3d heading_cross_CoM_to_fp =
      yaw_heading.cross(CoM_or_stance_foot_to_fp);
  if ( ((state == right_stance_state) && (heading_cross_CoM_to_fp(2) < 0)) ||
       ((state == left_stance_state) && (heading_cross_CoM_to_fp(2) > 0)) ) {
    Vector3d perp_heading_dir = heading_cross_CoM_to_fp.cross(yaw_heading);
    perp_heading_dir.normalize();
    Vector3d projection_of_CoM_or_stance_foot_to_fp =
        (CoM_or_stance_foot_to_fp.dot(perp_heading_dir)) * perp_heading_dir;
    foot_placement_pos(0) -= projection_of_CoM_or_stance_foot_to_fp(0);
    foot_placement_pos(1) -= projection_of_CoM_or_stance_foot_to_fp(1);
  }

  return foot_placement_pos;
}


Vector2d ImposeStepLengthGaurd(Vector2d foot_placement_pos,
    Vector2d CoM, double max_dist){
  Vector2d com_to_fp = foot_placement_pos - CoM.head(2);
  if ( com_to_fp.norm() > max_dist ) {
    std::cout << "Step length limit reached. It's " <<
        com_to_fp.norm() - max_dist << " (m) more than max.\n";
    Vector2d normalized_com_to_fp = com_to_fp.normalized();
    foot_placement_pos = CoM + normalized_com_to_fp * max_dist;
  }
  return foot_placement_pos;
}


}  // namespace systems
}  // namespace dairlib
