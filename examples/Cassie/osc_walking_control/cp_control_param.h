#pragma once
#include "systems/framework/output_vector.h"

using Eigen::Vector2d;

namespace dairlib {

// The CoM Traj is a fixed point. This is used to test operational space control.
const bool isDebugAndStandingStill = false;


const double stanceDurationPerLeg = 0.35;

// Position control parameters
double circle_radius_of_no_turning = 1;
Vector2d global_target_position_ = VectorXd::Zero(2);



}  // namespace dairlib


