#pragma once
#include "alip_utils.h"

namespace dairlib::systems::controllers {

using drake::Vector6d;

/*!
 * Calculates the pendulum state [theta_y, theta_x, r, L_y, L_x, r_dot]
 */
Vector6d CalcPendulumState(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context,
    alip_utils::PointOnFramed stance_foot, std::string floating_base_body);

}