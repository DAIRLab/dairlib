#pragma once

#include <Eigen/Dense>

#include <drake/multibody/plant/multibody_plant.h>

namespace dairlib {
namespace systems {
namespace controllers {

// TODO: use these functions in operational_space_control.cc

// Functions copy from operational_space_control.cc
// I copied it out because it's also used in osc_tracking_data, and didn't
// remove the one in operational_space_control, because there might still be
// changes going on in the code from upstream
Eigen::MatrixXd PositionMapFromSpringToNoSpring(
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::multibody::MultibodyPlant<double>& plant_wo_spr);
Eigen::MatrixXd VelocityMapFromSpringToNoSpring(
    const drake::multibody::MultibodyPlant<double>& plant_w_spr,
    const drake::multibody::MultibodyPlant<double>& plant_wo_spr);

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
