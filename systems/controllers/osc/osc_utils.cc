#include "systems/controllers/osc/osc_utils.h"
#include "multibody/multibody_utils.h"

#include <string>

using drake::multibody::MultibodyPlant;
using Eigen::MatrixXd;
using std::string;

namespace dairlib {
namespace systems {
namespace controllers {

Eigen::MatrixXd PositionMapFromSpringToNoSpring(
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr) {
  const std::map<string, int>& pos_map_w_spr =
      multibody::makeNameToPositionsMap(plant_w_spr);
  const std::map<string, int>& pos_map_wo_spr =
      multibody::makeNameToPositionsMap(plant_wo_spr);

  // Initialize the mapping from spring to no spring
  MatrixXd ret =
      MatrixXd::Zero(plant_wo_spr.num_positions(), plant_w_spr.num_positions());
  for (auto pos_pair_wo_spr : pos_map_wo_spr) {
    bool successfully_added = false;
    for (auto pos_pair_w_spr : pos_map_w_spr) {
      if (pos_pair_wo_spr.first == pos_pair_w_spr.first) {
        ret(pos_pair_wo_spr.second, pos_pair_w_spr.second) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }

  return ret;
}

Eigen::MatrixXd VelocityMapFromSpringToNoSpring(
    const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr) {
  const std::map<string, int>& vel_map_w_spr =
      multibody::makeNameToVelocitiesMap(plant_w_spr);
  const std::map<string, int>& vel_map_wo_spr =
      multibody::makeNameToVelocitiesMap(plant_wo_spr);

  // Initialize the mapping from spring to no spring
  MatrixXd ret = MatrixXd::Zero(plant_wo_spr.num_velocities(),
                                plant_w_spr.num_velocities());
  for (auto vel_pair_wo_spr : vel_map_wo_spr) {
    bool successfully_added = false;
    for (auto vel_pair_w_spr : vel_map_w_spr) {
      if (vel_pair_wo_spr.first == vel_pair_w_spr.first) {
        ret(vel_pair_wo_spr.second, vel_pair_w_spr.second) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }

  return ret;
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
