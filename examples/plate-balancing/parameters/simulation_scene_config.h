#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {

/**
 * @brief Scene configuration for the simulation in plate balancing.
 *
 * Contains environment model paths, their poses, and camera configuration for
 * simulation visualization.
 */
struct SimulationSceneConfig {
  std::vector<std::string>
      environment_models;  ///< List of environment model file paths.
  std::vector<Eigen::VectorXd>
      environment_orientations;  ///< List of orientation vectors for each
                                 ///< environment model.
  std::vector<Eigen::VectorXd>
      environment_positions;  ///< List of position vectors for each environment
                              ///< model.

  Eigen::VectorXd camera_pose;    ///< Camera pose in the world frame.
  Eigen::VectorXd camera_target;  ///< Camera target point in the world frame.

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(environment_models));
    a->Visit(DRAKE_NVP(environment_orientations));
    a->Visit(DRAKE_NVP(environment_positions));
    DRAKE_DEMAND(environment_models.size() == environment_orientations.size());
    DRAKE_DEMAND(environment_models.size() == environment_positions.size());

    a->Visit(DRAKE_NVP(camera_pose));
    a->Visit(DRAKE_NVP(camera_target));
  }
};

}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib