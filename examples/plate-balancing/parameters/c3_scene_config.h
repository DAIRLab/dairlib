#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {

/**
 * @brief Configuration parameters for the reduced plate balancing C3 scene.
 *
 * This struct holds file paths, model names, and geometric properties for the
 * Franka robot, end effector, and environment objects. It is intended to be
 * loaded from a YAML file.
 */
struct C3SceneConfig {
  std::string franka_model;        ///< Path to the Franka robot model file.
  std::string end_effector_model;  ///< Path to the end effector model file.
  std::string end_effector_name;   ///< Name of the end effector in the model.
  Eigen::Vector3d tool_attachment_frame;  ///< 3D vector specifying the tool's
                                          ///< attachment frame.
  double end_effector_thickness;          ///< Thickness of the end effector.
  std::string end_effector_lcs_model;  ///< Path to the local coordinate system
                                       ///< model for the end effector.
  std::vector<std::string> object_models;  ///< List of object model file paths
                                           ///< to be included in the scene.
  std::vector<std::string>
      environment_models;  ///< List of environment model file paths.
  std::vector<Eigen::VectorXd>
      environment_orientations;  ///< List of orientation vectors for each
                                 ///< environment model.
  std::vector<Eigen::VectorXd>
      environment_positions;  ///< List of position vectors for each environment
                              ///< model.

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(franka_model));
    a->Visit(DRAKE_NVP(end_effector_model));
    a->Visit(DRAKE_NVP(end_effector_name));
    a->Visit(DRAKE_NVP(tool_attachment_frame));
    a->Visit(DRAKE_NVP(end_effector_thickness));
    a->Visit(DRAKE_NVP(end_effector_lcs_model));
    a->Visit(DRAKE_NVP(object_models));
    a->Visit(DRAKE_NVP(environment_models));
    a->Visit(DRAKE_NVP(environment_orientations));
    a->Visit(DRAKE_NVP(environment_positions));
  }
};

}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib