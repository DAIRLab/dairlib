#pragma once

#include <string>
#include <vector>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace dairlib {
namespace multibody {

/// Class for using Drake Visualizer to draw multiple poses of the same robot.
/// Basic use:
///   MultiposeVisualizer visualizer = MultiposeVisualizer(URDF_file,
///   num_poses); visualizer.DrawPoses(poses);
///
/// where poses is an Eigen matrix (num_positions x num_poses)
///
/// Since this uses Drake Visualizer, it can be the only currently running
/// such use case. Does not currently support additional objects, though these
/// could be added at a later date.
class MultiposeVisualizer {
 public:
  /// Constructor
  /// @param model_file A full path to model (e.g. through FindResourceOrThrow)
  /// @param num_poses The number of simultaneous poses to draw (fixed)
  /// @param weld_frame_to_world Welds the frame of the given name to the world
  MultiposeVisualizer(std::string model_file, int num_poses,
                      std::string weld_frame_to_world = "");

  /// Constructor, scales all alpha transparencies by a constant
  /// @param model_file A full path to model (e.g. through FindResourceOrThrow)
  /// @param num_poses The number of simultaneous poses to draw (fixed)
  /// @param alpha_scale Scales the transparency alpha field of all bodies
  /// @param weld_frame_to_world Welds the frame of the given name to the world
  MultiposeVisualizer(std::string model_file, int num_poses, double alpha_scale,
                      std::string weld_frame_to_world = "");

  /// Constructor, scales all alpha transparencies by a constant
  /// @param model_file A full path to model (e.g. through FindResourceOrThrow)
  /// @param num_poses The number of simultaneous poses to draw (fixed)
  /// @param alpha_scale Vector, of same length as num_poses. Provideas variable
  /// scaling of the transparency alpha field of all bodies, indexed by pose
  /// @param weld_frame_to_world Welds the frame of the given name to the world
  MultiposeVisualizer(std::string model_file, int num_poses,
                      const Eigen::VectorXd& alpha_scale,
                      std::string weld_frame_to_world = "");

  /// Draws the poses in the given (num_positions x num_poses) matrix
  /// Note: the matrix can have extra rows (e.g. velocities), which will be
  /// ignored.
  void DrawPoses(Eigen::MatrixXd poses);

 private:
  int num_poses_;
  drake::multibody::MultibodyPlant<double>* plant_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagram_context_;
  std::vector<drake::multibody::ModelInstanceIndex> model_indices_;
};

}  // namespace multibody
}  // namespace dairlib
