#pragma once

#include <string>
#include <vector>

#include "drake/geometry/meshcat_visualizer.h"
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
  /// @param world_frame_offset An optional offset to apply when welding to
  /// world
  MultiposeVisualizer(std::string model_file, int num_poses,
                      std::string weld_frame_to_world = "",
                      const drake::math::RigidTransformd& world_frame_offset =
                          drake::math::RigidTransformd());

  /// Constructor, scales all alpha transparencies by a constant
  /// @param model_file A full path to model (e.g. through FindResourceOrThrow)
  /// @param num_poses The number of simultaneous poses to draw (fixed)
  /// @param alpha_scale Scales the transparency alpha field of all bodies
  /// @param weld_frame_to_world Welds the frame of the given name to the world
  /// @param world_frame_offset An optional offset to apply when welding to
  /// world
  MultiposeVisualizer(std::string model_file, int num_poses, double alpha_scale,
                      std::string weld_frame_to_world = "",
                      const drake::math::RigidTransformd& world_frame_offset =
                          drake::math::RigidTransformd());

  /// Constructor, scales all alpha transparencies by a constant
  /// @param model_file A full path to model (e.g. through FindResourceOrThrow)
  /// @param num_poses The number of simultaneous poses to draw (fixed)
  /// @param alpha_scale Vector, of same length as num_poses. Provides variable
  /// scaling of the transparency alpha field of all bodies, indexed by pose
  /// @param weld_frame_to_world Welds the frame of the given name to the world
  /// @param world_frame_offset An optional offset to apply when welding to
  /// world
  /// @param meshcat Pointer to meshcat visualizer for option to attach to an
  /// existing meshcat instance
  /// @param pose_trace_name Name of the pose trace to use in meshcat
  /// @param rgb the RGB color to use for all bodies.  If not provided, the
  /// color will default to what is defined in the model file.
  /// @param meshcat_path_prefix The meshcat tree path under which this
  /// visualizer's geometry is nested (replaces the internal
  /// MeshcatVisualizer's default "visualizer" prefix). If empty, falls back
  /// to the default "visualizer" prefix. Set this to match the path used by
  /// any other geometry (e.g. lines drawn directly via Meshcat::SetObject)
  /// that should appear alongside these poses in the meshcat tree.
  MultiposeVisualizer(
      std::string model_file, int num_poses, const Eigen::VectorXd& alpha_scale,
      std::string weld_frame_to_world = "",
      const drake::math::RigidTransformd& world_frame_offset =
          drake::math::RigidTransformd(),
      std::shared_ptr<drake::geometry::Meshcat> meshcat = nullptr,
      const std::string& pose_trace_name = "",
      const Eigen::VectorXd& rgb = Eigen::VectorXd(),
      const std::string& meshcat_path_prefix = "");

  /// Draws the poses in the given (num_positions x num_poses) matrix
  /// Note: the matrix can have extra rows (e.g. velocities), which will be
  /// ignored.
  void DrawPoses(Eigen::MatrixXd poses,
                 std::optional<double> time_in_recording = std::nullopt);

  const std::shared_ptr<drake::geometry::Meshcat> GetMeshcat() {
    return meshcat_;
  }

  const int GetNumConfig() { return num_config_; }
  const int GetNumVel() { return num_vel_; }
  const drake::geometry::Rgba GetColor() { return color_; }

 private:
  int num_poses_;
  int num_config_;
  int num_vel_;
  drake::multibody::MultibodyPlant<double>* plant_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  drake::geometry::MeshcatVisualizer<double>* meshcat_visualizer_;
  std::unique_ptr<drake::systems::Context<double>> diagram_context_;
  std::vector<drake::multibody::ModelInstanceIndex> model_indices_;
  drake::geometry::Rgba color_;
};

}  // namespace multibody
}  // namespace dairlib
