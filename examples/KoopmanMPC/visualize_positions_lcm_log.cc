#include <gflags/gflags.h>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "multibody/multipose_visualizer.h"
#include "multibody/visualization_utils.h"
#include "drake/lcm/drake_lcm.h"

using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

using drake::MatrixX;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::cout;
using std::endl;
using std::shared_ptr;
using std::string;
using std::to_string;
using std::vector;
using clk = std::chrono::high_resolution_clock;

using dairlib::multibody::MultiposeVisualizer;

namespace dairlib {

DEFINE_double(alpha, 0.1, "Transparency of the robots");
DEFINE_string(poses_file_name, "examples/KoopmanMPC/saved_runs/walking_visualization.csv", "filename for saved trajectory");

void visualizeFullOrderModelPose(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  MatrixXd poses = readCSV(FLAGS_poses_file_name);

  // Create MultiposeVisualizer
  VectorXd alpha_vec = FLAGS_alpha * VectorXd::Ones(poses.cols());
  alpha_vec.head(1) << 1;
  alpha_vec.tail(1) << 0.2;
  MultiposeVisualizer visualizer = MultiposeVisualizer(
      FindResourceOrThrow("examples/PlanarWalker/PlanarWalkerWithTorso.urdf"),
      poses.cols(), alpha_vec, "base");

  visualizer.DrawPoses(poses);
}
}// namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::visualizeFullOrderModelPose(argc, argv);
  return 0;
}