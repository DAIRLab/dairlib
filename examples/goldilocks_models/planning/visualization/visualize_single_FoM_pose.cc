#include <chrono>
#include <memory>
#include <string>
#include <gflags/gflags.h>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/multipose_visualizer.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

using drake::MatrixX;
using drake::trajectories::PiecewisePolynomial;
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

namespace dairlib {
namespace goldilocks_models {
namespace planning {

DEFINE_int32(robot_option, 1, "0: plannar robot. 1: cassie_fixed_spring");
DEFINE_string(file_name, "x_init_test.csv", "");

void visualizeFullOrderModelPose(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cout << "******\n";
  cout << "****** Reminder: turn of other visualizer before you run this.\n";
  cout << "******\n";

  // parameters
  const string directory = "../dairlib_data/goldilocks_models/planning/robot_" +
      to_string(FLAGS_robot_option) + "/data/";

  // Read in pose
  MatrixXd x_to_plot = readCSV(directory + FLAGS_file_name);
  cout << "x_to_plot= \n" << x_to_plot << endl;

  multibody::MultiposeVisualizer visualizer = multibody::MultiposeVisualizer(
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"),
      x_to_plot.cols());
  visualizer.DrawPoses(x_to_plot);

}
}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::goldilocks_models::planning::visualizeFullOrderModelPose(argc, argv);
  return 0;
}
