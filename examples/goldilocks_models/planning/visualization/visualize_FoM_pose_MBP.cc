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
#include "drake/geometry/geometry_visualization.h"
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

DEFINE_int32(robot_option, 0, "0: plannar robot. 1: cassie_fixed_spring");

// File source
DEFINE_int32(solve_idx, -1, "");
DEFINE_bool(snopt_suffix, false, "");
DEFINE_bool(global, false, "");

// There are two modes of this visualizer.
// The first mode visualizes all poses at once, and the second mode visualizes
// them in sequence
DEFINE_bool(view_multipose_at_once, false, "Visualize the poses at once");

// Flags for the first mode
DEFINE_bool(only_start_and_end_poses, false,
            "Visualize only the start and the end poses");
DEFINE_double(alpha, 0.15, "Transparency of the robots");

// Flags for the second mode
DEFINE_int32(start_mode, 0, "Starting at mode #");
DEFINE_bool(start_is_head, true, "Starting with x0 or xf");
DEFINE_int32(end_mode, -1, "Ending at mode #");
DEFINE_bool(end_is_head, false, "Ending with x0 or xf");
DEFINE_double(step_time, 1, "Duration per step * 2");

void visualizeFullOrderModelPose(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // parameters
  const string directory = "../dairlib_data/goldilocks_models/planning/robot_" +
                           to_string(FLAGS_robot_option) + "/data/";

  // Read in number of steps
  int end_mode = (FLAGS_end_mode >= 0)
                     ? FLAGS_end_mode
                     : readCSV(directory + string("n_step.csv"))(0, 0) - 1;

  // Read in pose
  MatrixXd x0_each_mode;
  MatrixXd xf_each_mode;
  string suffix = FLAGS_snopt_suffix ? "_snopt" : "";
  string global = FLAGS_global ? "_global_" : "_local_";
  if (FLAGS_solve_idx >= 0) {
    cout << "Drawing solve_idx " << FLAGS_solve_idx << endl;
    x0_each_mode =
        readCSV(directory + string(to_string(FLAGS_solve_idx) + global +
                                   "x0_FOM" + suffix + ".csv"));
    xf_each_mode =
        readCSV(directory + string(to_string(FLAGS_solve_idx) + global +
                                   "xf_FOM" + suffix + ".csv"));
  } else {
    cout << "Drawing debug_ files\n";
    x0_each_mode = readCSV(
        directory + string("debug" + global + "x0_FOM" + suffix + ".csv"));
    xf_each_mode = readCSV(
        directory + string("debug" + global + "xf_FOM" + suffix + ".csv"));
  }

  if (FLAGS_view_multipose_at_once) {
    MatrixXd poses;
    if (FLAGS_only_start_and_end_poses) {
      poses.resize(x0_each_mode.rows(), 2);
      poses << x0_each_mode.leftCols<1>(), xf_each_mode.rightCols<1>();
    } else {
      poses.resize(x0_each_mode.rows(), x0_each_mode.cols() + 1);
      poses << x0_each_mode, xf_each_mode.rightCols<1>();
    }
    VectorXd alpha_vec = FLAGS_alpha * VectorXd::Ones(poses.cols());
    alpha_vec.head(1) << 1;
    alpha_vec.tail(1) << 1;
    multibody::MultiposeVisualizer visualizer = multibody::MultiposeVisualizer(
        FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"),
        poses.cols(), alpha_vec);
    visualizer.DrawPoses(poses);
  } else {
    bool is_head = FLAGS_start_is_head;
    bool last_visited_mode = -1;
    for (int mode = FLAGS_start_mode; mode <= end_mode; mode++) {
      cout << "(mode, is_head) = (" << mode << ", " << is_head << ")\n";
      if (last_visited_mode != mode) {
        last_visited_mode = mode;
      }

      // Create a testing piecewise polynomial
      std::vector<double> T_breakpoint{0, FLAGS_step_time};
      std::vector<MatrixXd> Y;
      if (is_head) {
        Y.push_back(x0_each_mode.col(mode));
        Y.push_back(x0_each_mode.col(mode));
        cout << "x0 = \n" << x0_each_mode.col(mode).transpose() << endl;
      } else {
        Y.push_back(xf_each_mode.col(mode));
        Y.push_back(xf_each_mode.col(mode));
        cout << "xf = \n" << xf_each_mode.col(mode).transpose() << endl;
      }
      PiecewisePolynomial<double> pp_xtraj =
          PiecewisePolynomial<double>::FirstOrderHold(T_breakpoint, Y);

      // Create MBP for visualization
      drake::systems::DiagramBuilder<double> builder;
      MultibodyPlant<double> plant_vis(0.0);
      SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
      Vector3d ground_normal(0, 0, 1);
      CreateMBPForVisualization(&plant_vis, &scene_graph, ground_normal,
                                FLAGS_robot_option);

      // visualizer
      multibody::connectTrajectoryVisualizer(&plant_vis, &builder, &scene_graph,
                                             pp_xtraj);
      auto diagram = builder.Build();
      // while (true)
      drake::systems::Simulator<double> simulator(*diagram);
      simulator.set_target_realtime_rate(1);
      simulator.Initialize();
      simulator.AdvanceTo(pp_xtraj.end_time());

      if ((is_head == FLAGS_end_is_head) && (mode == end_mode)) continue;
      if (is_head == true) mode--;

      is_head = !is_head;
    }
  }

  return;
}
}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::goldilocks_models::planning::visualizeFullOrderModelPose(argc, argv);
  return 0;
}
