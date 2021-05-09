#include <chrono>
#include <memory>
#include <string>
#include <thread>
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

using dairlib::multibody::MultiposeVisualizer;

namespace dairlib {
namespace goldilocks_models {
namespace planning {

DEFINE_int32(robot_option, 1, "0: plannar robot. 1: cassie_fixed_spring");

DEFINE_bool(global, false, "");
DEFINE_bool(snopt_suffix, false, "");

// Flags for solve indices
DEFINE_int32(solve_idx, -1, "");
DEFINE_int32(solve_idx_end, -1,
             "If this is set, we will visualize the poses"
             " from `solve_idx` to `solve_idx_end`");
DEFINE_double(solve_idx_iterate_time, 0.1, "in seconds");

// There are two modes of this visualizer.
// The first mode visualizes all poses at once, and the second mode visualizes
// them in sequence
DEFINE_bool(view_multipose_at_once, false, "Visualize the poses at once");

// Option 1 -- multiple pose at once
DEFINE_bool(only_start_and_end_poses, false,
            "Visualize only the start and the end poses");
DEFINE_double(alpha, 0.1, "Transparency of the robots");

// Option 2 -- one pose (mode) at a time
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

  if (FLAGS_solve_idx_end >= 0) {
    DRAKE_DEMAND(FLAGS_solve_idx != -1);
    DRAKE_DEMAND(FLAGS_view_multipose_at_once);
  }

  // Setup solve indices
  int solve_idx_start = FLAGS_solve_idx;
  int solve_idx_end =
      (FLAGS_solve_idx_end >= 0) ? FLAGS_solve_idx_end : FLAGS_solve_idx;

  if (solve_idx_start < 0) {
    cout << "Drawing solve_idx " << solve_idx_start << " to " << solve_idx_end
         << endl;
  } else {
    cout << "Drawing debug_ files\n";
  }

  // Read in poses
  string suffix = FLAGS_snopt_suffix ? "_snopt" : "";
  string global = FLAGS_global ? "_global_" : "_local_";
  std::vector<MatrixXd> x0_each_mode_list;
  std::vector<MatrixXd> xf_each_mode_list;
  for (int i = solve_idx_start; i <= solve_idx_end; i++) {
    string x0_path =
        (i >= 0)
            ? directory + to_string(i) + global + "x0_FOM" + suffix + ".csv"
            : directory + "debug" + global + "x0_FOM" + suffix + ".csv";
    string xf_path =
        (i >= 0)
            ? directory + to_string(i) + global + "xf_FOM" + suffix + ".csv"
            : directory + "debug" + global + "xf_FOM" + suffix + ".csv";
    x0_each_mode_list.push_back(readCSV(x0_path));
    xf_each_mode_list.push_back(readCSV(xf_path));
  }  // for loop solve_idx
  DRAKE_DEMAND(x0_each_mode_list.size() == xf_each_mode_list.size());

  // Option 1 -- multiple pose at once
  if (FLAGS_view_multipose_at_once) {
    // Select poses for visualization
    MatrixXd x0_each_mode = x0_each_mode_list.at(0);
    MatrixXd xf_each_mode = xf_each_mode_list.at(0);
    std::vector<MatrixXd> poses(
        x0_each_mode_list.size(),
        FLAGS_only_start_and_end_poses
            ? MatrixXd::Zero(x0_each_mode.rows(), 2)
            : MatrixXd::Zero(x0_each_mode.rows(), x0_each_mode.cols() + 1));
    for (int i = 0; i <= solve_idx_end - solve_idx_start; i++) {
      if (FLAGS_only_start_and_end_poses) {
        poses[i] << x0_each_mode_list.at(i).leftCols<1>(),
            xf_each_mode_list.at(i).rightCols<1>();
      } else {
        poses[i] << x0_each_mode_list.at(i),
            xf_each_mode_list.at(i).rightCols<1>();
      }
    }
    // Create MultiposeVisualizer
    VectorXd alpha_vec = FLAGS_alpha * VectorXd::Ones(poses.at(0).cols());
    alpha_vec.head(1) << 1;
    alpha_vec.tail(1) << 0.2;
    MultiposeVisualizer visualizer = MultiposeVisualizer(
        FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"),
        poses.at(0).cols(), alpha_vec);

    // Draw
    for (int i = 0; i <= solve_idx_end - solve_idx_start; i++) {
      visualizer.DrawPoses(poses.at(i));
      if (i == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      // pause for `solve_idx_iterate_time` seconds
      std::this_thread::sleep_for(
          std::chrono::milliseconds(int(FLAGS_solve_idx_iterate_time * 1000)));
    }
  }
  // Option 2 -- one pose (mode) at a time
  else {
    DRAKE_DEMAND(x0_each_mode_list.size() == 1);
    DRAKE_DEMAND(xf_each_mode_list.size() == 1);
    MatrixXd x0_each_mode = x0_each_mode_list.at(0);
    MatrixXd xf_each_mode = xf_each_mode_list.at(0);

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
      if (is_head) mode--;

      is_head = !is_head;
    }  // for loop mode
  }    // if-else view_multipose_at_once
}
}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::goldilocks_models::planning::visualizeFullOrderModelPose(argc, argv);
  return 0;
}
