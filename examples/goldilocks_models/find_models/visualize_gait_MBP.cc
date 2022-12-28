#include <chrono>
#include <string>
#include <gflags/gflags.h>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/task.h"
#include "lcm/dircon_saved_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/system_utils.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

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

// Reminder: Drake visualizer should be running in parallel with this script

// If iter_end > iter_start, the program visualizes gaits for
// (iteration index, sample index) = (iter_start, sample),
//                                   (iter_start + 1, sample),
//                                   (iter_start + 2, sample),
//                                             ...
//                                   (iter_end - 1, sample),
//                                   (iter_end, sample).
// By default, iter_end = -1, and we only visualize
// (iteration index, sample index) = (iter_start, sample).
DEFINE_int32(iter_start, 1, "The iter idx");
DEFINE_int32(iter_end, -1, "The iter idx");
DEFINE_int32(sample, 0, "The sample idx");

DEFINE_double(realtime_factor, 1, "Rate of which the traj is played back");
DEFINE_int32(n_step, 1,
             "# of foot steps. "
             "(multi-steps only works when the gait is periodic)");

DEFINE_int32(robot_option, 1, "0: plannar robot. 1: cassie_fixed_spring");

DEFINE_double(pause_duration, 0.5,
              "pause duration in the beginning and the end of the gait");
DEFINE_bool(construct_cubic, false,
            "By default, we construct trajectories using first order hold."
            "Set this flag to true if you want to construct cubic spline. Old "
            "files (before 2019.12.31) didn't store derivatives information, "
            "so this option cannot be used on those files.");

DEFINE_string(path, "", "");

DEFINE_bool(draw_ground, false, "");

void swapTwoBlocks(MatrixXd* mat, int i_1, int j_1, int i_2, int j_2, int n_row,
                   int n_col) {
  MatrixXd temp_block1 = mat->block(i_1, j_1, n_row, n_col);
  MatrixXd temp_block2 = mat->block(i_2, j_2, n_row, n_col);
  mat->block(i_1, j_1, n_row, n_col) = temp_block2;
  mat->block(i_2, j_2, n_row, n_col) = temp_block1;
}

void visualizeGait(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // User settings
  /*string directory =
     "examples/goldilocks_models/find_models/data/robot_"
                           + to_string(FLAGS_robot_option) + "/";*/
  string directory = "../dairlib_data/goldilocks_models/find_models/robot_" +
                     to_string(FLAGS_robot_option) + "/";
  //  string directory =
  //  "../dairlib_data/goldilocks_models/find_boundary/robot_" +
  //                     to_string(FLAGS_robot_option) + "/";
  //  string directory =
  //      "../dairlib_data/goldilocks_models/find_boundary_sl_gi_not_optimized/"
  //      "robot_" +
  //      to_string(FLAGS_robot_option) + "/";

  if (FLAGS_path.length() != 0) {
    directory = FLAGS_path + "/";
  }
  cout << "directory = " << directory << endl;

  // Other settings
  int iter_start = FLAGS_iter_start;
  int iter_end =
      (FLAGS_iter_end >= FLAGS_iter_start) ? FLAGS_iter_end : FLAGS_iter_start;
  int n_step = FLAGS_n_step;  // Should be > 0

  // Construct a list of (iteration #, sample #) pairs to visualize for
  std::vector<std::pair<int, int>> iter_sample_pair_list;
  for (int iter = iter_start; iter <= iter_end; iter++) {
    iter_sample_pair_list.emplace_back(iter, FLAGS_sample);
  }

  // Loop through each (iteration #, sample #) pair and visualize the gait
  for (const auto& iter_sample_pair : iter_sample_pair_list) {
    int iter = iter_sample_pair.first;
    int sample = iter_sample_pair.second;

    // Read in trajectory
    dairlib::DirconTrajectory dircon_traj(directory + to_string(iter) +
                                          string("_") + to_string(sample) +
                                          string("_dircon_trajectory"));

    VectorXd time_mat;
    MatrixXd state_mat;
    MatrixXd statedot_mat;
    if (!FLAGS_construct_cubic) {
      time_mat = dircon_traj.GetBreaks();
      state_mat = dircon_traj.GetStateSamples(0);
      for (int i = 1; i < dircon_traj.GetNumModes(); i++) {
        int n_new_cols = dircon_traj.GetStateBreaks(i).size() - 1;
        state_mat.conservativeResize(state_mat.rows(),
                                     state_mat.cols() + n_new_cols);
        state_mat.rightCols(n_new_cols) =
            dircon_traj.GetStateSamples(i).rightCols(n_new_cols);
      }
    } else {
      time_mat = dircon_traj.GetBreaks();
      state_mat = dircon_traj.GetStateSamples(0);
      statedot_mat = dircon_traj.GetStateDerivativeSamples(0);
      for (int i = 1; i < dircon_traj.GetNumModes(); i++) {
        int n_new_cols = dircon_traj.GetStateBreaks(i).size() - 1;
        state_mat.conservativeResize(state_mat.rows(),
                                     state_mat.cols() + n_new_cols);
        statedot_mat.conservativeResize(statedot_mat.rows(),
                                        statedot_mat.cols() + n_new_cols);
        state_mat.rightCols(n_new_cols) =
            dircon_traj.GetStateSamples(i).rightCols(n_new_cols);
        statedot_mat.rightCols(n_new_cols) =
            dircon_traj.GetStateDerivativeSamples(i).rightCols(n_new_cols);
      }
    }

    int n_state = state_mat.rows();
    int n_q;
    if (FLAGS_robot_option == 0) {
      n_q = n_state / 2;
    } else if (FLAGS_robot_option == 1) {
      n_q = 19;
    } else {
      n_q = -1;
      DRAKE_DEMAND(false);  // Shouldn't come here
    }
    int n_node = time_mat.rows();
    VectorXd ones = VectorXd::Ones(n_node - 1);
    int translation_size;
    int translation_start_idx;
    if (FLAGS_robot_option == 0) {
      translation_size = 2;
      translation_start_idx = 0;
    } else if (FLAGS_robot_option == 1) {
      translation_size = 3;
      translation_start_idx = 4;
    } else {
      translation_size = -1;
      translation_start_idx = -1;
      DRAKE_DEMAND(false);  // Shouldn't come here
    }
    VectorXd xyz_translation =
        state_mat.block(translation_start_idx, n_node - 1, translation_size,
                        1) -
        state_mat.block(translation_start_idx, 0, translation_size, 1);

    // Concatenate the traj so it has multiple steps
    // 1. time
    VectorXd time_mat_cat(n_step * n_node - (n_step - 1));
    time_mat_cat(0) = 0;
    for (int i = 0; i < n_step; i++) {
      time_mat_cat.segment(1 + (n_node - 1) * i, n_node - 1) =
          time_mat.tail(n_node - 1) + time_mat_cat((n_node - 1) * i) * ones;
    }
    // 2. state (and its derivatives)
    std::vector<MatrixXd> mat_cat;  // first element is state (and second
                                    // element is its derivatives)
    mat_cat.push_back(MatrixXd(n_state, n_step * n_node - (n_step - 1)));
    if (FLAGS_construct_cubic) {
      mat_cat.push_back(MatrixXd(n_state, n_step * n_node - (n_step - 1)));
    }
    mat_cat[0].col(0) = state_mat.col(0);
    if (FLAGS_construct_cubic) {
      mat_cat[1].col(0) = statedot_mat.col(0);
    }
    for (int i = 0; i < n_step; i++) {
      // Copy over the data at all knots but the first one
      mat_cat[0].block(0, 1 + (n_node - 1) * i, n_state, n_node - 1) =
          state_mat.block(0, 1, n_state, n_node - 1);
      if (FLAGS_construct_cubic) {
        mat_cat[1].block(0, 1 + (n_node - 1) * i, n_state, n_node - 1) =
            statedot_mat.block(0, 1, n_state, n_node - 1);
      }

      // Translate x and z (only for position not its derivatives)
      if (FLAGS_robot_option == 0) {
        for (int j = 0; j < translation_size; j++) {
          mat_cat[0].block(j, 1 + (n_node - 1) * i, 1, n_node - 1) =
              state_mat.block(j, 1, 1, n_node - 1) +
              i * xyz_translation(j) * ones.transpose();
        }
      } else if (FLAGS_robot_option == 1) {
        if (i > 0) {
          for (int j = 0; j < translation_size; j++) {
            if (j == 1) {
              // It's mirror in x-z plane, so we don't need to translate in y.
            } else {
              mat_cat[0].block(j + translation_start_idx, 1 + (n_node - 1) * i,
                               1, n_node - 1) =
                  state_mat.block(j + translation_start_idx, 1, 1, n_node - 1) +
                  i * xyz_translation(j) * ones.transpose();
            }
          }
        }
      }

      // Flip the sign for the even number of stance phase
      if (i % 2) {
        if (FLAGS_robot_option == 1) {
          for (auto& mat_cat_member : mat_cat) {
            // Quaternion sign should also be flipped.
            mat_cat_member.block(1, 1 + (n_node - 1) * i, 1, n_node - 1) *= -1;
            mat_cat_member.block(3, 1 + (n_node - 1) * i, 1, n_node - 1) *= -1;
            // y should be flipped
            mat_cat_member.block(5, 1 + (n_node - 1) * i, 1, n_node - 1) *= -1;
            // Hip roll and yaw sign should also be flipped.
            mat_cat_member.block(7, 1 + (n_node - 1) * i, 4, n_node - 1) *= -1;

            // We do not flip the sign for the velocity part of state because
            // it's not used in visualization
          }
        }
      }

      // Swap the leg
      if (i % 2) {
        for (auto& mat_cat_member : mat_cat) {
          if (FLAGS_robot_option == 0) {
            // Position
            swapTwoBlocks(&mat_cat_member, 3, 1 + (n_node - 1) * i, 4,
                          1 + (n_node - 1) * i, 1, n_node - 1);
            swapTwoBlocks(&mat_cat_member, 5, 1 + (n_node - 1) * i, 6,
                          1 + (n_node - 1) * i, 1, n_node - 1);

            // We do not swap the velocity part because it's not used in
            // visualization
          } else if (FLAGS_robot_option == 1) {
            // Position
            swapTwoBlocks(&mat_cat_member, 7, 1 + (n_node - 1) * i, 8,
                          1 + (n_node - 1) * i, 1, n_node - 1);
            swapTwoBlocks(&mat_cat_member, 9, 1 + (n_node - 1) * i, 10,
                          1 + (n_node - 1) * i, 1, n_node - 1);
            swapTwoBlocks(&mat_cat_member, 11, 1 + (n_node - 1) * i, 12,
                          1 + (n_node - 1) * i, 1, n_node - 1);
            swapTwoBlocks(&mat_cat_member, 13, 1 + (n_node - 1) * i, 14,
                          1 + (n_node - 1) * i, 1, n_node - 1);
            swapTwoBlocks(&mat_cat_member, 15, 1 + (n_node - 1) * i, 16,
                          1 + (n_node - 1) * i, 1, n_node - 1);
            swapTwoBlocks(&mat_cat_member, 17, 1 + (n_node - 1) * i, 18,
                          1 + (n_node - 1) * i, 1, n_node - 1);

            // We do not swap the velocity part because it's not used in
            // visualization
          }
        }
      }  // end if (i % 2)
    }    // end for n_step

    // Create a testing piecewise polynomial
    std::vector<double> T_breakpoint;
    for (int i = 0; i < time_mat_cat.size(); i++) {
      T_breakpoint.push_back(time_mat_cat(i) + FLAGS_pause_duration);
    }
    std::vector<MatrixXd> Y;
    for (int i = 0; i < time_mat_cat.size(); i++) {
      Y.push_back(mat_cat[0].col(i));
    }
    std::vector<MatrixXd> Y_dot;
    if (FLAGS_construct_cubic) {
      for (int i = 0; i < time_mat_cat.size(); i++)
        Y_dot.push_back(mat_cat[1].col(i));
    }
    PiecewisePolynomial<double> pp_xtraj =
        (FLAGS_construct_cubic)
            ? PiecewisePolynomial<double>::CubicHermite(T_breakpoint, Y, Y_dot)
            : PiecewisePolynomial<double>::FirstOrderHold(T_breakpoint, Y);

    // Create MBP
    drake::logging::set_log_level("err");  // ignore warnings about joint limits
    drake::systems::DiagramBuilder<double> builder;
    MultibodyPlant<double> plant(0.0);
    SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
    if (FLAGS_draw_ground) {
      // Read in task name
      vector<string> task_name =
          ParseCsvToStringVec(directory + "task_names.csv");
      Task task(task_name);
      int ground_incline_idx = task.name_to_index_map().at("ground_incline");

      // Read in ground incline
      double ground_incline = readCSV(
          directory + to_string(iter) + string("_") + to_string(sample) +
          string("_task.csv"))(ground_incline_idx, 0);

      // Add ground
      Vector3d ground_normal(sin(ground_incline), 0, cos(ground_incline));
      multibody::addFlatTerrain(&plant, &scene_graph, 0.8, 0.8, ground_normal);
    }
    Parser parser(&plant, &scene_graph);
    std::string full_name;
    if (FLAGS_robot_option == 0) {
      full_name = FindResourceOrThrow(
          "examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
    } else if (FLAGS_robot_option == 1) {
      full_name =
          FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
    }
    parser.AddModelFromFile(full_name);
    if (FLAGS_robot_option == 0) {
      plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                       drake::math::RigidTransform<double>());
    }
    plant.Finalize();

    // visualizer
    int n_loops = 1;
    auto ball_plant = multibody::ConstructBallPlant(&scene_graph);
    multibody::connectTrajectoryVisualizer(&plant, &builder, &scene_graph,
                                           pp_xtraj, *ball_plant);
    auto diagram = builder.Build();
    diagram->set_name("traj_visualizer");
    DrawAndSaveDiagramGraph(*diagram);

    // while (true)
    for (int i = 0; i < n_loops; i++) {
      drake::systems::Simulator<double> simulator(*diagram);
      simulator.set_target_realtime_rate(FLAGS_realtime_factor);
      simulator.Initialize();
      simulator.AdvanceTo(pp_xtraj.end_time() + FLAGS_pause_duration);
    }
  }  // end for(int iter...)
}
}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::goldilocks_models::visualizeGait(argc, argv);

  return 0;
}
