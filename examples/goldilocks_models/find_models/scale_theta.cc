#include <gflags/gflags.h>
#include <Eigen/Dense>
#include <string>

#include "systems/goldilocks_models/file_utils.h"
#include "drake/common/drake_assert.h"

#include "examples/goldilocks_models/kinematics_expression.h"
#include "examples/goldilocks_models/dynamics_expression.h"

#include "common/find_resource.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/multibody/parsing/parser.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::string;
using std::to_string;
using std::cin;
using std::cout;
using std::endl;

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Parser;
using dairlib::FindResourceOrThrow;

namespace dairlib {
namespace goldilocks_models {

DEFINE_int32(robot_option, 0, "0: plannar robot. 1: cassie_fixed_spring");
DEFINE_int32(iter_start, 0, "Which iteration");
DEFINE_int32(iter_end, 0, "Which iteration");
DEFINE_int32(n_s, 4, "dimension of the RoM");
DEFINE_double(scaling_factor, -1, "Scaling factor");

DEFINE_bool(is_active_model, true, "Model has input?");
DEFINE_int32(n_tau, 2, "dimension of the input of the RoM");
DEFINE_int32(num_traj_opt_knots, 20, "# of traj opt knot points");
DEFINE_int32(num_batch, 1, "total number of batch");

inline bool file_exist (const std::string & name) {
  struct stat buffer;
  // cout << name << " exist? " << (stat (name.c_str(), &buffer) == 0) << endl;
  return (stat (name.c_str(), &buffer) == 0);
}

// Terminal command samples:
//  ./bazel-bin/examples/goldilocks_models/scale_theta --iter=211 --num_linear_term=4 --num_quadratic_term=10 --scaling_factor=0.2 --n_s=2 --num_traj_opt_knots=20 --n_tau=1
//  ./bazel-bin/examples/goldilocks_models/scale_theta --iter=212 --num_linear_term=4 --num_quadratic_term=10 --scaling_factor=0.2 --n_s=2 --is_active_model=false

// Assumptions:
// 1. The highest order is quadratic
// 2. The model is control-affine.
//    Also, one input element go to one position element! (This is required
//    because of the way we scale it.)
// 3. The input are at the end of the decision variables. (You can improve this
//    by comparing the values in csv.)
int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const string directory = "examples/goldilocks_models/find_models/data/";
  string prefix = directory;

  cout << "iter_start = " << FLAGS_iter_start << endl;
  cout << "iter_end = " << FLAGS_iter_end << endl;
  cout << "scaling_factor = " << FLAGS_scaling_factor << endl;

  cout << "n_s = " << FLAGS_n_s << endl;

  MatrixXd B_tau(0, 0);
  if (FLAGS_is_active_model) {
    cout << "The model has input.\n";
    cout << "n_tau = " << FLAGS_n_tau << endl;
    cout << "num_traj_opt_knots = " << FLAGS_num_traj_opt_knots << endl;
    cout << "num_batch = " << FLAGS_num_batch << endl;

    // Read in B_tau
    B_tau.resize(FLAGS_n_s, FLAGS_n_tau);
    if (file_exist(prefix + "B_tau.csv")) {
      MatrixXd B_tau_temp = readCSV(prefix + string("B_tau.csv"));
      DRAKE_DEMAND(B_tau_temp.rows() == FLAGS_n_s);
      DRAKE_DEMAND(B_tau_temp.cols() == FLAGS_n_tau);
      B_tau = B_tau_temp;
    } else {
      // Testing
      B_tau << 0, 0, 0, 0, 1, 0, 0, 1;
      cout << "You are manually assign B_tau. Make sure that it's correct.\n";
    }
    cout << "B_tau = \n" << B_tau << endl;
  } else {
    cout << "The model doesn't have input.\n";
  }

  cout << "We are currently using 1-norm for scaling.\n";

  cout << "Are the above numbers and settings correct? (Y/N)\n";
  char answer[1];
  cin >> answer;
  if (!((answer[0] == 'Y') || (answer[0] == 'y'))) {
    cout << "Ending the program, since the numbers are incorrect.\n";
    return 0;
  } else {
    cout << "Updating...\n";
  }

  // Read in the parameters for setup
  prefix = directory + to_string(FLAGS_iter_start);
  VectorXd theta_s = readCSV(prefix + string("_theta_s.csv")).col(0);
  VectorXd theta_sDDot = readCSV(prefix + string("_theta_sDDot.csv")).col(0);

  // Create MBP for setup
  MultibodyPlant<double> plant;
  Parser parser(&plant);
  std::string full_name = FindResourceOrThrow(
                            "examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
  parser.AddModelFromFile(full_name);
  plant.AddForceElement<drake::multibody::UniformGravityFieldElement>(
    -9.81 * Eigen::Vector3d::UnitZ());
  plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("base"),
    drake::math::RigidTransform<double>());
  plant.Finalize();

  // Setup
  KinematicsExpression<double> kin_expression(FLAGS_n_s, 0, &plant, FLAGS_robot_option);
  DynamicsExpression dyn_expression(FLAGS_n_s, 0, FLAGS_robot_option);
  VectorXd dummy_q = VectorXd::Ones(plant.num_positions());
  VectorXd dummy_s = VectorXd::Ones(FLAGS_n_s);
  int n_feature_s = kin_expression.getFeature(dummy_q).size();
  int n_feature_sDDot = dyn_expression.getFeature(dummy_s, dummy_s).size();
  DRAKE_DEMAND(theta_s.rows() == FLAGS_n_s * n_feature_s);
  DRAKE_DEMAND(theta_sDDot.rows() == FLAGS_n_s * n_feature_sDDot);

  // Create a list of idx corresponding to const, linear, quadratic terms...
  std::vector<int> const_term_list;
  std::vector<int> linear_term_list;
  std::vector<int> quadratic_term_list;
  double k_test_scale = 1.3;
  VectorXd scaled_dummy_s = k_test_scale * dummy_s;
  VectorXd original_feat = dyn_expression.getFeature(dummy_s, dummy_s);
  VectorXd scaled_feat = dyn_expression.getFeature(
                           scaled_dummy_s, scaled_dummy_s);
  for (int i = 0; i < n_feature_sDDot; i++) {
    if (scaled_feat(i) == original_feat(i)) {
      const_term_list.push_back(i);
    } else if (scaled_feat(i) == k_test_scale * original_feat(i)) {
      linear_term_list.push_back(i);
    } else if (scaled_feat(i) == pow(k_test_scale, 2) * original_feat(i)) {
      quadratic_term_list.push_back(i);
    } else {
      cout << "No matching scaling for index #" << i << endl;
      return 0;
    }
  }
  cout << "\nConstant terms: ";
  for (int i : const_term_list)
    cout << i << ", ";
  cout << "\nLinear terms: ";
  for (int i : linear_term_list)
    cout << i << ", ";
  cout << "\nQuadratic terms: ";
  for (int i : quadratic_term_list)
    cout << i << ", ";
  cout << endl;

  for (int iter = FLAGS_iter_start; iter <= FLAGS_iter_end; iter++) {
    cout << "iter = " << iter << endl;

    // Read in the parameters
    prefix = directory + to_string(iter);
    VectorXd theta_s = readCSV(prefix + string("_theta_s.csv")).col(0);
    VectorXd theta_sDDot = readCSV(prefix + string("_theta_sDDot.csv")).col(0);

    // Get the scaling factor for each element of s
    VectorXd scaling_factors(FLAGS_n_s);
    for (int s_row = 0; s_row < FLAGS_n_s; s_row++) {
      // Get the scaling factor automatically unless specified by the user
      if (FLAGS_scaling_factor != -1) {
        scaling_factors(s_row) = FLAGS_scaling_factor;
      } else {
        // Get scaling factor
        double theta_sum = 0;
        for (int j = 0; j < n_feature_s; j++) {
          theta_sum += abs(theta_s(j + s_row * n_feature_s)); // 1-norm
        }
        scaling_factors(s_row) = 1 / theta_sum;
      }
    }
    cout << "scaling_factors = " << scaling_factors.transpose() << endl;

    // Parameters part
    for (int s_row = 0; s_row < FLAGS_n_s; s_row++) {
      // iterate through each element of s

      // Scaling of kinematics parameters
      theta_s.segment(s_row * n_feature_s, n_feature_s) *= scaling_factors(s_row);

      // Scaling of dynamics parameters
      // constant term
      for (int i : const_term_list)
        theta_sDDot.segment(i + s_row * n_feature_sDDot, 1) *= scaling_factors(s_row);
      // linear terms
      for (int i : linear_term_list)
        theta_sDDot.segment(i + s_row * n_feature_sDDot, 1) *= 1;
      // quadratic terms
      for (int i : quadratic_term_list)
        theta_sDDot.segment(i + s_row * n_feature_sDDot, 1) /= scaling_factors(s_row);

      // Store (overwrite) the parameters
      prefix = directory + to_string(iter);
      writeCSV(prefix + string("_theta_s_new.csv"), theta_s);
      writeCSV(prefix + string("_theta_sDDot_new.csv"), theta_sDDot);
    }  // end for (each row of s)

    // Update t_and_s, t_and_ds, t_and_dds
    for (int batch = 0; batch < FLAGS_num_batch; batch++) {
      prefix = directory + to_string(iter) +  "_" + to_string(batch);
      MatrixXd t_s = readCSV(prefix + string("_t_and_s.csv"));
      MatrixXd t_ds = readCSV(prefix + string("_t_and_ds.csv"));
      MatrixXd t_dds = readCSV(prefix + string("_t_and_dds.csv"));
      for (int s_row = 0; s_row < FLAGS_n_s; s_row++) {
        t_s.row(1 + s_row) *= scaling_factors(s_row);
        t_ds.row(1 + s_row) *= scaling_factors(s_row);
        t_dds.row(1 + s_row) *= scaling_factors(s_row);
      }
      writeCSV(prefix + string("_t_and_s_new.csv"), t_s);
      writeCSV(prefix + string("_t_and_ds_new.csv"), t_ds);
      writeCSV(prefix + string("_t_and_dds_new.csv"), t_dds);
    }

    // Input part
    if (FLAGS_is_active_model) {
      for (int batch = 0; batch < FLAGS_num_batch; batch++) {
        prefix = directory + to_string(iter) +  "_" + to_string(batch);
        VectorXd w = readCSV(prefix + string("_w.csv")).col(0);

        // Read in inputs
        MatrixXd t_tau = readCSV(prefix + string("_t_and_tau.csv"));
        DRAKE_DEMAND(t_tau.rows() == FLAGS_n_tau + 1);
        DRAKE_DEMAND(t_tau.cols() == FLAGS_num_traj_opt_knots);
        MatrixXd tau = t_tau.block(1, 0, FLAGS_n_tau, FLAGS_num_traj_opt_knots);

        // Check if the idx are correct
        double idx_start = w.rows() - FLAGS_n_tau * FLAGS_num_traj_opt_knots;
        for (int j = 0; j < FLAGS_num_traj_opt_knots; j++) {
          for (int i = 0; i < FLAGS_n_tau; i++) {
            DRAKE_DEMAND(w(idx_start + j * FLAGS_n_tau + i) == tau(i, j));
          }
        }

        // Scaling tau
        // iterate through each element of s
        for (int s_row = 0; s_row < FLAGS_n_s; s_row++) {
          for (int i = 0; i < FLAGS_n_tau; i++) {
            // Identify which element of tau we should update
            // (We assume one input element go to one position element only.)
            VectorXd tau_temp = VectorXd::Zero(FLAGS_n_tau);
            tau_temp(i) = 1;
            if ((B_tau * tau_temp)(s_row) == 1) {
              cout << "(s_row = " << s_row << ", tau_idx = " << i << ")\n";
              // Update w
              for (int j = 0; j < FLAGS_num_traj_opt_knots; j++) {
                w(idx_start + j * FLAGS_n_tau + i) *= scaling_factors(s_row);
              }
              // Update t_and_tau
              t_tau.row(1 + i) *= scaling_factors(s_row);
            }
          }
        }  // end for (each row of s)

        writeCSV(prefix + string("_w_new.csv"), w);
        writeCSV(prefix + string("_t_and_tau_new.csv"), t_tau);
      }  // end for (batch)
    }  // end if (FLAGS_is_active_model)

  }  // end for (iter)


  return 0;
}

}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::goldilocks_models::doMain(argc, argv);
  return 0;
}
