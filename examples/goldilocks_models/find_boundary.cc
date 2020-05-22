//
// Created by jianshu on 5/20/20.
//

#include <gflags/gflags.h>
#include <stdio.h>  // For removing files
#include <thread>  // multi-threading
#include <chrono>
#include <ctime>
#include <queue>  // First in first out
#include <deque>  // queue with feature of finding elements
#include <utility>  // std::pair, std::make_pair
#include <bits/stdc++.h>  // system call
#include <cmath>
#include <numeric> // std::accumulate
#include <tuple>
#include <Eigen/QR>  // CompleteOrthogonalDecomposition

#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "examples/goldilocks_models/dynamics_expression.h"
#include "examples/goldilocks_models/find_models/traj_opt_given_weigths.h"
#include "examples/goldilocks_models/kinematics_expression.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/initial_guess.h"
#include "systems/goldilocks_models/file_utils.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::pair;
using std::string;
using std::to_string;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXcd;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::AutoDiffXd;
using dairlib::FindResourceOrThrow;


namespace dairlib::goldilocks_models {
double boundary_for_one_dimension(int max_iteration,double initial_low,
    double initial_high, double resolution){
  int iter = 0;
  double low = initial_low;
  double high = initial_high;
  for (iter = 0; iter <= max_iteration; iter++){
    //run trajectory optimization and judge the solution
    //int sample_success = traj_opt_result();
    int sample_success = 1;
    if (sample_success){
      low = high;
      high = 2*high;
    }
    else{
      high = (high+low)/2;
    }
    if(abs(high-low) < resolution){
      break;
    }
  }
  return low;
}

int find_boundary(int argc, char* argv[]){
  const string dir = "../dairlib_data/goldilocks_models/find_boundary/robot_" +
      to_string(FLAGS_robot_option) + "/";
  /*
   * initialize model
   */


  /*
   * initialize task space
   */
  double stride_length_0 = 0.2;
  double stride_length_resolution = 0.01;
  double ground_incline_0 = 0.1;
  double ground_incline_resolution = 0.01;
  double turning_rate_0 = 0.1;
  double turning_rate_resolution = 0.125;

  //iteration setting
  int max_sl_iter = 200;
  int min_sl_iter = 200;
  int max_gi_iter = 200;
  int min_gi_iter = 200;

  /*
   * start iteration
   */

  int iter;
  int boundary_sample_num = 0;

  double sl = 0.2;
  for (iter = 0; iter <= max_sl_iter; iter++){
    //fix stride length
    double sl = stride_length_0;

    //find max ground incline
    double max_gi = boundary_for_one_dimension(max_gi_iter,0,ground_incline_0,
        ground_incline_resolution);
    boundary_sample_num += 1;
    writeCSV();

    //find min ground incline
    double min_gi = boundary_for_one_dimension(min_gi_iter,0,-ground_incline_0,
        ground_incline_resolution);
    boundary_sample_num += 1;
    writeCSV(directory + initial_file_name, initial_guess);

  }

  return 0;
}
}


int main(int argc, char* argv[]) {
    return dairlib::goldilocks_models::find_boundary(argc, argv);
}