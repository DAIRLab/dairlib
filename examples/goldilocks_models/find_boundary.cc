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
    int find_boundary(int argc, char* argv[]){
        const string dir = "../dairlib_data/goldilocks_models/find_models/robot_0/";

        // initialize model

        // initialize task space

        // run trajectory optimizations

        // extend the task space

        return 0;
    }
}


int main(int argc, char* argv[]) {
    return dairlib::goldilocks_models::find_boundary(argc, argv);
}