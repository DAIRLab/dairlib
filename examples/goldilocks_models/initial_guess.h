//
// Created by jianshu on 3/25/20.
//
#include <Eigen/Dense>
#include <iostream>
#include "systems/goldilocks_models/file_utils.h"
#include "drake/solvers/mathematical_program.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/task.h"
#include <random>

using std::cout;
using std::string;
using std::to_string;
using std::endl;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib::goldilocks_models {
//    set initial guess using interpolation
    string set_initial_guess(const string directory, int iter,
            int sample, GridTasksGenerator task_gen,
            bool use_database,int robot);
//    set scale for theta and gamma
    MatrixXd get_theta_scale(const string directory, int iter);
    MatrixXd get_gamma_scale(GridTasksGenerator task_gen);
//    test initial guess
    int test_initial_guess(int iter_,int sample_,int robot_);
} //namespace
