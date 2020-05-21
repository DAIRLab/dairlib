//
// Created by jianshu on 3/25/20.
//
#include <Eigen/Dense>
#include <iostream>
#include "systems/goldilocks_models/file_utils.h"
#include "drake/solvers/mathematical_program.h"

using std::cout;
using std::string;
using std::to_string;
using std::endl;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib::goldilocks_models {
//    set initial guess using interpolation
    string set_initial_guess(const string directory, int iter,
            int sample, int total_sample_num, double min_sl, double max_sl,
            double min_gi, double max_gi, double min_tr, double max_tr);
//    set scale for theta and gamma
    MatrixXd get_theta_scale(const string directory, int iter);
    MatrixXd get_gamma_scale(int gamma_length, double min_sl, double max_sl, double min_gi, double max_gi);
} //namespace
