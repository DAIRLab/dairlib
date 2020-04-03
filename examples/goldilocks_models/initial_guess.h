//
// Created by jianshu on 3/25/20.
//
#include <Eigen/Dense>
#include <iostream>
#include "systems/goldilocks_models/file_utils.h"

using std::cout;
using std::string;
using std::to_string;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib::goldilocks_models {
    string set_initial_guess(const string directory, int iter,
            int sample, int total_sample_num, double min_sl, double max_sl,
            double min_gi, double max_gi);
//    to do
//    void get_theta_scale();
//    void get_gamma_scale();
} //namespace
