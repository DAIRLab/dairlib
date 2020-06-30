//
// Created by jianshu on 3/25/20.
//
#include <iostream>
#include <random>
#include <Eigen/Dense>

#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/task.h"
#include "systems/goldilocks_models/file_utils.h"
#include "drake/solvers/mathematical_program.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::string;
using std::to_string;

namespace dairlib::goldilocks_models {
// set initial guess using interpolation
string SetInitialGuessByInterpolation(const string directory, int iter,
                                      int sample, GridTasksGenerator task_gen,
                                      bool use_database, int robot, Task task,
                                      RomData rom);
// set scale for theta and gamma
VectorXd GetThetaScale(const string directory, int iter);
VectorXd GetGammaScale(GridTasksGenerator task_gen);
// utility functions
VectorXd CalculateInterpolation(VectorXd weight_vector,
                                MatrixXd solution_matrix);
void InterpolateAmongDifferentTasks(const string dir, string prefix,
                                    VectorXd current_gamma,
                                    VectorXd gamma_scale,
                                    VectorXd& weight_vector,
                                    MatrixXd& solution_matrix);
}  // namespace dairlib::goldilocks_models
