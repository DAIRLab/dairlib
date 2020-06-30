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

namespace dairlib::goldilocks_models {
// set initial guess using interpolation
std::string SetInitialGuessByInterpolation(const std::string& directory,
                                           int iter, int sample,
                                           const TasksGenerator* task_gen,
                                           const Task& task, const RomData& rom,
                                           bool use_database, int robot);
// set scale for theta and gamma
Eigen::VectorXd GetThetaScale(const std::string& directory, int iter);
Eigen::VectorXd GetGammaScale(const TasksGenerator* task_gen);
// utility functions
Eigen::VectorXd CalculateInterpolation(Eigen::VectorXd weight_vector,
                                       Eigen::MatrixXd solution_matrix);
void InterpolateAmongDifferentTasks(const std::string& dir, string prefix,
                                    Eigen::VectorXd current_gamma,
                                    Eigen::VectorXd gamma_scale,
                                    Eigen::VectorXd& weight_vector,
                                    Eigen::MatrixXd& solution_matrix);
}  // namespace dairlib::goldilocks_models
