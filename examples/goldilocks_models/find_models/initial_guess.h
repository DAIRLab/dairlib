//
// Created by jianshu on 3/25/20.
//
#include <iostream>
#include <random>
#include <Eigen/Dense>

#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "examples/goldilocks_models/task.h"
#include "common/file_utils.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib::goldilocks_models {
// set initial guess using interpolation
std::string SetInitialGuessByInterpolation(const std::string& directory,
                                           int iter, int sample,
                                           const TasksGenerator* task_gen,
                                           const Task& task,
                                           const ReducedOrderModel& rom);
// this is used in the feature of trying mediate samples to help failed samples;
std::string ChooseInitialGuessFromMediateIteration(const string& directory,
    int iter,int sample,const TasksGenerator* task_gen,const Task& task,
    const ReducedOrderModel& rom,const MediateTasksGenerator& task_gen_mediate);
std::string SetInitialGuessByExtrapolation(const string& directory, int iter,
                                      int sample,
                                      const TasksGenerator* task_gen,
                                      const Task& task);
// set scale for theta
Eigen::VectorXd GetThetaScale(const std::string& directory, int iter);
// utility functions
Eigen::VectorXd CalculateInterpolation(const Eigen::VectorXd& weight_vector,
                                       const Eigen::MatrixXd& solution_matrix);
void InterpolateAmongDifferentTasks(const std::string& dir, string prefix,
                                    const Eigen::VectorXd& current_task,
                                    const Eigen::VectorXd& task_scale,
                                    Eigen::VectorXd& weight_vector,
                                    Eigen::MatrixXd& solution_matrix);
}  // namespace dairlib::goldilocks_models
