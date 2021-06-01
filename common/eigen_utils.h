#pragma once

#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <cnpy.h>

/// CopyVectorXdToStdVector returns an std::vector<double> which is converted
/// from an Eigen::VectorXd.
std::vector<double> CopyVectorXdToStdVector(
    const Eigen::VectorXd& eigen_vec);

void cnpy2eigen(std::string data_fname, Eigen::MatrixXd& mat_out);