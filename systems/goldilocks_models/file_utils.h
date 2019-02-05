#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>

namespace dairlib {
namespace goldilocks_models {

/// Read a CSV formatted file as an Eigen Matrix
Eigen::MatrixXd readCSV(const std::string & path);

/// Write an Eigen Matrix into a CSV formatted file
void writeCSV(const std::string& path, const Eigen::MatrixXd& M);

}  // namespace goldilocks_models
}  // namespace dairlib
