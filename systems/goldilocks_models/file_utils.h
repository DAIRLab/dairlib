#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <fstream>

using std::cout;
using std::string;

namespace drake {
namespace goldilocks_models {

Eigen::MatrixXd readCSV (const string & path);

void writeCSV(const std::string& path, const Eigen::MatrixXd& M);

}
}