#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>

namespace dairlib {

/// Class to incrementally read CSV file rows as Eigen::VectorXd
/// useful for dealing with large files which might be too large or cumberseome
/// to fit in a ssingle Matrix
class CSVReader {
 public:
  CSVReader(std::string filename);
  bool has_next();
  Eigen::VectorXd next();
 private:
  bool has_next_;
  std::ifstream indata_;
  std::string next_line_;
};

/// Read a CSV formatted file as an Eigen Matrix
Eigen::MatrixXd readCSV(const std::string & path);

/// Write an Eigen Matrix into a CSV formatted file
void writeCSV(const std::string& path, const Eigen::MatrixXd& M);

void appendCSV(const std::string& path, const Eigen::MatrixXd& M);

}  // namespace dairlib
