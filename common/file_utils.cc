#include <vector>
#include "common/file_utils.h"
#include "drake/common/drake_assert.h"

using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::VectorXd;
using Eigen::Dynamic;
using Eigen::Map;
using Eigen::RowMajor;
using std::ofstream;
using std::ifstream;
using std::cout;
using std::string;
using std::cout;

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                       Eigen::DontAlignCols, ", ", "\n");

namespace dairlib {

CSVReader::CSVReader(std::string filepath) {
  indata_.open(filepath);
}

bool CSVReader::has_next() {
  if (std::getline(indata_, next_line_)) {
    has_next_ = true;
    return true;
  }
  has_next_ = false;
  return false;
}

Eigen::VectorXd CSVReader::next() {
  if (!has_next_) {
    throw std::logic_error("called next() when next line did not exist. "
                           "Either has_next() has not been called first or "
                           "you have reached the end of the CSV");
  }
  std::stringstream lineStream(next_line_);
  std::vector<double> data;
  string cell;
  while(std::getline(lineStream, cell, ',')){
    data.push_back(std::stod(cell));
  }
  return Map<const VectorXd>(data.data(), data.size());
  has_next_ = false;
}

MatrixXd readCSV(const string & path) {
    ifstream indata;
    indata.open(path);
    string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    if (values.size() == 0) {
      throw std::logic_error(
          ("Could not read " + path + " to load CSV.").c_str());
    }
    return Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>(
        values.data(), rows, values.size()/rows);
}

void writeCSV(const std::string& path, const MatrixXd& M) {
  ofstream outfile;
  outfile.open(path);
  outfile << M.format(CSVFormat);
  outfile.close();
}

void appendCSV(const std::string& path, const MatrixXd& M) {
  ofstream outfile;
  outfile.open(path, std::ios_base::app);
  outfile << M.format(CSVFormat);
  outfile << "\n";
  outfile.close();
}

}  // namespace dairlib
