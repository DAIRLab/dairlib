#include <vector>
#include "systems/goldilocks_models/file_utils.h"
#include "drake/common/drake_assert.h"

using Eigen::MatrixXd;
using Eigen::Matrix;
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
namespace goldilocks_models {

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

}  // namespace goldilocks_models
}  // namespace dairlib
