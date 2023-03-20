#include "common/file_utils.h"

#include <limits>
#include <vector>

#include "drake/common/drake_assert.h"

using Eigen::Dynamic;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::RowMajor;
using std::cout;
using std::ifstream;
using std::ofstream;
using std::string;

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                       Eigen::DontAlignCols, ", ", "\n");
const static Eigen::IOFormat CSVFormatFullPrec(Eigen::FullPrecision,
                                               Eigen::DontAlignCols, ", ",
                                               "\n");

typedef std::numeric_limits<double> dbl;
const static Eigen::IOFormat CSVFormatManuallySpecifiedPrecision(
    dbl::max_digits10, Eigen::DontAlignCols, ", ", "\n");

// Note: Use CSVFormatManuallySpecifiedPrecision instead of CSVFormatFullPrec.
// CSVFormatManuallySpecifiedPrecision has higher precision.

namespace dairlib {

MatrixXd readCSV(const string& path) {
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
    return MatrixXd(0, 0);
  } else {
    return Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>(
        values.data(), rows, values.size() / rows);
  }
}

void writeCSV(const std::string& path, const MatrixXd& M, bool full_precision) {
  ofstream outfile;
  outfile.open(path);
  if (full_precision) {
    outfile << M.format(CSVFormatManuallySpecifiedPrecision);
  } else {
    outfile << M.format(CSVFormat);
  }
  outfile.close();
}

}  // namespace dairlib
