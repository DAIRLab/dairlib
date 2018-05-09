#include <vector>
#include "file_utils.h"

using namespace Eigen;
using std::ofstream;
using std::ifstream;
using std::cout;
using std::string;

const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");


namespace drake {
namespace goldilocks_walking {

MatrixXd readCSV (const string & path) {
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
    return Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>(values.data(), rows, values.size()/rows);
}

void writeCSV(const std::string& path, const MatrixXd& M) {
  ofstream outfile;
  outfile.open(path);
  outfile << M.format(CSVFormat);
  outfile.close();
}

}
}