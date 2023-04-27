#include "geometry/poly_utils.h"

namespace dairlib::geometry {

int polytest_main(int argc, char** argv) {
  Eigen::Matrix<double, 2, 6> bowtie;
  bowtie << 0.1, 1.0, -1.0, -0.1, -1.0, 1.0, 0.0, 1.0, 1.0, 0.0, -1.0, -1.0;
  auto out = TestAcd(bowtie);
  return 0;
}
}

int main(int argc, char** argv) {
  return dairlib::geometry::polytest_main(argc, argv);
}