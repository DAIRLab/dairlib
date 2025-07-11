#include "math_utils.h"

#include <Eigen/Dense>

double blend_sigmoid(double t, double tau, double window) {
  double x = (t + window) / tau;
  return exp(x) / (1 + exp(x));
}

double blend_exp(double t, double tau, double window) {
  return 1 - exp(-(t + window) / tau);
}
namespace dairlib {
  int FindBin(const double* bins, int n, double x) {
    int low = 0;
    int high = n - 1;

    while (low < high - 1) {
      int mid = (high + low) / 2;
      if (bins[mid] <= x) {
        low = mid;
      } else {
        high = mid;
      }
    }
    return low;
  }
}