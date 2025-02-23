#include "blending_utils.h"

#include <Eigen/Dense>

double blend_sigmoid(double t, double tau, double window) {
  double x = (t + window) / tau;
  return exp(x) / (1 + exp(x));
}

double blend_exp(double t, double tau, double window) {
  return 1 - exp(-(t + window) / tau);
}
