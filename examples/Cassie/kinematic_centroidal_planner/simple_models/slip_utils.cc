#include "examples/Cassie/kinematic_centroidal_planner/simple_models/slip_utils.h"

#include <iostream>

#include "multibody/multibody_utils.h"

template <typename T>
T CalcSlipGrf(double k, double r0, double b, T r, T dr, T force) {
  auto F = force + k * (r0 - r) + b * (0 - dr);
  if (F < 0) {
    F = 0;
  }
  return F;
}

template double CalcSlipGrf<double>(double k, double r0, double b, double r,
                                    double dr, double force);
template drake::AutoDiffXd CalcSlipGrf<drake::AutoDiffXd>(
    double k, double r0, double b, drake::AutoDiffXd r, drake::AutoDiffXd dr,
    drake::AutoDiffXd force);
