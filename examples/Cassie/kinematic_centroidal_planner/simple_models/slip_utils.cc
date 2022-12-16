#include "examples/Cassie/kinematic_centroidal_planner/simple_models/slip_utils.h"

#include <iostream>

#include "multibody/multibody_utils.h"

template <typename T>
T SlipGrf(double k, double r0, double b, T r, T dr, T force) {
  auto F = k * (r0 - r) + force - b * dr;
  if (F < 0) {
    F = 0;
  }
  return F;
}

template double SlipGrf<double>(double k, double r0, double b, double r,
                                double dr, double force);
template drake::AutoDiffXd SlipGrf<drake::AutoDiffXd>(double k, double r0,
                                                      double b,
                                                      drake::AutoDiffXd r,
                                                      drake::AutoDiffXd dr,
                                                      drake::AutoDiffXd force);
