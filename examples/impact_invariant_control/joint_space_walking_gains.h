#include "drake/common/yaml/yaml_read_archive.h"

using Eigen::MatrixXd;

struct JointSpaceWalkingGains {
  // costs
  double w_input;
  double w_accel;
  double w_soft_constraint;
  std::vector<double> JointW;
  std::vector<double> JointKp;
  std::vector<double> JointKd;
  double impact_threshold;
  double mu;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(w_input));
    a->Visit(DRAKE_NVP(w_accel));
    a->Visit(DRAKE_NVP(w_soft_constraint));
    a->Visit(DRAKE_NVP(JointW));
    a->Visit(DRAKE_NVP(JointKp));
    a->Visit(DRAKE_NVP(JointKd));
    a->Visit(DRAKE_NVP(impact_threshold));
    a->Visit(DRAKE_NVP(mu));
  }
};