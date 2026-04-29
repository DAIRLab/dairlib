#pragma once
#include <numeric>

#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"

struct ElastoPlasticSC3Options : SamplingC3Options {
  Eigen::VectorXd q_target;
  std::vector<std::string> state_names;
  Eigen::VectorXd Kp;
  Eigen::VectorXd Kd;
  double w_Q_final;

  template <typename Archive>
  void Serialize(Archive* a) {
    SamplingC3Options::Serialize(a);
    a->Visit(DRAKE_NVP(q_target));
    a->Visit(DRAKE_NVP(state_names));
    a->Visit(DRAKE_NVP(Kp));
    a->Visit(DRAKE_NVP(Kd));
    a->Visit(DRAKE_NVP(w_Q_final));
  }

  /**
   * NOTE:  For elastoplastic systems, there is no difference between position
   * and pose tracking mode since the system state is represented by 3D node
   * positions.  Thus, C3Options and LCSFactory options will always use the
   * position parameters, and the pose parameters are unused.
   */
  C3Options GetC3Options() const { return c3_options_position; }
  LCSFactoryOptions GetLCSFactoryOptions() const {
    return lcs_factory_options_position;
  }
};
