#pragma once

#include <drake/common/yaml/yaml_io.h>

#include "common/file_utils.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"

#include "drake/common/yaml/yaml_read_archive.h"

/* Deformable model type options:
    0. kPlastic:  forms a plastic network (plastic prismatic joints with
         corresponding complementarity forces).
    1. kElastic:  forms an elastic network (spring-dampers).
    2. kElastoPlastic:  forms an elasto-plastic network (where each connection
         is a spring in series with a plastic prismatic joint, both in parallel
         with a damper). */
enum DeformableModelType {
  kPlastic,
  kElastic,        // TODO @bibit:  unimplemented
  kElastoPlastic,  // TODO @bibit:  unimplemented
};

struct ElastoPlasticSC3Options : SamplingC3Options {
  DeformableModelType deformable_model_type;

  std::vector<double> Kp;
  std::vector<double> Kd;
  double w_Q_final;

  // Serialize.
  template <typename Archive>
  void Serialize(Archive* a) {
    SamplingC3Options::Serialize(a);
    ENUM_DESERIALIZE(a, deformable_model_type);

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
