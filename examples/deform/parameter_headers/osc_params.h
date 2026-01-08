#pragma once

#include "examples/sampling_c3/parameter_headers/osc_params.h"

#include "drake/common/yaml/yaml_read_archive.h"

/// Extends DeformOSCParams and for now does not add anything.
struct DeformOSCParams : SamplingC3OSCParams {
  template <typename Archive>
  void Serialize(Archive* a) {
    SamplingC3OSCParams::Serialize(a);
  }
};
