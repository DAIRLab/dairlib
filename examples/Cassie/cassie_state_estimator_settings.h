#pragma once

#include "drake/common/yaml/yaml_read_archive.h"


struct CassieStateEstimatorContactThresholds{

  double knee_spring_threshold;
  double ankle_spring_threshold;
  double contact_force_threshold;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(knee_spring_threshold));
    a->Visit(DRAKE_NVP(ankle_spring_threshold));
    a->Visit(DRAKE_NVP(contact_force_threshold));
  }
};