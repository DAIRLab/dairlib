#pragma once
#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_io.h"

namespace dairlib::systems::controllers::id_mpc {

struct JointPDGains {
  std::unordered_map<std::string, double> kp;
  std::unordered_map<std::string, double> kd;

  template<typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(kp));
    a->Visit(DRAKE_NVP(kd));
  }

};

}