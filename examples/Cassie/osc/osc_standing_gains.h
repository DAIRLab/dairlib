#pragma once

#include "systems/controllers/osc/osc_gains.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_read_archive.h"

struct OSCStandingGains : OSCGains  {
  int rows;
  int cols;
  double HipYawKp;
  double HipYawKd;
  double HipYawW;
  std::vector<double> CoMKp;
  std::vector<double> CoMKd;
  std::vector<double> PelvisRotKp;
  std::vector<double> PelvisRotKd;
  std::vector<double> CoMW;
  std::vector<double> PelvisW;

  template <typename Archive>
  void Serialize(Archive* a) {
    OSCGains::Serialize(a);
    a->Visit(DRAKE_NVP(rows));
    a->Visit(DRAKE_NVP(cols));
    a->Visit(DRAKE_NVP(CoMKp));
    a->Visit(DRAKE_NVP(CoMKd));
    a->Visit(DRAKE_NVP(PelvisRotKp));
    a->Visit(DRAKE_NVP(PelvisRotKd));
    a->Visit(DRAKE_NVP(HipYawKp));
    a->Visit(DRAKE_NVP(HipYawKd));
    a->Visit(DRAKE_NVP(CoMW));
    a->Visit(DRAKE_NVP(PelvisW));
    a->Visit(DRAKE_NVP(HipYawW));
  }
};
