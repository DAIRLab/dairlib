#pragma once

#include "yaml-cpp/yaml.h"
#include "drake/common/yaml/yaml_read_archive.h"

struct RomDynEvalSettings {
  std::string dir_model;
  std::string dir_data;

  double rom_option;
  double model_iter;

  double x_min;
  double x_max;
  int n_samples_x;
  double y_min;
  double y_max;
  int n_samples_y;
  double z_min;
  double z_max;
  int n_samples_z;

  bool save_fig;


  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(dir_model));
    a->Visit(DRAKE_NVP(dir_data));

    a->Visit(DRAKE_NVP(rom_option));
    a->Visit(DRAKE_NVP(model_iter));

    a->Visit(DRAKE_NVP(x_min));
    a->Visit(DRAKE_NVP(x_max));
    a->Visit(DRAKE_NVP(n_samples_x));
    a->Visit(DRAKE_NVP(y_min));
    a->Visit(DRAKE_NVP(y_max));
    a->Visit(DRAKE_NVP(n_samples_y));
    a->Visit(DRAKE_NVP(z_min));
    a->Visit(DRAKE_NVP(z_max));
    a->Visit(DRAKE_NVP(n_samples_z));

    a->Visit(DRAKE_NVP(save_fig));
  }
};
