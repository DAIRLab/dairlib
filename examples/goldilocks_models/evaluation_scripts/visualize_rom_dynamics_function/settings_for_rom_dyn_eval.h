#pragma once

#include "yaml-cpp/yaml.h"
#include "drake/common/yaml/yaml_read_archive.h"

struct RomDynEvalSettings {
  std::string dir_script_backup;
  std::string dir_model;
  std::string dir_data;

  int rom_option;
  int model_iter;

  double comdot_x;
  double comdot_y;
  double comdot_z;

  double x_min;
  double x_max;
  int n_samples_x;
  double y_min;
  double y_max;
  int n_samples_y;
  double z_min;
  double z_max;
  int n_samples_z;

  int plot_xz_or_yz;
  bool save_fig;


  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(dir_script_backup));
    a->Visit(DRAKE_NVP(dir_model));
    a->Visit(DRAKE_NVP(dir_data));

    a->Visit(DRAKE_NVP(rom_option));
    a->Visit(DRAKE_NVP(model_iter));

    a->Visit(DRAKE_NVP(comdot_x));
    a->Visit(DRAKE_NVP(comdot_y));
    a->Visit(DRAKE_NVP(comdot_z));

    a->Visit(DRAKE_NVP(x_min));
    a->Visit(DRAKE_NVP(x_max));
    a->Visit(DRAKE_NVP(n_samples_x));
    a->Visit(DRAKE_NVP(y_min));
    a->Visit(DRAKE_NVP(y_max));
    a->Visit(DRAKE_NVP(n_samples_y));
    a->Visit(DRAKE_NVP(z_min));
    a->Visit(DRAKE_NVP(z_max));
    a->Visit(DRAKE_NVP(n_samples_z));

    a->Visit(DRAKE_NVP(plot_xz_or_yz));
    a->Visit(DRAKE_NVP(save_fig));
  }
};
