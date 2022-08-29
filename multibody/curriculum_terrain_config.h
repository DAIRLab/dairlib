#pragma once
#include "drake/common/yaml/yaml_read_archive.h"
#include "yaml-cpp/yaml.h"

struct CurriculumTerrainInfo {
    int total_length;
    int n_difficulties;
    int difficulty_length;
    std::vector<double> difficulty_heights;

    template <typename Archive>
    void Serialize(Archive *a) {
      a->Visit(DRAKE_NVP(total_length));
      a->Visit(DRAKE_NVP(n_difficulties));
      a->Visit(DRAKE_NVP(difficulty_length));
      a->Visit(DRAKE_NVP(difficulty_heights));
    }
};