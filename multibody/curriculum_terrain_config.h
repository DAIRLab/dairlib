#pragma once
#include "drake/common/yaml/yaml_read_archive.h"
#include "yaml-cpp/yaml.h"
namespace dairlib::multibody {
    struct CurriculumTerrainInfo {
        int total_length;
        int n_difficulties;
        double difficulty_length;
        std::vector<double> difficulty_heights;
        double min_box_width;
        double max_box_width;

        template<typename Archive>
        void Serialize(Archive *a) {
          a->Visit(DRAKE_NVP(total_length));
          a->Visit(DRAKE_NVP(n_difficulties));
          a->Visit(DRAKE_NVP(difficulty_length));
          a->Visit(DRAKE_NVP(difficulty_heights));
          a->Visit(DRAKE_NVP(min_box_width));
          a->Visit(DRAKE_NVP(max_box_width));
        }
    };
}