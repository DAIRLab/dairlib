#include "curriculum_height_map.h"

#include <utility>
#include "boxy_height_map.h"

namespace dairlib::multibody {
    CurriculumHeightMap::CurriculumHeightMap(
            const Eigen::Vector3d &normal, double dim_y, double dim_z,
            double rot_z, double mu,
            CurriculumTerrainInfo curriculum_params) : BoxyHeightMap(normal, dim_y, dim_z, rot_z, mu),
            curriculum_params_(std::move(curriculum_params)) {
      double global_marker = 0.0;
      std::cout << "Building Curriculum Map! num difficulties = " << curriculum_params_.n_difficulties << std::endl;
      for (int d=0; d < curriculum_params_.n_difficulties; d++) {
        double marker = 0.0;
        while (marker < curriculum_params_.difficulty_length) {
          double h = randd(0.0, curriculum_params_.difficulty_heights.at(d));
          double w = randd(curriculum_params_.min_box_width, curriculum_params_.max_box_width);
          marker = marker + w;
          global_marker = global_marker + w;
          BoxyHeightMap::AppendBox(h, w);
          if (global_marker > curriculum_params_.total_length) {
            return; // Done building the map
          }
        }
      }
    }
}
