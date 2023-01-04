#pragma once
#include "boxy_height_map.h"
#include "curriculum_terrain_config.h"

namespace dairlib::multibody {
    using Eigen::Vector3d;

    class CurriculumHeightMap : public BoxyHeightMap {
    public:
        CurriculumHeightMap() = default;

        CurriculumHeightMap(const Vector3d &normal,
                            double dim_y, double dim_z,
                            double rot_z, double mu,
                            CurriculumTerrainInfo curriculum_params);

    private:
        CurriculumTerrainInfo curriculum_params_;
    };
}
