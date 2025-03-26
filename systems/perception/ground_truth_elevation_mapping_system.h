#pragma once

#include "geometry/polygon_height_map.h"
#include "grid_map_core/grid_map_core.hpp"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::perception {

class GroundTruthElevationMappingSystem : public drake::systems::LeafSystem<double> {
 public:
  struct map_params {
    double resolution;
    double map_length;
    double map_width;
    Eigen::Vector3d track_point;
    std::string base_frame;
  };

  explicit GroundTruthElevationMappingSystem(
      const drake::multibody::MultibodyPlant<double>& plant,
      const geometry::ConvexPolygonSet& polygons,
      const map_params& params);

 private:

  void CalcMap(const drake::systems::Context<double>& context,
               grid_map::GridMap* map) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  geometry::PolygonHeightMap global_height_map_;

  const map_params params_;

};



}

