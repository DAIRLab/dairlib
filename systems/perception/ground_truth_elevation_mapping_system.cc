#include <iostream>
#include "ground_truth_elevation_mapping_system.h"

#include "systems/framework/output_vector.h"

namespace dairlib::perception {

using drake::systems::Context;
using drake::multibody::MultibodyPlant;

using geometry::ConvexPolygonSet;
using geometry::PolygonHeightMap;
using systems::OutputVector;


GroundTruthElevationMappingSystem::GroundTruthElevationMappingSystem(
    const MultibodyPlant<double> &plant,
    const geometry::ConvexPolygonSet &polygons,
    const GroundTruthElevationMappingSystem::map_params &params) :
    plant_(plant),
    plant_context_(plant_.CreateDefaultContext()),
    global_height_map_(polygons, params.resolution),
    params_(params){

  DeclareVectorInputPort("x, u, t", OutputVector<double>(plant));

  DeclareAbstractOutputPort(
      "elevation_map", grid_map::GridMap(),
      &GroundTruthElevationMappingSystem::CalcMap);
}

void GroundTruthElevationMappingSystem::CalcMap(const Context<double>& context,
                                           grid_map::GridMap* map) const {

  const auto& robot_state = get_input_port().Eval<OutputVector<double>>(context);
  plant_.SetPositions(plant_context_.get(), robot_state.GetPositions());

  const Eigen::Vector3d track_point_world = plant_.GetBodyByName(params_.base_frame).EvalPoseInWorld(*plant_context_) * params_.track_point;

  map->setGeometry(
      grid_map::Length(params_.map_length, params_.map_width),
      params_.resolution,
      track_point_world.head<2>()
  );
  map->add("elevation");
  grid_map::GridMapIterator iterator(*map);

  for (; not iterator.isPastEnd(); ++iterator) {
    grid_map::Position pos;
    map->getPosition(*iterator, pos);
    double height = global_height_map_.CalcHeight(pos[0], pos[1]);
    if (std::isinf(height)) {
      height = std::nan("nan");
    }
    map->at("elevation", *iterator) = height;
  }

}

}
