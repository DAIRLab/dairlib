#include "plane_seg_system.h"

#include "grid_map_core/grid_map_core.hpp"

#include "systems/perception/pointcloud/point_cloud_conversions.h"

namespace dairlib::perception {

using Eigen::Vector3f;
using grid_map::GridMap;

PlaneSegSystem::PlaneSegSystem(std::string layer) : layer_(layer) {
  DeclareAbstractInputPort("elevation_map", drake::Value<GridMap>());
  DeclareAbstractOutputPort("polygons", &PlaneSegSystem::CalcOutput);
}

// Note - the comments in this function are directly copied from the plane_seg
// repository
planeseg::BlockFitter::Result PlaneSegSystem::ProcessDataAsCloud(
    planeseg::LabeledCloud::Ptr &inCloud, Vector3f origin,
    Vector3f lookDir) const {

  planeseg::BlockFitter fitter;
  fitter.setSensorPose(origin, lookDir);
  fitter.setCloud(inCloud);
  fitter.setDebug(false); // MFALLON modification
  fitter.setRemoveGround(false); // MFALLON modification from default

  // this was 5 for LIDAR. changing to 10 really improved elevation map segmentation
  // I think its because the RGB-D map can be curved
  fitter.setMaxAngleOfPlaneSegmenter(10);

  return fitter.go();
}

void PlaneSegSystem::CalcOutput(
    const drake::systems::Context<double> &context,
    geometry::ConvexPolygonSet *output) const {

  // translate map to pointcloud
  GridMap grid_map = this->EvalAbstractInput(context, 0)->get_value<GridMap>();
  planeseg::LabeledCloud::Ptr inCloud(new planeseg::LabeledCloud());

  AssignFields<pcl::PointXYZL>(grid_map, layer_, inCloud);

  Vector3f origin = Vector3f::Zero();
  origin.head<2>() = grid_map.getPosition().cast<float>();
  planeseg::BlockFitter::Result res =
      ProcessDataAsCloud(inCloud, origin, Vector3f::UnitX());

  output->clear();

  for (const auto& block: res.mBlocks) {
    geometry::ConvexPolygon poly;
    poly.SetPlane(block.mPose.linear().col(2).cast<double>(),
                  block.mPose.translation().cast<double>());

    for (size_t i = 0; i <= block.mHull.size(); ++i) {
      size_t j = (i+1) % block.mHull.size();
      poly.AddVertices(block.mHull.at(i).cast<double>(),
                       block.mHull.at(j).cast<double>());
    }
    output->append(poly);
  }
}

}