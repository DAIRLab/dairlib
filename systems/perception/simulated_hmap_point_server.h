#pragma once

#include <pcl/filters/frustum_culling.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


#include "drake/systems/framework/leaf_system.h"
#include "systems/perception/elevation_mapping_system.h"
#include "geometry/convex_polygon_set.h"
#include "geometry/polygon_height_map.h"



namespace dairlib {
namespace perception {

template<typename PointT>
class SimulatedHmapPointServer : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimulatedHmapPointServer);
  SimulatedHmapPointServer(const drake::multibody::MultibodyPlant<double>& plant,
                           std::unique_ptr<drake::systems::Context<double>> context,
                           elevation_mapping_params params,
                           drake::systems::sensors::CameraInfo camera_info,
                           geometry::ConvexPolygonSet polygons);

 private:

  void Calc(const drake::systems::Context<double> &context,
            typename pcl::PointCloud<PointT>::Ptr *ptr) const;

  void AssignFields(const drake::systems::Context<double> &context,
                    typename pcl::PointCloud<PointT>::Ptr& ptr) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const drake::multibody::Body<double>& robot_base_;


  sensor_pose_params sensor_pose_;
  drake::systems::sensors::CameraInfo camera_info_;
  const Eigen::Vector3d track_point_;

  elevation_mapping_params params_;

  geometry::ConvexPolygonSet polygons_;
  geometry::PolygonHeightMap hmap_;


  drake::systems::InputPortIndex input_port_state_;
  drake::systems::OutputPortIndex output_port_pcl_;

  mutable pcl::FrustumCulling<PointT> fov_filter_{};

};

}
}

