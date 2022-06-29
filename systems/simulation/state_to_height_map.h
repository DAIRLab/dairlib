#pragma once

#include "multibody/boxy_height_map.h"
#include "drake/systems/framework/leaf_system.h"

/// TODO: Finish this if necessary
namespace dairlib::systems::simulation {

struct height_query_params {
  Eigen::Vector2d xlim;
  Eigen::Vector2d ylim;
  int nx;
  int ny;
};

class StateToHeightMap : public drake::systems::LeafSystem<double> {
 public:
  StateToHeightMap(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>& context,
                   const drake::multibody::BodyFrame<double>& frame_B,
                   const Eigen::Vector3d& origin_in_frame_B,
                   const drake::math::RotationMatrix<double>& R_BE,
                   height_query_params params,
                   const multibody::BoxyHeightMap& height_map);

 private:

  void CalcHeightMap(const drake::systems::Context<double>& context,
                     drake::MatrixX<double>* height_map) const;

  const height_query_params params_;
  const multibody::BoxyHeightMap& height_map_;

  // Member variables for coordinate transforms
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>& context_;
  const drake::multibody::BodyFrame<double>& frame_B_;
  const Eigen::Vector3d& origin_in_frame_B_;
  const drake::math::RotationMatrix<double>& R_BE_;
};

}

