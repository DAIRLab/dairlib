//
// Created by brian on 6/29/22.
//

#include "state_to_height_map.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::simulation {

using drake::multibody::MultibodyPlant;
using drake::multibody::Frame;
using drake::multibody::BodyFrame;
using drake::systems::Context;
using drake::math::RotationMatrix;
using drake::MatrixX;

using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;


StateToHeightMap::StateToHeightMap(const MultibodyPlant<double>& plant,
                                   Context<double> &context,
                                   const BodyFrame<double> &frame_B,
                                   const Vector3d &origin_in_frame_B,
                                   const RotationMatrix<double> &R_BE,
                                   height_query_params params,
                                   const multibody::BoxyHeightMap &height_map) :
    plant_(plant),
    context_(context),
    frame_B_(frame_B),
    origin_in_frame_B_(origin_in_frame_B),
    params_(params),
    R_BE_(R_BE),
    height_map_(height_map) {

  this->DeclareVectorInputPort(
          "x, u, t", OutputVector<double>(plant.num_positions(),
                                          plant.num_velocities(),
                                          plant.num_actuators()))
      .get_index();

  this->DeclareAbstractOutputPort(
      "height_map", drake::MatrixX<double>(),
        &StateToHeightMap::CalcHeightMap);
}

void StateToHeightMap::CalcHeightMap(const Context<double> &context,
                                     MatrixX<double> *height_map) const {
  // Read in current robot state
  const OutputVector<double> *robot_output =
      (OutputVector<double> *) this->EvalVectorInput(context, 0);

}


}