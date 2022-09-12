#include "flat_terrain_foothold_source.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems {

using multibody::SetPositionsAndVelocitiesIfNew;
using geometry::ConvexFoothold;

using Eigen::VectorXd;
using Eigen::Vector3d;

using drake::systems::Context;
using drake::multibody::MultibodyPlant;

FlatTerrainFootholdSource::FlatTerrainFootholdSource(
    const MultibodyPlant<double> &plant, Context<double> *context,
    std::vector<controllers::alip_utils::PointOnFramed> left_right_foot)
    : plant_(plant), context_(context), left_right_foot_(left_right_foot) {

  DRAKE_DEMAND(left_right_foot_.size() == 2);

  DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant_.num_positions(),
                                      plant_.num_velocities(),
                                      plant_.num_actuators()));

  DeclareAbstractOutputPort("footholds",
                            std::vector<geometry::ConvexFoothold>(1),
                            &FlatTerrainFootholdSource::CalcFoothold);

}

void FlatTerrainFootholdSource::CalcFoothold(
    const Context<double> &context,
    std::vector<geometry::ConvexFoothold> *footholds) const {

  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, 0));
  multibody::SetPositionsAndVelocitiesIfNew<double>(
      plant_, robot_output->GetState(), context_);

  const auto& pose_left = left_right_foot_.front().second.CalcPoseInWorld(*context_);
  const auto& pose_right = left_right_foot_.back().second.CalcPoseInWorld(*context_);

  Vector3d left_pos = pose_left.translation() + pose_left.rotation() * left_right_foot_.front().first;
  Vector3d right_pos = pose_right.translation() + pose_right.rotation() * left_right_foot_.back().first;

  double h = std::min(left_pos(2), right_pos(2));
  left_pos(2) = h;

  ConvexFoothold f;
  f.SetContactPlane(Vector3d::UnitZ(), left_pos);
  f.AddFace(-Vector3d::UnitZ(), left_pos -Vector3d::UnitZ());
  footholds->clear();
  footholds->push_back(f);
}


}