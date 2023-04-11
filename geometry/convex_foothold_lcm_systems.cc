// STL includes
#include <iostream>

// dair includes
#include "poly_utils.h"
#include "convex_foothold_lcm_systems.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::RowVector3d;
using drake::systems::Context;

using convex_plane_decomposition_msgs::PlanarRegion;
using convex_plane_decomposition_msgs::PlanarTerrain;
using convex_plane_decomposition_msgs::Point2d;

namespace dairlib::geometry {

ConvexFootholdRosReceiver::ConvexFootholdRosReceiver() {
  PlanarTerrain terrain_msg;
  DeclareAbstractInputPort(
      "PlanarTerrainRosMsg", drake::Value<PlanarTerrain>(terrain_msg));
  DeclareAbstractOutputPort(
      "foolholds", &ConvexFootholdRosReceiver::CopyTerrain);
}

void ConvexFootholdRosReceiver::CopyTerrain(
    const drake::systems::Context<double> &context,
    ConvexFootholdSet* footholds) const {
  footholds->clear();
  const auto& planes =
      EvalAbstractInput(context, 0)->get_value<PlanarTerrain>();
  for (const auto& planar_region : planes.planarRegions) {
    footholds->append(
        MakeFootholdFromInnerApproximationWithIRIS(planar_region)
    );
  }
}


ConvexFootholdSender::ConvexFootholdSender() {
  ConvexFootholdSet set;
  DeclareAbstractInputPort("convex_foothold_set",
                           drake::Value<ConvexFootholdSet>(set));
  DeclareAbstractOutputPort("lcmt_convex_foothold_set",
                            &ConvexFootholdSender::CopyTerrain);
}

void ConvexFootholdSender::CopyTerrain(
    const Context<double> &context, lcmt_foothold_set *footholds) const {
  std::cout << context.get_time() << std::endl;
  auto foothold_set = EvalAbstractInput(context, 0)->get_value<ConvexFootholdSet>();
  foothold_set.CopyToLcm(footholds);
}

}