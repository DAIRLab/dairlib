// STL includes
#include <iostream>

// dair includes
#include "convex_polygon_lcm_systems.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::RowVector3d;
using drake::systems::Context;

namespace dairlib::geometry {


ConvexPolygonReceiver::ConvexPolygonReceiver() {
  DeclareAbstractInputPort("lcmt_convex_foothold_set",
                           drake::Value<lcmt_foothold_set>());
  DeclareAbstractOutputPort("convex_foothold_set",
                            &ConvexPolygonReceiver::CopyTerrain);
}

void ConvexPolygonReceiver::CopyTerrain(
    const Context<double> &context, ConvexPolygonSet *footholds) const {
  auto foothold_set = EvalAbstractInput(context, 0)->get_value<lcmt_foothold_set>();
  *footholds = ConvexPolygonSet::CopyFromLcm(foothold_set);
}

ConvexPolygonSender::ConvexPolygonSender() {
  ConvexPolygonSet set;
  DeclareAbstractInputPort("convex_foothold_set",
                           drake::Value<ConvexPolygonSet>(set));
  DeclareAbstractOutputPort("lcmt_convex_foothold_set",
                            &ConvexPolygonSender::CopyTerrain);
}

void ConvexPolygonSender::CopyTerrain(
    const Context<double> &context, lcmt_foothold_set *footholds) const {
  auto foothold_set = EvalAbstractInput(context, 0)->get_value<ConvexPolygonSet>();
  foothold_set.CopyToLcm(footholds);
}

}