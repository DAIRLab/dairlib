// STL includes
#include <iostream>

// dair includes
#include "convex_foothold_lcm_systems.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::RowVector3d;
using drake::systems::Context;

namespace dairlib::geometry {


ConvexFootholdReceiver::ConvexFootholdReceiver() {
  DeclareAbstractInputPort("lcmt_convex_foothold_set",
                           drake::Value<lcmt_foothold_set>());
  DeclareAbstractOutputPort("convex_foothold_set",
                            &ConvexFootholdReceiver::CopyTerrain);
}

void ConvexFootholdReceiver::CopyTerrain(
    const Context<double> &context, ConvexFootholdSet *footholds) const {
  auto foothold_set = EvalAbstractInput(context, 0)->get_value<lcmt_foothold_set>();
  *footholds = ConvexFootholdSet::CopyFromLcm(foothold_set);
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
  auto foothold_set = EvalAbstractInput(context, 0)->get_value<ConvexFootholdSet>();
  foothold_set.CopyToLcm(footholds);
}

}