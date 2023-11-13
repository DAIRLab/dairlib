#pragma once

#include "dairlib/lcmt_foothold_set.hpp"
#include "dairlib/lcmt_convex_decomposition_debug.hpp"
#include "geometry/convex_polygon_set.h"
#include "drake/systems/framework/leaf_system.h"
#include "convex_plane_decomposition_msgs/PlanarTerrain.h"

namespace dairlib::geometry {

class ConvexPolygonRosReceiver : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexPolygonRosReceiver);
  ConvexPolygonRosReceiver(double convexity_threshold);
  const drake::systems::OutputPort<double>& get_output_port_footholds() const {
    return this->get_output_port(foothold_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_debug() const {
    return this->get_output_port(debug_output_port_);
  }

 private:
  double convexity_threshold_;
  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;
  void CopyTerrain(const drake::systems::Context<double> &context,
                   ConvexPolygonSet *footholds) const;
  void CopyDebug(const drake::systems::Context<double>& context,
                 lcmt_convex_decomposition_debug* debug) const;

  // TODO (@Brian-Acosta) Declare these states in the constructor and move
  //  computation to the unrestricted update
  drake::systems::AbstractStateIndex foothold_state_idx_;
  drake::systems::AbstractStateIndex debug_state_idx_;

  drake::systems::OutputPortIndex foothold_output_port_;
  drake::systems::OutputPortIndex debug_output_port_;
};
}