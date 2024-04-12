#pragma once

// dairlib
#include "geometry/convex_polygon_set.h"

// drake
#include "drake/geometry/meshcat.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"


namespace dairlib {
namespace geometry {

/*!
 * LeafSystem for visualizing foothold constraints, footstep solutions, and CoM
 * traj solutions for the alip_mpfc controller
 */
class ConvexPolygonVisualizer : public drake::systems::LeafSystem<double> {
 public:

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexPolygonVisualizer);

  /*!
   * Constructor.
   * @param meshcat Shared pointer to a meshcat instance. Cannot be NULL
   * @param plant
   */
  ConvexPolygonVisualizer(
      std::shared_ptr<drake::geometry::Meshcat> meshcat,
      double update_period_sec=0
  );

  const drake::systems::InputPort<double>& get_input_port_polygons() const {
    return this->get_input_port(polygon_input_port_);
  }

 private:
  /*!
   * @brief Returns a meshcat path for the foothold visualizations
   * @param i index of the foothold
   */
  static std::string make_polygon_path(int i) {
    return "/dairlib_convex_polygon_meshes/" + std::to_string(i);
  }

  void DrawPolygons(geometry::ConvexPolygonSet& polygons,
                     int n_prev_polygons,
                     const std::string& prefix="") const;


  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  drake::systems::InputPortIndex polygon_input_port_;
  drake::systems::DiscreteStateIndex n_polygons_idx_;
  mutable std::shared_ptr<drake::geometry::Meshcat> meshcat_;

};

}
}


