
#include <iostream>

#include "multibody/geom_geom_collider.h"

namespace dairlib {
namespace solvers {

class LCSFactoryPreProcessor {
public:
    // This class takes in a vector of vector of SortedPair<GeometryId> 
    // and resolves any non single pairs to find the closest contact pair 
    // to pass onto the LCS Factory.

    /// @param contact_geoms A vector of vector of SortedPair<GeometryId>

static std::vector<drake::SortedPair<drake::geometry::GeometryId>> PreProcessor(const drake::multibody::MultibodyPlant<double>& plant,
        const drake::systems::Context<double>& context, const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_pairs, 
        int num_friction_directions);
};

}   // namespace solvers
}  // namespace dairlib