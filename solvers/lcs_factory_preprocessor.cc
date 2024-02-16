#include "solvers/lcs_factory_preprocessor.h"
#include <iostream>

#include "multibody/geom_geom_collider.h"

namespace dairlib {
namespace solvers {

using std::vector;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;


vector<SortedPair<GeometryId>> LCSFactoryPreProcessor::PreProcessor(const MultibodyPlant<double>& plant,
    const Context<double>& context, const vector<vector<SortedPair<GeometryId>>>& contact_geoms, 
    int num_friction_directions) {
    
    int n_contacts = contact_geoms.size();
    std::vector<SortedPair<GeometryId>> closest_contacts;
    // Reserve space for the closest contacts
    closest_contacts.reserve(n_contacts);

    for (int i = 0; i < n_contacts; i++) {
        if (contact_geoms[i].size() == 1) {
            closest_contacts.push_back(contact_geoms[i][0]);
        } 
        else {
            // Find the closest pair if there are multiple pairs
            std::vector<double> distances;

            for (int j = 0; j < contact_geoms[i].size(); j++) {
                // Evaluate the distance for each pair
                SortedPair<GeometryId> pair {(contact_geoms.at(i)).at(j)};
                multibody::GeomGeomCollider collider(plant, pair);

                auto [phi_i, J_i] = collider.EvalPolytope(context, num_friction_directions);
 
                distances.push_back(phi_i);
            }
            // Pick minimum distance contact pair
            auto min_distance_it = std::min_element(distances.begin(), distances.end());
            int min_distance_index = std::distance(distances.begin(), min_distance_it);
            closest_contacts.push_back(contact_geoms[i][min_distance_index]);
        }
    }
    return closest_contacts;
}

}   // namespace solvers
}  // namespace dairlib