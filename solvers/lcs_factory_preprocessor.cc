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


vector<SortedPair<GeometryId>> LCSFactoryPreProcessor::PreProcessor(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const vector<vector<SortedPair<GeometryId>>>& contact_geoms, 
    int num_friction_directions, int num_contacts, bool find_closest_contacts,
    bool verbose) {

    int n_contacts = num_contacts;
    // If is_resolving_contacts is true, return closest contacts
    if(find_closest_contacts) {
        std::vector<SortedPair<GeometryId>> closest_contacts;
        // Reserve space for the closest contacts
        closest_contacts.reserve(n_contacts);

        for (int i = 0; i < n_contacts; i++) {
            if (contact_geoms[i].size() == 1) {
                closest_contacts.push_back(contact_geoms[i][0]);
                if(verbose) {
                    const auto& query_port = plant.get_geometry_query_input_port();
                    const auto& query_object =
                        query_port.template Eval<drake::geometry::QueryObject<double>>(context);
                    const auto& inspector = query_object.inspector();
                    SortedPair<GeometryId> verbose_test_pair = contact_geoms.at(i).at(0);
                    multibody::GeomGeomCollider collider(plant, verbose_test_pair);
                    auto [phi_i, J_i] = collider.EvalPolytope(context, num_friction_directions);
                    std::cout << "Contact pair "<< i <<" : (" << inspector.GetName(verbose_test_pair.first()) 
                                << ", " << inspector.GetName(verbose_test_pair.second())
                                << ") with phi = " << phi_i << std::endl;
                }
            } 
            else {
                if(verbose) {
                    std::cout << "Contact pair "<< i <<" : choosing between:" << std::endl;
                }
                // Find the closest pair if there are multiple pairs
                std::vector<double> distances;

                for (int j = 0; j < contact_geoms[i].size(); j++) {
                    // Evaluate the distance for each pair
                    SortedPair<GeometryId> pair {(contact_geoms.at(i)).at(j)};
                    multibody::GeomGeomCollider collider(plant, pair);

                    auto [phi_i, J_i] = collider.EvalPolytope(context, num_friction_directions);
    
                    distances.push_back(phi_i);

                    if(verbose) {
                        const auto& query_port = plant.get_geometry_query_input_port();
                        const auto& query_object =
                            query_port.template Eval<drake::geometry::QueryObject<double>>(context);
                        const auto& inspector = query_object.inspector();
                        SortedPair<GeometryId> verbose_test_pair = contact_geoms.at(i).at(j);
                        std::cout << "   " << j << "(" << inspector.GetName(verbose_test_pair.first()) 
                                    << ", " << inspector.GetName(verbose_test_pair.second())
                                    << ") with phi = " << phi_i << std::endl;
                    }
                }
                // Pick minimum distance contact pair
                auto min_distance_it = std::min_element(distances.begin(), distances.end());
                int min_distance_index = std::distance(distances.begin(), min_distance_it);
                closest_contacts.push_back(contact_geoms[i][min_distance_index]);

                if(verbose) {
                    std::cout << "   --> Chose option " << min_distance_index << std::endl;
                }
            }
        }
        return closest_contacts;
    }
    // If is_resolving_contacts is false, return all contacts
    else {
        std::vector<SortedPair<GeometryId>> all_contacts;
        // Reserve space for the closest contacts
        all_contacts.reserve(n_contacts);
        for (std::vector<SortedPair<GeometryId>> contact_pair : contact_geoms) {
            for (SortedPair<GeometryId> pair : contact_pair) {
                all_contacts.push_back(pair);

                if(verbose) {
                    const auto& query_port = plant.get_geometry_query_input_port();
                    const auto& query_object =
                        query_port.template Eval<drake::geometry::QueryObject<double>>(context);
                    const auto& inspector = query_object.inspector();
                    multibody::GeomGeomCollider collider(plant, pair);
                    auto [phi_i, J_i] = collider.EvalPolytope(context, num_friction_directions);
                    std::cout << "Contact pair: (" << inspector.GetName(pair.first()) 
                                << ", " << inspector.GetName(pair.second())
                                << ") with phi = " << phi_i << std::endl;
                }
            }
        }
        return all_contacts;
    }
}

}   // namespace solvers
}  // namespace dairlib