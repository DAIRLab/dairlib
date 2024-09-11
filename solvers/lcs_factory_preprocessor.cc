#include "solvers/lcs_factory_preprocessor.h"
#include <iostream>

#include "multibody/geom_geom_collider.h"

namespace dairlib {
namespace solvers {

using std::vector;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::math::RigidTransform;
using drake::geometry::FrameId;
using drake::systems::Context;
using drake::geometry::SignedDistancePair;


vector<SortedPair<GeometryId>> LCSFactoryPreProcessor::PreProcessor(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const vector<vector<SortedPair<GeometryId>>>& contact_geoms,
    const vector<int>& resolve_contacts_to_list,
    int num_friction_directions, int num_contacts,
    bool verbose) {

    int n_contacts = num_contacts;
    // If is_resolving_contacts is true, return closest contacts
    std::vector<SortedPair<GeometryId>> closest_contacts;
    // Reserve space for the closest contacts
    closest_contacts.reserve(n_contacts);

    for (int i = 0; i < contact_geoms.size(); i++) {
        DRAKE_ASSERT(contact_geoms[i].size() >= resolve_contacts_to_list[i]);
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

                const SignedDistancePair<double> signed_distance_pair =
                    query_object.ComputeSignedDistancePairClosestPoints(
                        verbose_test_pair.first(), verbose_test_pair.second());

                const Eigen::Vector3d& p_ACa =
                    inspector.GetPoseInFrame(verbose_test_pair.first()).template cast<double>() *
                    signed_distance_pair.p_ACa;
                const Eigen::Vector3d& p_BCb =
                    inspector.GetPoseInFrame(verbose_test_pair.second()).template cast<double>() *
                    signed_distance_pair.p_BCb;

                // Represent the witness points as points in world frame.
                RigidTransform T_body1_contact = RigidTransform(p_ACa);
                const FrameId f1_id = inspector.GetFrameId(verbose_test_pair.first());
                const Body<double>* body1 = plant.GetBodyFromFrameId(f1_id);
                RigidTransform T_world_body1 = body1->EvalPoseInWorld(context);
                Eigen::Vector3d p_world_contact_a = T_world_body1*T_body1_contact.translation();

                RigidTransform T_body2_contact = RigidTransform(p_BCb);
                const FrameId f2_id = inspector.GetFrameId(verbose_test_pair.second());
                const Body<double>* body2 = plant.GetBodyFromFrameId(f2_id);
                RigidTransform T_world_body2 = body2->EvalPoseInWorld(context);
                Eigen::Vector3d p_world_contact_b = T_world_body2*T_body2_contact.translation();

                std::cout << "Contact pair "<< i <<" : (" << inspector.GetName(verbose_test_pair.first()) 
                            << ", " << inspector.GetName(verbose_test_pair.second())
                            << ") with phi = " << phi_i << " between world points ["
                            << p_world_contact_a.transpose() << "], ["
                            << p_world_contact_b.transpose() << "]" << std::endl;
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

                    // Get the witness points on each geometry.
                    const SignedDistancePair<double> signed_distance_pair =
                        query_object.ComputeSignedDistancePairClosestPoints(
                            verbose_test_pair.first(), verbose_test_pair.second());

                    const Eigen::Vector3d& p_ACa =
                        inspector.GetPoseInFrame(verbose_test_pair.first()).template cast<double>() *
                        signed_distance_pair.p_ACa;
                    const Eigen::Vector3d& p_BCb =
                        inspector.GetPoseInFrame(verbose_test_pair.second()).template cast<double>() *
                        signed_distance_pair.p_BCb;

                    // Represent the witness points as points in world frame.
                    RigidTransform T_body1_contact = RigidTransform(p_ACa);
                    const FrameId f1_id = inspector.GetFrameId(verbose_test_pair.first());
                    const Body<double>* body1 = plant.GetBodyFromFrameId(f1_id);
                    RigidTransform T_world_body1 = body1->EvalPoseInWorld(context);
                    Eigen::Vector3d p_world_contact_a = T_world_body1*T_body1_contact.translation();

                    RigidTransform T_body2_contact = RigidTransform(p_BCb);
                    const FrameId f2_id = inspector.GetFrameId(verbose_test_pair.second());
                    const Body<double>* body2 = plant.GetBodyFromFrameId(f2_id);
                    RigidTransform T_world_body2 = body2->EvalPoseInWorld(context);
                    Eigen::Vector3d p_world_contact_b = T_world_body2*T_body2_contact.translation();

                    std::cout << "   " << j << "(" << inspector.GetName(verbose_test_pair.first()) 
                                << ", " << inspector.GetName(verbose_test_pair.second())
                                << ") with phi = " << phi_i << " between world points ["
                                << p_world_contact_a.transpose() << "], ["
                                << p_world_contact_b.transpose() << "]" << std::endl;
                }
            }
            // Pick the resolve_contacts_to_list[i] minimum distance contact pairs.
            for (int j = 0; j < resolve_contacts_to_list[i]; j++) {
                // Pick minimum distance contact pair
                auto min_distance_it = std::min_element(distances.begin(), distances.end());
                int min_distance_index = std::distance(distances.begin(), min_distance_it);
                closest_contacts.push_back(contact_geoms[i][min_distance_index]);
                distances[min_distance_index] = 100;

                if(verbose) {
                    std::cout << "   --> Chose option " << min_distance_index << std::endl;
                }

            }
        }
    }
    DRAKE_ASSERT(closest_contacts.size() == n_contacts);
    return closest_contacts;
}

}   // namespace solvers
}  // namespace dairlib