// We model the belt as a series of rigid links connected by linear spring
// damper systems.
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/linear_spring_damper.h>
#include <drake/multibody/parsing/parser.h>
#include "common/find_resource.h"

void ConstructBeltModel(drake::multibody::MultibodyPlant<double>* plant,
                        drake::geometry::SceneGraph<double>* scene_graph) {
    drake::multibody::Parser parser(plant, scene_graph);

    drake::multibody::ModelInstanceIndex end_point_1 = parser.AddModels(
        dairlib::FindResourceOrThrow("examples/belt_assembly/urdf/point_mass.urdf"))[0];
    drake::multibody::ModelInstanceIndex end_point_2 = parser.AddModels(
        dairlib::FindResourceOrThrow("examples/belt_assembly/urdf/point_mass.urdf"))[0];

    // Add a linear spring damper system between the two point masses
    plant->AddForceElement<drake::multibody::LinearSpringDamper<double>>(
}
