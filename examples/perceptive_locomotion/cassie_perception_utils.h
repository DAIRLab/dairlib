#pragma once

#include "systems/perception/perceptive_locomotion_preprocessor.h"
#include "systems/perception/elevation_mapping_system.h"

namespace dairlib {
namespace perceptive_locomotion {

std::shared_ptr<perception::PerceptiveLocomotionPreprocessor>
MakeCassieElevationMappingPreProcessor(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* plant_context);

}
}