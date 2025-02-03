#include "walking_reference_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::systems::Context;

WalkingReferenceSystem::WalkingReferenceSystem(
    const drake::multibody::MultibodyPlant<double> &plant,
    Context<double> *plant_context, const GaitParams &params) :
    plant_(plant), plant_context_(plant_context), params_(params) {

}


}