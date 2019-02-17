#include "examples/Cassie/networking/cassie_input_translator.h"
#include "examples/Cassie/datatypes/cassie_user_in_t.h"
#include "examples/Cassie/datatypes/cassie_names.h"

namespace dairlib {
namespace systems {

using std::string;
using Eigen::VectorXd;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::multibody::JointActuatorIndex;
using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;
using systems::OutputVector;

namespace {
  constexpr int kNumActuators = 10;
}

CassieInputTranslator::CassieInputTranslator(
    const MultibodyPlant<double>& plant) {
  std::map<std::string, int> actuatorIndexMap =
    multibody::makeNameToActuatorsMap(plant);
  // build index map
  DRAKE_ASSERT(kNumActuators == cassieEffortNames.size());
  for (auto const& name : cassieEffortNames) {
    userin_to_uvector_index_.push_back(actuatorIndexMap.at(name));
  }
}



void CassieInputTranslator::Output(const Context<double>& context,
    cassie_user_in_t* cassie_in) const {
  const TimestampedVector<double>* command = (TimestampedVector<double>*)
      this->EvalVectorInput(context, 0);
  for (int i = 0; i < kNumActuators; i++) {
    cassie_in->torque[i] = command->GetAtIndex(userin_to_uvector_index_.at(i));
  }
}

}  // namespace systems
}  // namespace dairlib
