#include "multibody/com_pose_system.h"

namespace dairlib {
namespace multibody {

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::multibody::MultibodyPlant;

ComPoseSystem::ComPoseSystem(
    const MultibodyPlant<double>& plant) : plant_(plant) {
  com_output_port_  = this->DeclareVectorOutputPort(BasicVector<double>(7),
      &ComPoseSystem::OutputCom).get_index();
  xy_com_output_port_  = this->DeclareVectorOutputPort(BasicVector<double>(7),
      &ComPoseSystem::OutputXyCom).get_index();
  position_input_port_ = this->DeclareVectorInputPort(BasicVector<double>(
      plant_.num_positions())).get_index();
}

void ComPoseSystem::OutputCom(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto q = this->EvalVectorInput(context, position_input_port_);

  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositions(plant_context.get(), q->get_value());
  auto com = plant_.CalcCenterOfMassPosition(*plant_context);
  auto pose = output->get_mutable_value();
  pose << 1, 0, 0, 0, com;
}

void ComPoseSystem::OutputXyCom(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto q = this->EvalVectorInput(context, position_input_port_);

  auto plant_context = plant_.CreateDefaultContext();
  plant_.SetPositions(plant_context.get(), q->get_value());
  auto com = plant_.CalcCenterOfMassPosition(*plant_context);
  auto pose = output->get_mutable_value();
  pose << 1, 0, 0, 0, com.head(2), 0;
}


}  // namespace multibody
}  // namespace dairlib
