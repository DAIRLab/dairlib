#pragma once

#include <string>
#include <map>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace multibody {

// Simple system which outputs a MultibodyPlant's COM as a 3D pose (quat,x,y,z)
// Where the quat is always the identity and the x,y,z corresponds to the COM
// The input port (q) is a BasicVector of size plant.num_positions
// There are two output ports:
//  com_output_port_ (1, 0, 0, x, y, z) is the COM pose
//  xy_com_output_port_ (1, 0, 0, 0, y, z) is the COM pose, projected to z=0
class ComPoseSystem : public drake::systems::LeafSystem<double> {
 public:
  explicit ComPoseSystem(const drake::multibody::MultibodyPlant<double>& plant);

  const drake::systems::OutputPort<double>& get_com_output_port() const {
    return get_output_port(com_output_port_);
  }

  const drake::systems::OutputPort<double>& get_xy_com_output_port() const {
    return get_output_port(xy_com_output_port_);
  }

 private:
  void OutputCom(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const;
  void OutputXyCom(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  int com_output_port_;
  int xy_com_output_port_;
  int position_input_port_;
};

}  // namespace multibody
}  // namespace dairlib
