#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <drake/multibody/plant/multibody_plant.h>
#include "systems/framework/timestamped_vector.h"

#include "common/find_resource.h"
#include "dairlib/lcmt_radio_out.hpp"

#include "examples/cube_flip/trifinger/parameter_headers/trifinger_controller_params.h"
#include "examples/cube_flip/parameter_headers/franka_plate_controller_params.h"
#include "examples/cube_flip/parameter_headers/iC3_options.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using drake::geometry::GeometryId;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::AutoDiffXd;
using drake::systems::BasicVector;
using dairlib::systems::TimestampedVector;

namespace dairlib {

// Outputs iC3 plan index based on tracking options and context time

class iC3TimingSystem : public drake::systems::LeafSystem<double> {
  public:

    explicit iC3TimingSystem(iC3Options ic3_options, FrankaPlateControllerParams franka_controller_params);
    explicit iC3TimingSystem(iC3Options ic3_options, TrifingerControllerParams trifinger_controller_params);

    const drake::systems::InputPort<double>& get_input_port_radio() const {
      return this->get_input_port(radio_port_);
    }
    const drake::systems::OutputPort<double>& get_output_port_index() const {
      return this->get_output_port(index_output_port_);
    }



  private:
    void OutputIndex(const Context<double>& context,
                  BasicVector<double>* index) const;

    drake::systems::EventStatus SetFirstCallTime(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

    drake::systems::InputPortIndex radio_port_;
    drake::systems::OutputPortIndex index_output_port_;

    double t0_idx_;
    mutable bool called_;

    iC3Options ic3_options_;
    TrifingerControllerParams trifinger_controller_params_;
    FrankaPlateControllerParams franka_controller_params_;

    // example idx 0 = plate
    // example idx 1 = trifinger
    int example_idx_;

};

}  // namespace dairlib
