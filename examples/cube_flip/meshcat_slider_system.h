#include <memory>
#include <string>

#include "drake/geometry/meshcat.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

class MeshcatSliderSystem : public drake::systems::LeafSystem<double> {
 public:
  MeshcatSliderSystem(std::shared_ptr<drake::geometry::Meshcat> meshcat,
                      const std::string& slider_name)
      : meshcat_(std::move(meshcat)), slider_name_(slider_name) {
      
    // Declare an output port of size 1 (a single double).
    this->DeclareVectorOutputPort(
        "slider_value", 
        drake::systems::BasicVector<double>(1),
        &MeshcatSliderSystem::CalcOutput,
        {this->time_ticket()});
  }

 private:
  void CalcOutput(const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) const {
    (*output)[0] = meshcat_->GetSliderValue(slider_name_);
  }

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  std::string slider_name_;
};