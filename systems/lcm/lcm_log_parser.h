#include <string>

#include "drake/lcm/drake_lcm_log.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_log_playback_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "systems/framework/timestamped_vector.h"

namespace dairlib {
namespace systems {
namespace lcm {

class VectorAggregator : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VectorAggregator)

  VectorAggregator(int vector_length) {
    DeclareVectorInputPort(
      TimestampedVector<double>(vector_length));
    DeclarePerStepEvent<drake::systems::PublishEvent<double>>(
        drake::systems::PublishEvent<double>(
            drake::systems::Event<double>::TriggerType::kPerStep));
    vector_length_ = vector_length;
  }

  const std::vector<Eigen::VectorXd>& get_received_vectors() const {
    return received_vectors_;
  }

  const std::vector<double> get_received_timestamps() const {
    return std::vector<double>(received_timestamp_.begin(),
                               received_timestamp_.end());
  }

  Eigen::VectorXd BuildTimestampVector() const {
    Eigen::VectorXd timestamp;
    timestamp = Eigen::Map<Eigen::VectorXd>(received_timestamp_.data(), received_timestamp_.size());

    return timestamp;
  }

  Eigen::MatrixXd BuildMatrixFromVectors() const {
    Eigen::MatrixXd data(vector_length_, received_vectors_.size());
    for (uint i = 0; i < received_vectors_.size(); i++) {
      data.col(i) = received_vectors_[i];
    }

    return data;
  }

 private:
  void DoPublish(const drake::systems::Context<double>& context,
                 const std::vector<const drake::systems::PublishEvent<double>*>&
                     events) const override {
    const TimestampedVector<double>* input = 
        dynamic_cast<const TimestampedVector<double>*>(EvalVectorInput(context, 0));

    // const TimestampedVector<double>* input =
    //     EvalVectorInput<const TimestampedVector<double>>(context, 0);

    bool is_new_input = false;
    if (received_vectors_.empty() && input->get_timestamp() != 0)
      is_new_input = true;
    if (!received_vectors_.empty() &&
        (input->get_timestamp() != received_timestamp_.back())) {
      is_new_input = true;
    }

    if (is_new_input) {
      received_vectors_.push_back(input->CopyVectorNoTimestamp());
      received_timestamp_.push_back(input->get_timestamp());

      std::cout << "t: " << context.get_time() << std::endl;
    }
  }

  int vector_length_;
  mutable std::vector<Eigen::VectorXd> received_vectors_;
  mutable std::vector<double> received_timestamp_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace dairlib
