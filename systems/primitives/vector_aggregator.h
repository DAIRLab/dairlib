#pragma once

#include <string>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

#include "systems/framework/timestamped_vector.h"

namespace dairlib {
namespace systems {

/// VectorAggregator collects a list of TimeStampedVector objects
/// Assumes that the vectors come in order, and adds them to a std::vector
/// Uses the timestamp field to determine uniqueness--this will reject
/// any timestamp that exactly matches the previous timestamp. However,
/// this class does NOT check for ordering of timestmaps.
class VectorAggregator : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VectorAggregator)

  /// @param vector_length is the length of the input TimestampedVector
  VectorAggregator(int vector_length) {
    DeclareVectorInputPort(
      TimestampedVector<double>(vector_length));
    DeclarePerStepEvent<drake::systems::PublishEvent<double>>(
        drake::systems::PublishEvent<double>(
            drake::systems::Event<double>::TriggerType::kPerStep));
    vector_length_ = vector_length;
  }

  /// Return the list of received vectors in raw form
  const std::vector<Eigen::VectorXd>& get_received_vectors() const {
    return received_vectors_;
  }

  /// Return the list of received timestamps in raw form
  const std::vector<double> get_received_timestamps() const {
    return std::vector<double>(received_timestamp_.begin(),
                               received_timestamp_.end());
  }

  /// Build an Eigen vector out of the timestamps
  Eigen::VectorXd BuildTimestampVector() const {
    Eigen::VectorXd timestamp;
    timestamp = Eigen::Map<Eigen::VectorXd>(received_timestamp_.data(),
                                            received_timestamp_.size());

    return timestamp;
  }

  /// Build an Eigen Matrix out of the received vectors
  /// The ith column of the matrix is the ith received vectors
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
      dynamic_cast<const TimestampedVector<double>*>(
        EvalVectorInput(context, 0));

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
    }
  }

  int vector_length_;
  mutable std::vector<Eigen::VectorXd> received_vectors_;
  mutable std::vector<double> received_timestamp_;
};

}  // namespace systems
}  // namespace dairlib
