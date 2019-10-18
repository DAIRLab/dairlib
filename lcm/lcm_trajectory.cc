#include <chrono>
#include <algorithm>
#include <memory>
#include <utility>
#include <cstring>
#include <fstream>

#include "drake/common/value.h"
#include "lcm/lcm_trajectory.h"

using drake::AbstractValue;
using std::vector;
using std::string;
using std::unordered_map;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::RowMajor;
using Eigen::Map;
using drake::systems::lcm::Serializer;

namespace dairlib {

LcmTrajectory::Trajectory::Trajectory(string traj_name,
                                      const lcmt_trajectory_block& traj_block) {
  int num_points = traj_block.num_points;
  int num_datatypes = traj_block.num_datatypes;
  this->traj_name = traj_name;
  this->datatypes = vector<string>(traj_block.datatypes);
  this->time_vector = VectorXd::Map(traj_block.time_vec.data(),
                                    num_points);
  this->datapoints = MatrixXd(num_points, num_datatypes);

  // Convert vector<vector<double>> to EigenMatrix
  for (int i = 0; i < num_points; ++i) {
    this->datapoints.row(i) = VectorXd::Map(&traj_block.datapoints[i][0],
                                            num_datatypes);
  }
}

LcmTrajectory::LcmTrajectory(const vector<Trajectory>& trajectories,
                             const vector<string>& trajectory_names,
                             const string& name,
                             const string& description):
  trajectory_names_(trajectory_names) {
  int index = 0;
  for (string name : trajectory_names_) {
    trajectories_[name] = trajectories[index++];
  }
  metadata_ = constructMetadataObject(name, description);
}

LcmTrajectory::LcmTrajectory(const lcmt_saved_traj& traj) {
  metadata_ = traj.metadata;
  trajectories_ = unordered_map<string, Trajectory>();
  trajectory_names_ = vector<std::string>();
  for (int i = 0; i < traj.num_trajectories; ++i) {
    string traj_name = traj.trajectory_names[i];
    trajectory_names_.push_back(traj_name);
    trajectories_[traj_name] = Trajectory(traj_name, traj.trajectories[i]);
  }
}

lcmt_saved_traj LcmTrajectory::generateLcmObject() const {
  lcmt_saved_traj traj;
  traj.metadata = metadata_;
  traj.num_trajectories = trajectories_.size();

  // For each trajectory
  for (auto & traj_el : trajectories_) {
    lcmt_trajectory_block traj_block;
    const Trajectory* cpp_traj = &traj_el.second;

    traj_block.trajectory_name = cpp_traj->traj_name;
    traj_block.num_points = cpp_traj->time_vector.size();
    traj_block.num_datatypes = cpp_traj->datatypes.size();

    // Reserve space for vectors
    traj_block.time_vec.reserve(traj_block.num_points);
    traj_block.datatypes.reserve(traj_block.num_datatypes);
    traj_block.datapoints = vector<vector<double>>(traj_block.num_points,
                            vector<double>(traj_block.num_datatypes));

    // Copy Eigentypes to std::vector
    traj_block.time_vec = vector<double>(cpp_traj->time_vector.data(),
                                         cpp_traj->time_vector.data() +
                                         cpp_traj->time_vector.size());
    traj_block.datatypes = vector<string>(cpp_traj->datatypes);
    for (int i = 0; i < traj_block.num_points; ++i) {
      memcpy(traj_block.datapoints[i].data(),
             cpp_traj->datapoints.data() + i * traj_block.num_datatypes,
             sizeof(double) * traj_block.num_datatypes);
    }

    traj.trajectories.push_back(traj_block);
    traj.trajectory_names.push_back(traj_el.first);
  }
  return traj;
}

void LcmTrajectory::writeToFile(string filepath) {
  std::vector<uint8_t> bytes;
  drake::systems::lcm::Serializer<lcmt_saved_traj> serializer;
  serializer.Serialize(*AbstractValue::Make(generateLcmObject()), &bytes);
  std::ofstream fout(filepath);
  fout.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
  // fout << bytes.data();
  fout.close();
}

lcmt_saved_traj LcmTrajectory::loadFromFile(const std::string filepath) {
  drake::systems::lcm::Serializer<lcmt_saved_traj> serializer;
  std::ifstream inFile(filepath, std::ios_base::binary);

  // Determine size of buffer
  inFile.seekg(0, std::ios_base::end);
  size_t length = inFile.tellg();
  inFile.seekg(0, std::ios_base::beg);

  std::vector<uint8_t> bytes;
  bytes.reserve(length);
  std::copy(std::istreambuf_iterator<char>(inFile),
            std::istreambuf_iterator<char>(),
            std::back_inserter(bytes) );

  // Deserialization process
  lcmt_saved_traj traj;
  std::unique_ptr<AbstractValue> traj_value = AbstractValue::Make(traj);
  serializer.Deserialize(reinterpret_cast<void*>(bytes.data()),
                         static_cast<int>(bytes.size()),
                         traj_value.get());
  return traj_value->get_value<lcmt_saved_traj>();
}

lcmt_metadata LcmTrajectory::constructMetadataObject(string name,
    string description) const {
  lcmt_metadata metadata;

  metadata.datetime = "";
  metadata.git_dirty_flag = false;
  metadata.name = name;
  metadata.description = description;
  // TODO(yangwill): autofill git details
  return metadata;
}

}  // namespace dairlib
