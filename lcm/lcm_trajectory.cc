#include "lcm/lcm_trajectory.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <fstream>
#include <memory>
#include <utility>
#include <iostream>

#include "drake/common/value.h"

using drake::AbstractValue;
using drake::systems::lcm::Serializer;
using Eigen::Dynamic;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::RowMajor;
using Eigen::VectorXd;
using std::string;
using std::unordered_map;
using std::vector;

namespace dairlib {

std::string exec(const char* cmd) {
  std::array<char, 128> buffer{};
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

LcmTrajectory::Trajectory::Trajectory(string traj_name,
                                      const lcmt_trajectory_block& traj_block) {
  int num_points = traj_block.num_points;
  int num_datatypes = traj_block.num_datatypes;
  this->traj_name = traj_name;
  this->datatypes = vector<string>(traj_block.datatypes);
  this->time_vector = VectorXd::Map(traj_block.time_vec.data(), num_points);
  this->datapoints = MatrixXd(num_datatypes, num_points);

  // Convert vector<vector<double>> to EigenMatrix
  for (int i = 0; i < num_datatypes; ++i) {
    this->datapoints.row(i) =
        VectorXd::Map(&traj_block.datapoints[i][0], num_points);
  }
}

LcmTrajectory::LcmTrajectory(const vector<Trajectory>& trajectories,
                             const vector<string>& trajectory_names,
                             const string& name, const string& description,
                             bool get_metadata)
    : trajectory_names_(trajectory_names) {
  int index = 0;
  for (const string& traj_name : trajectory_names_) {
    trajectories_[traj_name] = trajectories[index++];
  }
  if (get_metadata) {
    std::cout << "NOTE: Using subprocesses to get LcmTrajectory metadata\n";
    ConstructMetadataObject(name, description);
  }
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

lcmt_saved_traj LcmTrajectory::GenerateLcmObject() const {
  lcmt_saved_traj traj;
  traj.metadata = metadata_;
  traj.num_trajectories = trajectories_.size();

  // For each trajectory
  for (auto& traj_el : trajectories_) {
    lcmt_trajectory_block traj_block;
    const Trajectory* cpp_traj = &traj_el.second;

    traj_block.trajectory_name = cpp_traj->traj_name;
    traj_block.num_points = cpp_traj->time_vector.size();
    traj_block.num_datatypes = cpp_traj->datatypes.size();

    // Reserve space for vectors
    traj_block.time_vec.reserve(traj_block.num_points);
    traj_block.datatypes.reserve(traj_block.num_datatypes);
    traj_block.datapoints = vector<vector<double>>(
        traj_block.num_datatypes, vector<double>(traj_block.num_points));

    // Copy Eigentypes to std::vector
    traj_block.time_vec = vector<double>(
        cpp_traj->time_vector.data(),
        cpp_traj->time_vector.data() + cpp_traj->time_vector.size());
    traj_block.datatypes = vector<string>(cpp_traj->datatypes);
    for (int i = 0; i < traj_block.num_datatypes; ++i) {
      // Temporary copy due to underlying data of Eigen::Matrix
      // being column major
      VectorXd tempRow = cpp_traj->datapoints.row(i);
      memcpy(traj_block.datapoints[i].data(), tempRow.data(),
             sizeof(double) * traj_block.num_points);
    }

    traj.trajectories.push_back(traj_block);
    traj.trajectory_names.push_back(traj_el.first);
  }
  return traj;
}

void LcmTrajectory::WriteToFile(const string& filepath) {
  try {
    std::ofstream fout(filepath);
    if (!fout) {
      throw std::exception();
    }

    std::vector<uint8_t> bytes;
    drake::systems::lcm::Serializer<lcmt_saved_traj> serializer;
    serializer.Serialize(*AbstractValue::Make(GenerateLcmObject()), &bytes);

    fout.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    fout.close();
  } catch (std::exception& e) {
    std::cerr << "Could not open file: " << filepath
              << "\nException: " << e.what() << std::endl;
    throw e;
  }
}

void LcmTrajectory::LoadFromFile(const std::string& filepath) {
  std::vector<uint8_t> bytes;
  drake::systems::lcm::Serializer<lcmt_saved_traj> serializer;
  try {
    std::ifstream inFile(filepath, std::ios_base::binary);

    // Determine size of buffer
    inFile.seekg(0, std::ios_base::end);
    size_t length = inFile.tellg();
    inFile.seekg(0, std::ios_base::beg);

    bytes.reserve(length);
    std::copy(std::istreambuf_iterator<char>(inFile),
              std::istreambuf_iterator<char>(), std::back_inserter(bytes));
    inFile.close();
  } catch (std::exception& e) {
    std::cerr << "Could not open file: " << filepath
              << "\nException: " << e.what() << std::endl;
    throw e;
  }
  // Deserialization process
  lcmt_saved_traj traj;
  std::unique_ptr<AbstractValue> traj_value = AbstractValue::Make(traj);
  serializer.Deserialize(reinterpret_cast<void*>(bytes.data()),
                         static_cast<int>(bytes.size()), traj_value.get());
  traj = traj_value->get_value<lcmt_saved_traj>();

  metadata_ = traj.metadata;
  trajectories_ = unordered_map<string, Trajectory>();
  trajectory_names_ = vector<std::string>();
  for (int i = 0; i < traj.num_trajectories; ++i) {
    string traj_name = traj.trajectory_names[i];
    trajectory_names_.push_back(traj_name);
    trajectories_[traj_name] = Trajectory(traj_name, traj.trajectories[i]);
  }
}

void LcmTrajectory::AddTrajectory(const std::string& trajectory_name,
                                  const LcmTrajectory::Trajectory& trajectory) {
  DRAKE_ASSERT(trajectories_.find(trajectory_name) == trajectories_.end());
  trajectory_names_.push_back(trajectory_name);
  trajectories_[trajectory_name] = trajectory;
}

void LcmTrajectory::ConstructMetadataObject(string name, string description) {
  std::time_t t = std::time(nullptr);  // get time now

  // convert now to string form
  metadata_.datetime = asctime(std::localtime(&t));
  metadata_.git_dirty_flag = (!exec("git diff-index HEAD").empty());
  metadata_.name = name;
  metadata_.description = description;
  metadata_.git_commit_hash = exec("git rev-parse HEAD");
}

}  // namespace dairlib
