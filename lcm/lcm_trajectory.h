#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "dairlib/lcmt_saved_traj.hpp"

#include "drake/systems/lcm/serializer.h"

namespace dairlib {

/// Used for saving/loading trajectories.
/// To save a LcmTrajectory object, create one or multiple
/// LcmTrajectory::Trajectory structs and wrap them using the LcmTrajectory
/// constructor. Finally call WriteToFile() with the desired relative filepath
///
/// To load a saved LcmTrajectory object, call the LoadFromFile() with relative
/// filepath of the previously saved LcmTrajectory object

class LcmTrajectory {
 public:
  /// Simple struct used for saving trajectories
  /// lcmt_trajectory_block is the lcmtype analog
  struct Trajectory {
    Trajectory() = default;
    Trajectory(std::string traj_name, const lcmt_trajectory_block& traj_block);

    std::string traj_name;
    Eigen::VectorXd time_vector;
    // Rows correspond to datatypes
    // Cols correspond to different time indices
    Eigen::MatrixXd datapoints;
    std::vector<std::string> datatypes;
  };

  LcmTrajectory() = default;
  LcmTrajectory(const std::vector<Trajectory>& trajectories,
                const std::vector<std::string>& trajectory_names,
                const std::string& name,
                const std::string& description,
                bool get_metadata=true);

  explicit LcmTrajectory(const lcmt_saved_traj& traj);

  explicit LcmTrajectory(const std::string& filepath) {
    LoadFromFile(filepath);
  }

  virtual ~LcmTrajectory() = default;

  /// Writes this LcmTrajectory object to a file specified by filepath
  /// @throws std::exception along with the invalid filepath if unable to open
  /// the file
  void WriteToFile(const std::string& filepath);

  /// Loads a previously saved LcmTrajectory object from the file specified by
  /// filepath
  /// @throws std::exception along with the invalid filepath if error
  /// reading/opening the file
  virtual void LoadFromFile(const std::string& filepath);

  lcmt_metadata GetMetadata() const { return metadata_; }

  const Trajectory& GetTrajectory(const std::string& trajectory_name) const {
    return trajectories_.at(trajectory_name);
  }

  /// Add additional LcmTrajectory::Trajectory objects
  void AddTrajectory(const std::string& trajectory_name,
                     const Trajectory& trajectory);

  /// Returns a vector of the names of the stored Trajectory objects
  const std::vector<std::string>& GetTrajectoryNames() const {
    return trajectory_names_;
  }

  lcmt_saved_traj GenerateLcmObject() const;
 protected:
  /// Constructs a lcmt_metadata object with a specified name and description
  /// Other relevant metadata details such as datatime and git status are
  /// automatically generated
  void ConstructMetadataObject(std::string name, std::string description);

 private:

  lcmt_metadata metadata_;
  std::unordered_map<std::string, Trajectory> trajectories_;
  std::vector<std::string> trajectory_names_;
};

}  // namespace dairlib
