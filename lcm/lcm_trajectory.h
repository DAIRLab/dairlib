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
/// constructor. Finally call writeToFile() with the desired relative filepath
///
/// To load a saved LcmTrajectory object, call the loadFromFile() with relative
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
                const std::string& name, const std::string& description);

  explicit LcmTrajectory(const lcmt_saved_traj& traj);

  explicit LcmTrajectory(const std::string& filepath) {
    loadFromFile(filepath);
  }

  /// Writes this LcmTrajectory object to a file specified by filepath
  /// @throws std::exception along with the invalid filepath if unable to open
  /// the file
  virtual void writeToFile(const std::string& filepath);

  /// Loads a previously saved LcmTrajectory object from the file specified by
  /// filepath
  /// @throws std::exception along with the invalid filepath if error
  /// reading/opening the file
  virtual void loadFromFile(const std::string& filepath);

  lcmt_metadata getMetadata() const { return metadata_; }

  Trajectory getTrajectory(const std::string& trajectory_name) const {
    return trajectories_.at(trajectory_name);
  }

  /// Add additional LcmTrajectory::Trajectory objects
  void addTrajectory(const std::string& trajectory_name,
                     const Trajectory& trajectory);

  const std::vector<std::string>& getTrajectoryNames() const {
    return trajectory_names_;
  }

 protected:
  /// Constructs a lcmt_metadata object with a specified name and description
  /// Other relevant metadata details such as datatime and git status are
  /// automatically generated
  void constructMetadataObject(std::string name, std::string description);

 private:
  lcmt_saved_traj generateLcmObject() const;

  lcmt_metadata metadata_;
  std::unordered_map<std::string, Trajectory> trajectories_;
  std::vector<std::string> trajectory_names_;
};

}  // namespace dairlib
