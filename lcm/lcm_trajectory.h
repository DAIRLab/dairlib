#pragma once

#include "drake/systems/lcm/serializer.h"
#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <vector>

#include "dairlib/lcmt_saved_traj.hpp"

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

  /// Writes this LcmTrajectory object to a file specified by filepath
  /// @throws std::exception along with the invalid filepath if unable to open the file
  void writeToFile(const std::string& filepath);

  /// Loads a previously saved LcmTrajectory object from the file specified by filepath
  /// @throws std::exception along with the invalid filepath if error reading/opening the file
  static lcmt_saved_traj loadFromFile(const std::string& filepath);

  const lcmt_metadata& getMetadata() const { return metadata_; }

  Trajectory getTrajectory(const std::string& trajectory_name) const {
    return trajectories_.at(trajectory_name);
  }

  const std::vector<std::string>& getTrajectoryNames() const {
    return trajectory_names_;
  }

 private:
  lcmt_saved_traj generateLcmObject() const;
  /// Constructs a lcmt_metadata object with a specified name and description
  /// Other relevant metadata details such as datatime and git status are automatically generated
  lcmt_metadata constructMetadataObject(std::string name,
                                        std::string description) const;

  lcmt_metadata metadata_;
  std::unordered_map<std::string, Trajectory> trajectories_;
  std::vector<std::string> trajectory_names_;
};

} // namespace dairlib
