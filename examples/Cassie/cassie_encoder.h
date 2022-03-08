#pragma once

#include <memory>

#include <drake/common/eigen_types.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

#define CASSIE_ENC_RES_LOW 8192
#define CASSIE_ENC_RES_HIGH 262144

#define DRIVE_FILTER_NB 9
#define CASSIE_JOINT_FILTER_NB 4
#define CASSIE_JOINT_FILTER_NA 3

static int drive_filter_b[DRIVE_FILTER_NB] = {2727, 534, -2658, -795, 72,
                                              110,  19,  -6,    -3};

static double joint_filter_b[CASSIE_JOINT_FILTER_NB] = {12.348, 12.348, -12.348,
                                                        -12.348};

static double joint_filter_a[CASSIE_JOINT_FILTER_NA] = {1.0, -1.7658, 0.79045};

static std::map<std::string, int> drive_encoder_resolutions = {
    {"hip_roll_left", CASSIE_ENC_RES_LOW},
    {"hip_yaw_left", CASSIE_ENC_RES_LOW},
    {"hip_pitch_left", CASSIE_ENC_RES_LOW},
    {"knee_left", CASSIE_ENC_RES_LOW},
    {"toe_left", CASSIE_ENC_RES_HIGH},
    {"hip_roll_right", CASSIE_ENC_RES_LOW},
    {"hip_yaw_right", CASSIE_ENC_RES_LOW},
    {"hip_pitch_right", CASSIE_ENC_RES_LOW},
    {"knee_right", CASSIE_ENC_RES_LOW},
    {"toe_right", CASSIE_ENC_RES_HIGH}};

static std::map<std::string, int> drive_gear_ratios = {
    {"hip_roll_left", 25},
    {"hip_yaw_left", 25},
    {"hip_pitch_left", 16},
    {"knee_left", 16},
    {"toe_left", 50},
    {"hip_roll_right", 25},
    {"hip_yaw_right", 25},
    {"hip_pitch_right", 16},
    {"knee_right", 16},
    {"toe_right", 50}};

static std::map<std::string, int> joint_encoder_resolutions = {
    {"knee_joint_left", CASSIE_ENC_RES_HIGH},
    {"ankle_joint_left", CASSIE_ENC_RES_HIGH},
    {"knee_joint_right", CASSIE_ENC_RES_HIGH},
    {"ankle_joint_right", CASSIE_ENC_RES_HIGH}};

/// Class to capture the quantization effects of Cassie's encoders
/// The resolution and velocity filter values are taken from the supplied MuJoCo
/// simulator from Agility Robotics
class CassieEncoder final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieEncoder)

  explicit CassieEncoder(const drake::multibody::MultibodyPlant<double>& plant);

  ~CassieEncoder() override = default;

 protected:
  void UpdateFilter(const drake::systems::Context<double>& context,
                    systems::BasicVector<double>* output) const;

 private:
  struct DriveFilter {
    int drive_pos_index;
    int drive_vel_index;
    int drive_encoder_resolution;  // ticks per rotation
    int gear_ratio;  // ticks per rotation
    int x[DRIVE_FILTER_NB];
  };
  struct JointFilter {
    int joint_pos_index;
    int joint_vel_index;
    int joint_encoder_resolution;  // ticks per rotation
    double x[CASSIE_JOINT_FILTER_NB];
    double y[CASSIE_JOINT_FILTER_NA];
  };

  bool is_abstract() const { return false; }

  int num_positions_;
  int num_velocities_;
  std::vector<std::shared_ptr<JointFilter>> joint_filters_;
  std::vector<std::shared_ptr<DriveFilter>> drive_filters_;
};

}  // namespace dairlib
