#include <stdlib.h>
#include <unistd.h>  // sleep/usleep

#include <cmath>
#include <fstream>
#include <string>

#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/evaluation_scripts/visualize_rom_dynamics_function/settings_for_rom_dyn_eval.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/system_utils.h"

namespace dairlib::goldilocks_models {

using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::MatrixX;
using drake::VectorX;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;

using systems::OutputVector;

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Read-in the parameters
  RomDynEvalSettings gains;
  const YAML::Node& root = YAML::LoadFile(FindResourceOrThrow(
      "examples/goldilocks_models/evaluation_scripts/"
      "visualize_rom_dynamics_function/settings_for_rom_dyn_eval.yaml"));
  drake::yaml::YamlReadArchive(root).Accept(&gains);

  // Create data folder if it doesn't exist
  cout << "data directory = " << gains.dir_data << endl;
  if (!CreateFolderIfNotExist(gains.dir_data, false)) return 0;

  // Build fix-spring Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_wo_springs(0.0);
  addCassieMultibody(&plant_wo_springs, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false);
  plant_wo_springs.Finalize();

  // Reduced order model
  std::unique_ptr<ReducedOrderModel> rom =
      CreateRom(gains.rom_option, 1 /*robot_option*/, plant_wo_springs, true);
  ReadModelParameters(rom.get(), gains.dir_model, gains.model_iter);

  /// Some checks
  DRAKE_DEMAND(rom->n_y() == 3);
  DRAKE_DEMAND(rom->n_tau() == 0);
  DRAKE_DEMAND(gains.x_max >= gains.x_min);
  DRAKE_DEMAND(gains.y_max >= gains.y_min);
  DRAKE_DEMAND(gains.z_max >= gains.z_min);
  DRAKE_DEMAND((gains.plot_xz_or_yz == 0) || (gains.plot_xz_or_yz == 1));

  /// Create folder and copy scripts
  if (!CreateFolderIfNotExist(gains.dir_data, false)) return 0;
  std::system(("cp "
               "examples/goldilocks_models/evaluation_scripts/"
               "visualize_rom_dynamics_function/* " +
               gains.dir_script_backup)
                  .c_str());

  /// Evaluation
  // Construct script
  VectorXd x_samples(gains.n_samples_x);
  if (gains.n_samples_x == 1) {
    DRAKE_DEMAND(gains.x_min == gains.x_max);
    x_samples(0) = gains.x_min;
  } else {
    double x_delta = (gains.x_max - gains.x_min) / (gains.n_samples_x - 1);
    for (int i = 0; i < gains.n_samples_x; i++)
      x_samples(i) = gains.x_min + i * x_delta;
  }
  VectorXd y_samples(gains.n_samples_y);
  if (gains.n_samples_y == 1) {
    DRAKE_DEMAND(gains.y_min == gains.y_max);
    y_samples(0) = gains.y_min;
  } else {
    double y_delta = (gains.y_max - gains.y_min) / (gains.n_samples_y - 1);
    for (int i = 0; i < gains.n_samples_y; i++)
      y_samples(i) = gains.y_min + i * y_delta;
  }
  VectorXd z_samples(gains.n_samples_z);
  if (gains.n_samples_z == 1) {
    DRAKE_DEMAND(gains.z_min == gains.z_max);
    z_samples(0) = gains.z_min;
  } else {
    double z_delta = (gains.z_max - gains.z_min) / (gains.n_samples_z - 1);
    for (int i = 0; i < gains.n_samples_z; i++)
      z_samples(i) = gains.z_min + i * z_delta;
  }

  // Eval dynamics function
  Vector3d y;
  Vector3d ydot;
  ydot << gains.comdot_x, gains.comdot_y, gains.comdot_z;
  Vector3d yddot;
  VectorXd tau(0);

  if (gains.plot_xz_or_yz == 0) {
    MatrixXd x_accel(gains.n_samples_z, gains.n_samples_x);
    MatrixXd y_accel(gains.n_samples_z, gains.n_samples_x);
    MatrixXd z_accel(gains.n_samples_z, gains.n_samples_x);
    for (int j = 0; j < gains.n_samples_y; j++) {
      for (int i = 0; i < gains.n_samples_x; i++) {
        for (int k = 0; k < gains.n_samples_z; k++) {
          y << x_samples(i), y_samples(j), z_samples(k);

          yddot = rom->EvalDynamicFunc(y, ydot, tau);
          if (yddot.array().isNaN().all()) {
            cout << "!!! yddot contains NaN !!!\n";
            cout << "!!! we will stop running the program and not run python "
                    "plotting script !!!\n";
            cout << "x_samples = \n" << x_samples.transpose() << endl;
            cout << "y_samples = \n" << y_samples.transpose() << endl;
            cout << "z_samples = \n" << z_samples.transpose() << endl;
            cout << "yddot = " << yddot << endl;
            DRAKE_UNREACHABLE();
          }
          x_accel(k, i) = yddot(0);
          y_accel(k, i) = yddot(1);
          z_accel(k, i) = yddot(2);
        }
      }

      // Store data
      writeCSV(gains.dir_data + string("x_accel_" + std::to_string(j) + ".csv"),
               x_accel);
      writeCSV(gains.dir_data + string("y_accel_" + std::to_string(j) + ".csv"),
               y_accel);
      writeCSV(gains.dir_data + string("z_accel_" + std::to_string(j) + ".csv"),
               z_accel);
    }
  } else if (gains.plot_xz_or_yz == 1) {
    MatrixXd x_accel(gains.n_samples_z, gains.n_samples_y);
    MatrixXd y_accel(gains.n_samples_z, gains.n_samples_y);
    MatrixXd z_accel(gains.n_samples_z, gains.n_samples_y);
    for (int i = 0; i < gains.n_samples_x; i++) {
      for (int j = 0; j < gains.n_samples_y; j++) {
        for (int k = 0; k < gains.n_samples_z; k++) {
          y << x_samples(i), y_samples(j), z_samples(k);

          yddot = rom->EvalDynamicFunc(y, ydot, tau);
          if (yddot.array().isNaN().all()) {
            cout << "!!! yddot contains NaN !!!\n";
            cout << "!!! we will stop running the program and not run python "
                    "plotting script !!!\n";
            cout << "x_samples = \n" << x_samples.transpose() << endl;
            cout << "y_samples = \n" << y_samples.transpose() << endl;
            cout << "z_samples = \n" << z_samples.transpose() << endl;
            cout << "yddot = " << yddot << endl;
            DRAKE_UNREACHABLE();
          }
          x_accel(k, j) = yddot(0);
          y_accel(k, j) = yddot(1);
          z_accel(k, j) = yddot(2);
        }
      }

      // Store data
      writeCSV(gains.dir_data + string("x_accel_" + std::to_string(i) + ".csv"),
               x_accel);
      writeCSV(gains.dir_data + string("y_accel_" + std::to_string(i) + ".csv"),
               y_accel);
      writeCSV(gains.dir_data + string("z_accel_" + std::to_string(i) + ".csv"),
               z_accel);
    }
  }

  return 0;
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::DoMain(argc, argv);
}
