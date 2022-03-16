#include "examples/goldilocks_models/goldilocks_utils.h"

#include <sys/stat.h>  // Check the existence of a file/folder

#include <cstdlib>  // System call to create folder (and also parent directory)
#include <fstream>
#include <iostream>
#include <regex>

#include "drake/multibody/parsing/parser.h"

using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::shared_ptr;
using std::string;
using std::to_string;
using std::vector;

namespace dairlib {
namespace goldilocks_models {

SubQpData::SubQpData(int N_sample) {
  // Initialize all the member
  for (int i = 0; i < N_sample; i++) {
    w_sol_vec.push_back(std::make_shared<VectorXd>());
    H_vec.push_back(std::make_shared<MatrixXd>());
    b_vec.push_back(std::make_shared<VectorXd>());
    b_main_vec.push_back(std::make_shared<VectorXd>());
    c_vec.push_back(std::make_shared<VectorXd>());
    A_vec.push_back(std::make_shared<MatrixXd>());
    lb_vec.push_back(std::make_shared<VectorXd>());
    ub_vec.push_back(std::make_shared<VectorXd>());
    y_vec.push_back(std::make_shared<VectorXd>());
    B_vec.push_back(std::make_shared<MatrixXd>());
    is_success_vec.push_back(std::make_shared<double>());

    A_active_vec.push_back(std::make_shared<MatrixXd>());
    B_active_vec.push_back(std::make_shared<MatrixXd>());
    nw_vec.push_back(std::make_shared<int>());
    nl_vec.push_back(std::make_shared<int>());
    P_vec.push_back(std::make_shared<MatrixXd>());
    q_vec.push_back(std::make_shared<VectorXd>());
  }
}

void CreateMBP(MultibodyPlant<double>* plant, int robot_option) {
  if (robot_option == 0) {
    Parser parser(plant);
    string full_name = FindResourceOrThrow(
        "examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
    parser.AddModelFromFile(full_name);
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"),
                      drake::math::RigidTransform<double>());
    plant->Finalize();

  } else if (robot_option == 1) {
    addCassieMultibody(plant, nullptr, true,
                       "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                       false);
    plant->Finalize();
  } else {
    throw std::runtime_error("robot_option " + to_string(robot_option) +
                             "is not implemented");
  }
}

void CreateMBPForVisualization(MultibodyPlant<double>* plant,
                               drake::geometry::SceneGraph<double>* scene_graph,
                               Eigen::Vector3d ground_normal,
                               int robot_option) {
  if (robot_option == 0) {
    multibody::addFlatTerrain(plant, scene_graph, 0.8, 0.8, ground_normal);
    Parser parser(plant, scene_graph);
    string full_name = FindResourceOrThrow(
        "examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
    parser.AddModelFromFile(full_name);
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"),
                      drake::math::RigidTransform<double>());
    plant->Finalize();

  } else if (robot_option == 1) {
    multibody::addFlatTerrain(plant, scene_graph, 0.8, 0.8, ground_normal);
    Parser parser(plant, scene_graph);
    addCassieMultibody(plant, scene_graph, true,
                       "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                       false);
    plant->Finalize();
  } else {
    throw std::runtime_error("robot_option " + to_string(robot_option) +
                             "is not implemented");
  }
}

// Note:
// * Whenever you add a new model, remember to update the model number in
//     1. goldilocks_model_traj_opt.cc
//     2. cassie_rom_planner_system.cc (where we called SetInitialGuess)
// * See google sheet for the ROM list
std::unique_ptr<ReducedOrderModel> CreateRom(
    int rom_option, int robot_option,
    const drake::multibody::MultibodyPlant<double>& plant, bool print_info) {
  // TODO: add unit test to make sure the ROM settings are correct (it is
  //  possible that I create bugs while editing.)

  /// Basis for mapping function (dependent on the robot)
  vector<int> empty_inds = {};
  std::unique_ptr<MonomialFeatures> mapping_basis;
  if (robot_option == 0) {
    mapping_basis = std::make_unique<MonomialFeatures>(
        2, plant.num_positions(), empty_inds, "mapping basis");
  } else {  // robot_option == 1
    // TODO: we completely remove the quaternion for now. We want to have
    //  roll and pitch, so add this component later (need to map quat to roll
    //  pitch yaw)
    // Note that we shouldn't use pelvis z because it drifts in state estimation
    vector<int> skip_inds = {0, 1, 2, 3, 4, 5};  // quaternion, x, and y
    //    vector<int> skip_inds = {3, 4, 5};  // quaternion, x, and y
    std::map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
    DRAKE_DEMAND(pos_map.at("base_qw") == 0);  // Assumption
    if (((rom_option >= 10) && (rom_option <= 14)) ||
        ((rom_option >= 22) && (rom_option <= 24))) {
      // Also skip the right leg joints (swing leg)
      cout << "Joint skipped in mapping function: ";
      for (auto& pair : pos_map) {
        if (pair.first.find("right") != std::string::npos) {
          cout << pair.first << ", ";
          skip_inds.push_back(pair.second);
        }
      }
      cout << endl;
    }
    if ((rom_option >= 15) && (rom_option <= 24)) {
      // skip pelvis z
      skip_inds.push_back(6);
    }
    if ((0 <= rom_option && rom_option <= 6) || (rom_option == 8) ||
        (rom_option == 9) || ((rom_option >= 10) && (rom_option <= 13)) ||
        rom_option == 17 || rom_option == 18 || rom_option == 21 ||
        rom_option == 22 || rom_option == 23 || rom_option == 24) {
      // Second order
      mapping_basis = std::make_unique<MonomialFeatures>(
          2, plant.num_positions(), skip_inds, "mapping basis");
    } else if (rom_option == 7) {
      // Zeroth order
      mapping_basis = std::make_unique<MonomialFeatures>(
          0, plant.num_positions(), skip_inds, "mapping basis");
    } else if (rom_option == 14 || rom_option == 15 || rom_option == 16 ||
               rom_option == 20) {
      // Fourth order
      mapping_basis = std::make_unique<MonomialFeatures>(
          4, plant.num_positions(), skip_inds, "mapping basis");
    } else if (rom_option == 19) {
      // sixth order
      mapping_basis = std::make_unique<MonomialFeatures>(
          6, plant.num_positions(), skip_inds, "mapping basis");
    } else {
      DRAKE_UNREACHABLE();
    }
  }

  if (print_info) {
    mapping_basis->PrintInfo();
  }

  /// Basis for dynamic function
  std::unique_ptr<MonomialFeatures> dynamic_basis;
  if (rom_option == 0) {
    dynamic_basis = std::make_unique<MonomialFeatures>(
        2, 2 * Lipm::kDimension(2), empty_inds, "dynamic basis");
  } else if (rom_option == 1) {
    dynamic_basis = std::make_unique<MonomialFeatures>(
        2, 2 * LipmWithSwingFoot::kDimension(2), empty_inds, "dynamic basis");
  } else if (rom_option == 2) {
    dynamic_basis = std::make_unique<MonomialFeatures>(
        2, 2 * FixHeightAccel::kDimension, empty_inds, "dynamic basis");
  } else if (rom_option == 3) {
    dynamic_basis = std::make_unique<MonomialFeatures>(
        2, 2 * FixHeightAccelWithSwingFoot::kDimension, empty_inds,
        "dynamic basis");
  } else if (rom_option == 4 || rom_option == 7 ||
             ((rom_option >= 9) && (rom_option <= 11)) || rom_option == 17 ||
             rom_option == 18 || rom_option == 21 || rom_option == 22 ||
             rom_option == 23 || rom_option == 24) {
    // Highest degree = 2
    dynamic_basis = std::make_unique<MonomialFeatures>(
        2, 2 * Lipm::kDimension(3), empty_inds, "dynamic basis");
  } else if (rom_option == 5 || rom_option == 6) {
    dynamic_basis = std::make_unique<MonomialFeatures>(
        2, 2 * LipmWithSwingFoot::kDimension(3), empty_inds, "dynamic basis");
  } else if (rom_option == 8) {
    dynamic_basis = std::make_unique<MonomialFeatures>(
        2, 2 * Lipm::kDimension(3) + 1, empty_inds, "dynamic basis");
  } else if ((rom_option >= 12) && (rom_option <= 16)) {
    // Highest degree = 4
    dynamic_basis = std::make_unique<MonomialFeatures>(
        4, 2 * Lipm::kDimension(3), empty_inds, "dynamic basis");
  } else if (rom_option == 19 || rom_option == 20) {
    // Highest degree = 6
    dynamic_basis = std::make_unique<MonomialFeatures>(
        6, 2 * Lipm::kDimension(3), empty_inds, "dynamic basis");
  } else {
    throw std::runtime_error("Not implemented");
  }
  if (print_info) {
    dynamic_basis->PrintInfo();
  }

  /// Contact frames and position for mapping function
  string stance_foot_body_name;
  Vector3d stance_foot_contact_point_pos;
  string swing_foot_body_name;
  Vector3d swing_foot_contact_point_pos;
  if (robot_option == 0) {
    stance_foot_body_name = "left_lower_leg_mass";
    stance_foot_contact_point_pos = Vector3d(0, 0, -0.5);
    swing_foot_body_name = "right_lower_leg_mass";
    swing_foot_contact_point_pos = Vector3d(0, 0, -0.5);
  } else {  // robot_option == 1
    auto left_toe = LeftToeFront(plant);
    auto left_heel = LeftToeRear(plant);
    stance_foot_body_name = "toe_left";
    stance_foot_contact_point_pos = (left_toe.first + left_heel.first) / 2;
    swing_foot_body_name = "toe_right";
    swing_foot_contact_point_pos = (left_toe.first + left_heel.first) / 2;
  }
  auto stance_foot = std::pair<const Vector3d, const Frame<double>&>(
      stance_foot_contact_point_pos,
      plant.GetFrameByName(stance_foot_body_name));
  auto swing_foot = std::pair<const Vector3d, const Frame<double>&>(
      swing_foot_contact_point_pos, plant.GetFrameByName(swing_foot_body_name));

  /// Construct reduced-order model
  std::unique_ptr<ReducedOrderModel> rom;
  if (rom_option == 0) {
    rom = std::make_unique<Lipm>(plant, stance_foot, *mapping_basis,
                                 *dynamic_basis, 2);
  } else if (rom_option == 1) {
    rom = std::make_unique<LipmWithSwingFoot>(
        plant, stance_foot, swing_foot, *mapping_basis, *dynamic_basis, 2);
  } else if (rom_option == 2) {
    rom = std::make_unique<FixHeightAccel>(plant, stance_foot, *mapping_basis,
                                           *dynamic_basis);
  } else if (rom_option == 3) {
    rom = std::make_unique<FixHeightAccelWithSwingFoot>(
        plant, stance_foot, swing_foot, *mapping_basis, *dynamic_basis);
  } else if (rom_option == 4 || rom_option == 18) {
    rom = std::make_unique<Lipm>(plant, stance_foot, *mapping_basis,
                                 *dynamic_basis, 3);
  } else if (rom_option == 5) {
    rom = std::make_unique<LipmWithSwingFoot>(
        plant, stance_foot, swing_foot, *mapping_basis, *dynamic_basis, 3);
  } else if (rom_option == 6) {
    // Fix the mapping function of the COM
    std::set<int> invariant_idx = {0, 1, 2};
    rom = std::make_unique<LipmWithSwingFoot>(plant, stance_foot, swing_foot,
                                              *mapping_basis, *dynamic_basis, 3,
                                              invariant_idx);
  } else if (rom_option == 7) {
    // Fix the mapping function of the COM
    // TODO: rom_option=7 is unfinished. Should we use COM wrt world?
    DRAKE_UNREACHABLE();
    std::set<int> invariant_idx = {0, 1, 2};
    rom = std::make_unique<Lipm>(plant, stance_foot, *mapping_basis,
                                 *dynamic_basis, 3, invariant_idx);
  } else if (rom_option == 8) {
    // 3D Gip (general inverted pendulum)
    rom = std::make_unique<Gip>(plant, stance_foot, *mapping_basis,
                                *dynamic_basis, 3);
  } else if (rom_option == 9 || rom_option == 16 || rom_option == 17 ||
             rom_option == 23) {
    // Fix the mapping function of the COM xy part
    std::set<int> invariant_idx = {0, 1};
    rom = std::make_unique<Lipm>(plant, stance_foot, *mapping_basis,
                                 *dynamic_basis, 3, invariant_idx);
  } else if (rom_option == 10 || rom_option == 12 || rom_option == 14 ||
             rom_option == 15 || rom_option == 19 || rom_option == 20 ||
             rom_option == 21 || rom_option == 23) {
    // Fix the mapping function of the COM xy part
    // Use pelvis as a proxy for COM
    std::set<int> invariant_idx = {0, 1};
    rom = std::make_unique<Lipm>(plant, stance_foot, *mapping_basis,
                                 *dynamic_basis, 3, invariant_idx, true);
  } else if (rom_option == 11 || rom_option == 13 || rom_option == 22) {
    // Fix the mapping function of LIPM
    // Use pelvis as a proxy for COM
    // ROM is only a function of stance leg's joints
    std::set<int> invariant_idx = {0, 1, 2};
    rom = std::make_unique<Lipm>(plant, stance_foot, *mapping_basis,
                                 *dynamic_basis, 3, invariant_idx, true);
  } else {
    throw std::runtime_error("Not implemented");
  }
  if (print_info) {
    rom->PrintInfo();
  }

  return rom;
}

void ReadModelParameters(ReducedOrderModel* rom, const std::string& dir,
                         int model_iter) {
  // Check that we are using the correct model
  DRAKE_DEMAND(rom->n_y() == readCSV(dir + string("rom_n_y.csv"))(0, 0));
  DRAKE_DEMAND(rom->n_tau() == readCSV(dir + string("rom_n_tau.csv"))(0, 0));
  DRAKE_DEMAND(rom->n_feature_y() ==
               readCSV(dir + string("rom_n_feature_y.csv"))(0, 0));
  DRAKE_DEMAND(rom->n_feature_yddot() ==
               readCSV(dir + string("rom_n_feature_yddot.csv"))(0, 0));
  if (rom->n_tau() != 0) {
    DRAKE_DEMAND((rom->B() - readCSV(dir + string("rom_B.csv"))).norm() == 0);
  }

  cout << "reading model parameter "
       << dir + to_string(model_iter) + "_theta_y.csv\n\n";

  // Update the ROM parameters from file
  VectorXd theta_y =
      readCSV(dir + to_string(model_iter) + string("_theta_y.csv")).col(0);
  VectorXd theta_yddot =
      readCSV(dir + to_string(model_iter) + string("_theta_yddot.csv")).col(0);
  rom->SetThetaY(theta_y);
  rom->SetThetaYddot(theta_yddot);
}

std::map<int, int> MirrorPosIndexMap(
    const drake::multibody::MultibodyPlant<double>& plant, int robot_option) {
  std::map<int, int> ret;
  std::map<std::string, int> pos_map = multibody::makeNameToPositionsMap(plant);

  if (robot_option == 0) {
    vector<std::pair<string, string>> l_r_pairs{
        std::pair<string, string>("left_", "right_"),
        std::pair<string, string>("right_", "left_")};
    std::vector<std::string> joint_names = {"hip_pin", "knee_pin"};

    for (const auto& joint_name : joint_names) {
      for (const auto& l_r_pair : l_r_pairs) {
        ret[pos_map.at(l_r_pair.first + joint_name)] =
            pos_map.at(l_r_pair.second + joint_name);
      }
    }
  } else {
    vector<std::pair<string, string>> l_r_pairs{
        std::pair<string, string>("_left", "_right"),
        std::pair<string, string>("_right", "_left")};
    std::vector<std::string> joint_names = {
        "hip_roll", "hip_yaw", "hip_pitch", "knee", "ankle_joint", "toe"};

    for (const auto& joint_name : joint_names) {
      for (const auto& l_r_pair : l_r_pairs) {
        ret[pos_map.at(joint_name + l_r_pair.first)] =
            pos_map.at(joint_name + l_r_pair.second);
      }
    }
  }

  return ret;
}
std::set<int> MirrorPosSignChangeSet(
    const drake::multibody::MultibodyPlant<double>& plant, int robot_option) {
  std::set<int> ret;
  std::map<std::string, int> pos_map = multibody::makeNameToPositionsMap(plant);

  if (robot_option == 0) {
    // No sign change
  } else {
    std::vector<std::string> asy_joint_names = {
        "base_qx",       "base_qz",       "base_y",      "hip_roll_right",
        "hip_yaw_right", "hip_roll_left", "hip_yaw_left"};
    // Below also work, because the sign of qw is the rotation direction.
    //        std::vector<std::string> asy_joint_names = {
    //            "base_qw",       "base_qy",       "base_y", "hip_roll_right",
    //            "hip_yaw_right", "hip_roll_left", "hip_yaw_left"};

    for (const auto& name : asy_joint_names) {
      ret.insert(pos_map.at(name));
    }
  }

  return ret;
}
std::map<int, int> MirrorVelIndexMap(
    const drake::multibody::MultibodyPlant<double>& plant, int robot_option) {
  std::map<int, int> ret;
  std::map<std::string, int> vel_map =
      multibody::makeNameToVelocitiesMap(plant);

  if (robot_option == 0) {
    vector<std::pair<string, string>> l_r_pairs{
        std::pair<string, string>("left_", "right_"),
        std::pair<string, string>("right_", "left_")};
    std::vector<std::string> joint_names = {"hip_pin", "knee_pin"};

    for (const auto& joint_name : joint_names) {
      for (const auto& l_r_pair : l_r_pairs) {
        ret[vel_map.at(l_r_pair.first + joint_name + "dot")] =
            vel_map.at(l_r_pair.second + joint_name + "dot");
      }
    }
  } else {
    vector<std::pair<string, string>> l_r_pairs{
        std::pair<string, string>("_left", "_right"),
        std::pair<string, string>("_right", "_left")};
    std::vector<std::string> joint_names = {
        "hip_roll", "hip_yaw", "hip_pitch", "knee", "ankle_joint", "toe"};

    for (const auto& joint_name : joint_names) {
      for (const auto& l_r_pair : l_r_pairs) {
        ret[vel_map.at(joint_name + l_r_pair.first + "dot")] =
            vel_map.at(joint_name + l_r_pair.second + "dot");
      }
    }
  }

  return ret;
}
std::set<int> MirrorVelSignChangeSet(
    const drake::multibody::MultibodyPlant<double>& plant, int robot_option) {
  std::set<int> ret;
  std::map<std::string, int> vel_map =
      multibody::makeNameToVelocitiesMap(plant);

  if (robot_option == 0) {
    // No sign change
  } else {
    std::vector<std::string> asy_joint_names = {
        "base_wx",           "base_wz",          "base_vy",
        "hip_roll_rightdot", "hip_yaw_rightdot", "hip_roll_leftdot",
        "hip_yaw_leftdot"};
    // We still need to obey the right hand rule for rotation, so below is
    // incorrect.
    //    std::vector<std::string> asy_joint_names = {
    //        "base_wy",          "base_vy",          "hip_roll_rightdot",
    //        "hip_yaw_rightdot", "hip_roll_leftdot", "hip_yaw_leftdot"};

    for (const auto& name : asy_joint_names) {
      ret.insert(vel_map.at(name));
    }
  }

  return ret;
}

// Create time knots for creating cubic splines
vector<double> createTimeKnotsGivenTimesteps(const vector<VectorXd>& h_vec) {
  vector<double> T_breakpoint;
  double time = 0;
  T_breakpoint.push_back(time);
  for (unsigned int i = 0; i < h_vec.size(); i++) {
    time += h_vec[i](0);
    T_breakpoint.push_back(time);
  }
  return T_breakpoint;
}

PiecewisePolynomial<double> CreateCubicSplineGivenYAndYdot(
    const vector<VectorXd>& h_vec, const vector<VectorXd>& s_vec,
    const vector<VectorXd>& ds_vec) {
  // Create time knots
  vector<double> T_breakpoint = createTimeKnotsGivenTimesteps(h_vec);

  // Create traj value and its derivatives (convert VectorXd to MatrixXd)
  vector<MatrixXd> s(T_breakpoint.size(), MatrixXd::Zero(1, 1));
  vector<MatrixXd> s_dot(T_breakpoint.size(), MatrixXd::Zero(1, 1));
  for (unsigned int i = 0; i < s_vec.size(); i++) {
    s[i] = s_vec[i];
    s_dot[i] = ds_vec[i];
  }

  // Construct splines
  return PiecewisePolynomial<double>::CubicHermite(T_breakpoint, s, s_dot);
}

void StoreSplineOfY(const vector<VectorXd>& h_vec,
                    const PiecewisePolynomial<double>& y_spline,
                    const string& directory, const string& prefix) {
  // parameters
  int n_sample_each_seg = 2;

  // setup
  int n_y = y_spline.value(0).rows();

  // Create time knots
  vector<double> T_breakpoint = createTimeKnotsGivenTimesteps(h_vec);

  // Create the matrix for csv file
  // The first row is time, and the rest rows are y
  MatrixXd t_and_y(1 + n_y, 1 + (n_sample_each_seg - 1) * h_vec.size());
  MatrixXd t_and_ydot(1 + n_y, 1 + (n_sample_each_seg - 1) * h_vec.size());
  MatrixXd t_and_yddot(1 + n_y, 1 + (n_sample_each_seg - 1) * h_vec.size());
  t_and_y(0, 0) = 0;
  t_and_ydot(0, 0) = 0;
  t_and_yddot(0, 0) = 0;
  t_and_y.block(1, 0, n_y, 1) = y_spline.value(0);
  t_and_ydot.block(1, 0, n_y, 1) = y_spline.derivative(1).value(0);
  t_and_yddot.block(1, 0, n_y, 1) = y_spline.derivative(2).value(0);
  for (unsigned int i = 0; i < h_vec.size(); i++) {
    for (int j = 1; j < n_sample_each_seg; j++) {
      double time = T_breakpoint[i] + j * h_vec[i](0) / (n_sample_each_seg - 1);
      t_and_y(0, j + i * (n_sample_each_seg - 1)) = time;
      t_and_ydot(0, j + i * (n_sample_each_seg - 1)) = time;
      t_and_yddot(0, j + i * (n_sample_each_seg - 1)) = time;
      t_and_y.block(1, j + i * (n_sample_each_seg - 1), n_y, 1) =
          y_spline.value(time);
      t_and_ydot.block(1, j + i * (n_sample_each_seg - 1), n_y, 1) =
          y_spline.derivative(1).value(time);
      t_and_yddot.block(1, j + i * (n_sample_each_seg - 1), n_y, 1) =
          y_spline.derivative(2).value(time);
    }
  }

  // Store into csv file
  writeCSV(directory + prefix + string("t_and_y.csv"), t_and_y);
  writeCSV(directory + prefix + string("t_and_ydot.csv"), t_and_ydot);
  writeCSV(directory + prefix + string("t_and_yddot.csv"), t_and_yddot);
}

void CheckSplineOfY(const vector<VectorXd>& h_vec,
                    const vector<VectorXd>& yddot_vec,
                    const PiecewisePolynomial<double>& y_spline) {
  // parameters
  double tol = 1e-4;

  // Create time knots
  vector<double> T_breakpoint = createTimeKnotsGivenTimesteps(h_vec);

  // Compare
  for (unsigned int i = 0; i < T_breakpoint.size(); i++) {
    VectorXd yddot_by_drake = y_spline.derivative(2).value(T_breakpoint[i]);
    VectorXd yddot_by_hand = yddot_vec[i];
    DRAKE_DEMAND((yddot_by_drake - yddot_by_hand).norm() < tol);
  }
}

void storeTau(const vector<VectorXd>& h_vec, const vector<VectorXd>& tau_vec,
              const string& directory, const string& prefix) {
  // setup
  int n_tau = tau_vec[0].rows();

  // Create time knots
  vector<double> T_breakpoint = createTimeKnotsGivenTimesteps(h_vec);

  // Create the matrix for csv file
  // The first row is time, and the rest rows are tau
  MatrixXd t_and_tau(1 + n_tau, tau_vec.size());
  for (unsigned int i = 0; i < T_breakpoint.size(); i++) {
    t_and_tau(0, i) = T_breakpoint[i];
    t_and_tau.block(1, i, n_tau, 1) = tau_vec[i];
  }

  // Store into csv file
  writeCSV(directory + prefix + string("t_and_tau.csv"), t_and_tau);
}

VectorXd createPrimeNumbers(int num_prime) {
  DRAKE_DEMAND(num_prime <= 25);

  VectorXd prime_until_100(25);
  prime_until_100 << 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53,
      59, 61, 67, 71, 73, 79, 83, 89, 97;
  return prime_until_100.head(num_prime);
}

bool file_exist(const std::string& name) {
  struct stat buffer;
  // cout << name << " exist? " << (stat (name.c_str(), &buffer) == 0) << endl;
  return (stat(name.c_str(), &buffer) == 0);
}

bool folder_exist(const std::string& pathname_string, bool print) {
  // Convert string to char
  const char* pathname = pathname_string.c_str();

  struct stat info;
  if (stat(pathname, &info) != 0) {
    if (print) printf("cannot access %s\n", pathname);
    return false;
  } else if (info.st_mode & S_IFDIR) {
    if (print) printf("%s is a directory\n", pathname);
    return true;
  } else {
    if (print) printf("%s is no directory\n", pathname);
    return false;
  }
}

bool CreateFolderIfNotExist(const string& dir, bool ask_for_permission) {
  if (!folder_exist(dir)) {
    if (ask_for_permission) {
      cout << dir
           << " doesn't exsit. We will create the folder.\nProceed? (Y/N)\n";
      char answer[1];
      std::cin >> answer;
      if (!((answer[0] == 'Y') || (answer[0] == 'y'))) {
        cout << "Ending the program.\n";
        return false;
      }
    }

    // Creating a directory
    // This method probably only works in Linux/Unix?
    // See:
    // https://codeyarns.com/2014/08/07/how-to-create-directory-using-c-on-linux/
    // It will create parent directories as well.
    std::string string_for_system_call = "mkdir -p " + dir;
    if (system(string_for_system_call.c_str()) == -1) {
      printf("Error creating directory!n");
      return false;
    } else {
      cout << "This folder has been created: " << dir << endl;
    }
  }
  return true;
}

vector<std::string> ParseCsvToStringVec(const std::string& file_name,
                                        bool is_column_vector) {
  // Read file into a std::string
  std::ifstream ifs(file_name);
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      (std::istreambuf_iterator<char>()));

  // parse
  vector<std::string> ret;
  std::stringstream ss(content);
  std::string item;
  char delimiter = (is_column_vector) ? '\n' : ',';
  while (std::getline(ss, item, delimiter)) {
    // cout << item << endl;
    ret.push_back(item);
  }
  return ret;
}

void SaveStringVecToCsv(const vector<std::string>& strings,
                        const std::string& file_name) {
  std::ofstream ofile;
  ofile.open(file_name, std::ofstream::out);
  for (auto& mem : strings) {
    ofile << mem << endl;
  }
  ofile.close();
}

BodyPoint FiveLinkRobotLeftContact(const MultibodyPlant<double>& plant) {
  return BodyPoint(Vector3d(0, 0, -0.5),
                   plant.GetFrameByName("left_lower_leg_mass"));
}
BodyPoint FiveLinkRobotRightContact(const MultibodyPlant<double>& plant) {
  return BodyPoint(Vector3d(0, 0, -0.5),
                   plant.GetFrameByName("right_lower_leg_mass"));
}

void PlannerSetting::PrintAll() const {
  cout << "rom_option" << rom_option << endl;
  cout << "iter" << iter << endl;
  cout << "sample" << sample << endl;
  cout << "n_step" << n_step << endl;
  cout << "knots_per_mode" << knots_per_mode << endl;
  cout << "zero_touchdown_impact" << zero_touchdown_impact << endl;
  cout << "use_double_contact_points" << use_double_contact_points << endl;
  cout << "equalize_timestep_size" << equalize_timestep_size << endl;
  cout << "fix_duration" << fix_duration << endl;
  cout << "feas_tol" << feas_tol << endl;
  cout << "opt_tol" << opt_tol << endl;
  cout << "max_iter" << max_iter << endl;
  cout << "use_ipopt" << use_ipopt << endl;
  cout << "log_solver_info" << log_solver_info << endl;
  cout << "time_limit" << time_limit << endl;
  cout << "w_Q" << gains.w_Q << endl;
  cout << "w_R" << gains.w_R << endl;
  cout << "w_rom_reg" << gains.w_rom_reg << endl;
  cout << "w_reg_quat_" << gains.w_reg_quat << endl;
  cout << "w_reg_xy_" << gains.w_reg_xy << endl;
  cout << "w_reg_z_" << gains.w_reg_z << endl;
  cout << "w_reg_joints_" << gains.w_reg_joints << endl;
  cout << "w_reg_hip_yaw_" << gains.w_reg_hip_yaw << endl;
  cout << "dir_model" << dir_model << endl;
  cout << "dir_data" << dir_data << endl;
  cout << "init_file" << init_file << endl;
  cout << "solve_idx_for_read_from_file" << solve_idx_for_read_from_file
       << endl;
}

}  // namespace goldilocks_models
}  // namespace dairlib
