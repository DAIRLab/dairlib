#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include "attic/multibody/multibody_solvers.h"
#include "attic/multibody/rigidbody_utils.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib {
namespace multibody {
namespace {

using std::cout;
using std::endl;
using std::unique_ptr;
using std::shared_ptr;
using std::make_unique;
using std::make_shared;
using std::map;
using std::string;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::AutoDiffXd;
using drake::VectorX;
using drake::MatrixX;
using drake::math::initializeAutoDiff;
using drake::math::DiscardGradient;
using drake::multibody::joints::FloatingBaseType;
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::joints::kQuaternion;
using drake::multibody::joints::kFixed;
using drake::multibody::AddFlatTerrainToWorld;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using dairlib::buildCassieTree;
using dairlib::multibody::ContactInfo;
using dairlib::multibody::ContactToolkit;
using dairlib::multibody::GetBodyIndexFromName;
using dairlib::multibody::PositionConstraint;
using dairlib::multibody::FixedPointConstraint;
using dairlib::multibody::ContactConstraint;
using dairlib::multibody::PositionSolver;
using dairlib::multibody::FixedPointSolver;
using dairlib::multibody::ContactSolver;

class MultibodySolversTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setting the random seed
    std::srand((unsigned int)time(0));

    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf";
    buildCassieTree(tree_rpy_, filename, kRollPitchYaw);
    buildCassieTree(tree_quaternion_, filename, kQuaternion);
    buildCassieTree(tree_fixed_, filename, kFixed);

    // Adding the ground
    AddFlatTerrainToWorld(&tree_rpy_, 100, 0.2);
    AddFlatTerrainToWorld(&tree_quaternion_, 100, 0.2);

    num_positions_fixed_ = tree_fixed_.get_num_positions();
    num_velocities_fixed_ = tree_fixed_.get_num_velocities();
    num_states_fixed_ = num_positions_fixed_ + num_velocities_fixed_;
    num_efforts_fixed_ = tree_fixed_.get_num_actuators();

    num_positions_rpy_ = tree_rpy_.get_num_positions();
    num_velocities_rpy_ = tree_rpy_.get_num_velocities();
    num_states_rpy_ = num_positions_rpy_ + num_velocities_rpy_;
    num_efforts_rpy_ = tree_rpy_.get_num_actuators();

    num_positions_quaternion_ = tree_quaternion_.get_num_positions();
    num_velocities_quaternion_ = tree_quaternion_.get_num_velocities();
    num_states_quaternion_ =
        num_positions_quaternion_ + num_velocities_quaternion_;
    num_efforts_quaternion_ = tree_quaternion_.get_num_actuators();

    // Setting the initial Cassie joint angles
    position_map_fixed_ = tree_fixed_.computePositionNameToIndexMap();
    x0_fixed_ = VectorXd::Zero(num_states_fixed_);
    x0_fixed_(position_map_fixed_.at("hip_roll_left")) = 0.1;
    x0_fixed_(position_map_fixed_.at("hip_roll_right")) = -0.1;
    x0_fixed_(position_map_fixed_.at("hip_yaw_left")) = 0.01;
    x0_fixed_(position_map_fixed_.at("hip_yaw_right")) = -0.01;
    x0_fixed_(position_map_fixed_.at("hip_pitch_left")) = .269;
    x0_fixed_(position_map_fixed_.at("hip_pitch_right")) = .269;
    x0_fixed_(position_map_fixed_.at("knee_left")) = -.744;
    x0_fixed_(position_map_fixed_.at("knee_right")) = -.744;
    x0_fixed_(position_map_fixed_.at("ankle_joint_left")) = .81;
    x0_fixed_(position_map_fixed_.at("ankle_joint_right")) = .81;
    x0_fixed_(position_map_fixed_.at("toe_left")) = 0;
    x0_fixed_(position_map_fixed_.at("toe_right")) = 0;

    position_map_rpy_ = tree_rpy_.computePositionNameToIndexMap();
    x0_rpy_ = VectorXd::Zero(num_states_rpy_);
    x0_rpy_(position_map_rpy_.at("base_z")) = 3;
    x0_rpy_(position_map_rpy_.at("hip_roll_left")) = 0.1;
    x0_rpy_(position_map_rpy_.at("hip_roll_right")) = -0.1;
    x0_rpy_(position_map_rpy_.at("hip_yaw_left")) = 0.01;
    x0_rpy_(position_map_rpy_.at("hip_yaw_right")) = -0.01;
    x0_rpy_(position_map_rpy_.at("hip_pitch_left")) = .269;
    x0_rpy_(position_map_rpy_.at("hip_pitch_right")) = .269;
    x0_rpy_(position_map_rpy_.at("knee_left")) = -.744;
    x0_rpy_(position_map_rpy_.at("knee_right")) = -.744;
    x0_rpy_(position_map_rpy_.at("ankle_joint_left")) = .81;
    x0_rpy_(position_map_rpy_.at("ankle_joint_right")) = .81;
    x0_rpy_(position_map_rpy_.at("toe_left")) = -60.0 * M_PI / 180.0;
    x0_rpy_(position_map_rpy_.at("toe_right")) = -60.0 * M_PI / 180.0;

    position_map_quaternion_ = tree_quaternion_.computePositionNameToIndexMap();
    x0_quaternion_ = VectorXd::Zero(num_states_quaternion_);
    x0_quaternion_(position_map_quaternion_.at("base_z")) = 3;
    x0_quaternion_(position_map_quaternion_.at("base_qw")) = 1.0;
    x0_quaternion_(position_map_quaternion_.at("hip_roll_left")) = 0.1;
    x0_quaternion_(position_map_quaternion_.at("hip_roll_right")) = -0.1;
    x0_quaternion_(position_map_quaternion_.at("hip_yaw_left")) = 0.01;
    x0_quaternion_(position_map_quaternion_.at("hip_yaw_right")) = -0.01;
    x0_quaternion_(position_map_quaternion_.at("hip_pitch_left")) = .269;
    x0_quaternion_(position_map_quaternion_.at("hip_pitch_right")) = .269;
    x0_quaternion_(position_map_quaternion_.at("knee_left")) = -.744;
    x0_quaternion_(position_map_quaternion_.at("knee_right")) = -.744;
    x0_quaternion_(position_map_quaternion_.at("ankle_joint_left")) = .81;
    x0_quaternion_(position_map_quaternion_.at("ankle_joint_right")) = .81;
    x0_quaternion_(position_map_quaternion_.at("toe_left")) =
        -60.0 * M_PI / 180.0;
    x0_quaternion_(position_map_quaternion_.at("toe_right")) =
        -60.0 * M_PI / 180.0;

    // Setting up fixed_joints_vector_ and fixed_joints_map_ for the rpy
    // floating base model
    fixed_joints_vector_rpy_.push_back(position_map_rpy_.at("base_roll"));
    //fixed_joints_vector_rpy_.push_back(position_map_rpy_.at("base_pitch"));
    fixed_joints_vector_rpy_.push_back(position_map_rpy_.at("base_yaw"));
    fixed_joints_vector_rpy_.push_back(position_map_rpy_.at("hip_pitch_left"));
    fixed_joints_vector_rpy_.push_back(position_map_rpy_.at("hip_pitch_right"));

    // Setting up fixed_joints_vector_ and fixed_joints_map_ for the quaternion
    // floating base model
    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("base_qw"));
    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("base_qx"));
    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("base_qy"));
    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("base_qz"));
    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("hip_pitch_left"));
    fixed_joints_vector_quaternion_.push_back(
        position_map_quaternion_.at("hip_pitch_right"));

    // Setting up the fixed joints map
    for (auto& ind : fixed_joints_vector_rpy_) {
      fixed_joints_map_rpy_[ind] = x0_rpy_(ind);
    }

    for (auto& ind : fixed_joints_vector_quaternion_) {
      fixed_joints_map_quaternion_[ind] = x0_quaternion_(ind);
    }

    // Collison detect
    // Contact information for the rpy and quaternion models
    VectorXd phi_total_rpy;
    Matrix3Xd normal_total_rpy, xA_total_rpy, xB_total_rpy;
    vector<int> idxA_total_rpy, idxB_total_rpy;
    KinematicsCache<double> k_cache_rpy = tree_rpy_.doKinematics(
        x0_rpy_.head(num_positions_rpy_), x0_rpy_.tail(num_velocities_rpy_));

    VectorXd phi_total_quaternion;
    Matrix3Xd normal_total_quaternion, xA_total_quaternion, xB_total_quaternion;
    vector<int> idxA_total_quaternion, idxB_total_quaternion;
    KinematicsCache<double> k_cache_quaternion = tree_quaternion_.doKinematics(
        x0_quaternion_.head(num_positions_quaternion_),
        x0_quaternion_.tail(num_velocities_quaternion_));

    tree_rpy_.collisionDetect(k_cache_rpy, phi_total_rpy, normal_total_rpy,
                              xA_total_rpy, xB_total_rpy, idxA_total_rpy,
                              idxB_total_rpy);

    tree_quaternion_.collisionDetect(
        k_cache_quaternion, phi_total_quaternion, normal_total_quaternion,
        xA_total_quaternion, xB_total_quaternion, idxA_total_quaternion,
        idxB_total_quaternion);

    const int world_ind_rpy = GetBodyIndexFromName(tree_rpy_, "world");
    const int toe_left_ind_rpy = GetBodyIndexFromName(tree_rpy_, "toe_left");
    const int toe_right_ind_rpy = GetBodyIndexFromName(tree_rpy_, "toe_right");

    const int world_ind_quaternion =
        GetBodyIndexFromName(tree_quaternion_, "world");
    const int toe_left_ind_quaternion =
        GetBodyIndexFromName(tree_quaternion_, "toe_left");
    const int toe_right_ind_quaternion =
        GetBodyIndexFromName(tree_quaternion_, "toe_right");

    // Extracting information into the four contacts for the floating base types
    VectorXd phi_rpy(4);
    Matrix3Xd normal_rpy(3, 4), xA_rpy(3, 4), xB_rpy(3, 4);
    vector<int> idxA_rpy(4), idxB_rpy(4);

    VectorXd phi_quaternion(4);
    Matrix3Xd normal_quaternion(3, 4), xA_quaternion(3, 4), xB_quaternion(3, 4);
    vector<int> idxA_quaternion(4), idxB_quaternion(4);

    int k = 0;
    for (unsigned i = 0; i < idxA_total_rpy.size(); ++i) {
      int ind_a = idxA_total_rpy.at(i);
      int ind_b = idxB_total_rpy.at(i);
      if ((ind_a == world_ind_rpy && ind_b == toe_left_ind_rpy) ||
          (ind_a == world_ind_rpy && ind_b == toe_right_ind_rpy) ||
          (ind_a == toe_left_ind_rpy && ind_b == world_ind_rpy) ||
          (ind_a == toe_right_ind_rpy && ind_b == world_ind_rpy)) {
        xA_rpy.col(k) = xA_total_rpy.col(i);
        xB_rpy.col(k) = xB_total_rpy.col(i);
        idxA_rpy.at(k) = idxA_total_rpy.at(i);
        idxB_rpy.at(k) = idxB_total_rpy.at(i);
        ++k;
      }
    }

    k = 0;
    for (unsigned i = 0; i < idxA_total_quaternion.size(); ++i) {
      int ind_a = idxA_total_quaternion.at(i);
      int ind_b = idxB_total_quaternion.at(i);
      if ((ind_a == world_ind_quaternion && ind_b == toe_left_ind_quaternion) ||
          (ind_a == world_ind_quaternion &&
           ind_b == toe_right_ind_quaternion) ||
          (ind_a == toe_left_ind_quaternion && ind_b == world_ind_quaternion) ||
          (ind_a == toe_right_ind_quaternion &&
           ind_b == world_ind_quaternion)) {
        xA_quaternion.col(k) = xA_total_quaternion.col(i);
        xB_quaternion.col(k) = xB_total_quaternion.col(i);
        idxA_quaternion.at(k) = idxA_total_quaternion.at(i);
        idxB_quaternion.at(k) = idxB_total_quaternion.at(i);
        ++k;
      }
    }

    // Creating the contact info for both the floating base types
    // idxB is the vector index for the body which is accessed through
    // ContactInfo.idxA
    // In this case xA corresponds to the points on the ground and hence xA
    // and
    // xB must be interchanged when constructing contact_info_
    contact_info_rpy_ = {xB_rpy, idxB_rpy};
    contact_info_quaternion_ = {xB_quaternion, idxB_quaternion};

    num_position_constraints_fixed_ = tree_fixed_.getNumPositionConstraints();

    num_contacts_rpy_ = contact_info_rpy_.num_contacts;
    num_position_constraints_rpy_ = tree_rpy_.getNumPositionConstraints();
    num_forces_rpy_ = num_position_constraints_rpy_ + num_contacts_rpy_ * 3;

    num_contacts_quaternion_ = contact_info_quaternion_.num_contacts;
    num_position_constraints_quaternion_ =
        tree_quaternion_.getNumPositionConstraints();
    num_forces_quaternion_ =
        num_position_constraints_quaternion_ + num_contacts_quaternion_ * 3;

    q_fixed_ = x0_fixed_.head(num_positions_fixed_);
    u_fixed_ = VectorXd::Zero(num_efforts_fixed_);
    lambda_fixed_ = VectorXd::Zero(num_position_constraints_fixed_);

    q_rpy_ = x0_rpy_.head(num_positions_rpy_);
    u_rpy_ = VectorXd::Zero(num_efforts_rpy_);
    lambda_rpy_ = VectorXd::Zero(num_forces_rpy_);

    q_quaternion_ = x0_quaternion_.head(num_positions_quaternion_);
    u_quaternion_ = VectorXd::Zero(num_efforts_quaternion_);
    lambda_quaternion_ = VectorXd::Zero(num_forces_quaternion_);
  }

  RigidBodyTree<double> tree_fixed_;
  RigidBodyTree<double> tree_rpy_;
  RigidBodyTree<double> tree_quaternion_;
  map<string, int> position_map_fixed_;
  map<string, int> position_map_rpy_;
  map<string, int> position_map_quaternion_;
  vector<int> fixed_joints_vector_rpy_;
  vector<int> fixed_joints_vector_quaternion_;
  map<int, double> fixed_joints_map_rpy_;
  map<int, double> fixed_joints_map_quaternion_;
  int num_positions_fixed_;
  int num_positions_rpy_;
  int num_positions_quaternion_;
  int num_velocities_fixed_;
  int num_velocities_rpy_;
  int num_velocities_quaternion_;
  int num_states_fixed_;
  int num_states_rpy_;
  int num_states_quaternion_;
  int num_efforts_fixed_;
  int num_efforts_rpy_;
  int num_efforts_quaternion_;
  int num_contacts_rpy_;
  int num_contacts_quaternion_;
  int num_position_constraints_fixed_;
  int num_position_constraints_rpy_;
  int num_position_constraints_quaternion_;
  int num_forces_rpy_;
  int num_forces_quaternion_;
  ContactInfo contact_info_rpy_;
  ContactInfo contact_info_quaternion_;
  VectorXd x0_fixed_;
  VectorXd x0_rpy_;
  VectorXd x0_quaternion_;
  VectorXd q_fixed_;
  VectorXd q_rpy_;
  VectorXd q_quaternion_;
  VectorXd u_fixed_;
  VectorXd u_rpy_;
  VectorXd u_quaternion_;
  VectorXd lambda_fixed_;
  VectorXd lambda_rpy_;
  VectorXd lambda_quaternion_;
};

TEST_F(MultibodySolversTest, TestPositionConstraintInitialization) {
  // Constraint class initializations
  // Position constraint

  PositionConstraint position_constraint_fixed(tree_fixed_);
  PositionConstraint position_constraint_rpy(tree_rpy_);
  PositionConstraint position_constraint_quaternion(tree_quaternion_);
}

TEST_F(MultibodySolversTest, TestContactConstraintInitialization) {
  // Contact constraint for both floating bases
  ContactConstraint contact_constraint_rpy(tree_rpy_, contact_info_rpy_);
  ContactConstraint contact_constraint_quaternion(tree_quaternion_,
                                                  contact_info_quaternion_);
}

TEST_F(MultibodySolversTest, TestFixedPointConstraintInitialization) {
  // Fixed point constraint with and without contact
  FixedPointConstraint fp_constraint_fixed(tree_fixed_);
  FixedPointConstraint fp_constraint_rpy(tree_rpy_, contact_info_rpy_);
  FixedPointConstraint fp_constraint_quaternion(tree_quaternion_,
                                                contact_info_quaternion_);
}

TEST_F(MultibodySolversTest, TestPositionSolverBasic) {
  // Testing basic getters and setters
  // Fixed base
  // Constructor 1
  PositionSolver position_solver_fixed1(tree_fixed_);
  position_solver_fixed1.set_filename("position_log");
  position_solver_fixed1.set_major_tolerance(0.001);
  position_solver_fixed1.set_minor_tolerance(0.01);

  shared_ptr<MathematicalProgram> prog_position_fixed1 =
      position_solver_fixed1.get_program();

  ASSERT_EQ(position_solver_fixed1.get_major_tolerance(), 0.001);
  ASSERT_EQ(position_solver_fixed1.get_minor_tolerance(), 0.01);

  // Constructor 2
  MatrixXd Q = MatrixXd::Identity(num_positions_fixed_, num_positions_fixed_);
  PositionSolver position_solver_fixed2(tree_fixed_, q_fixed_, Q);
  position_solver_fixed2.set_filename("position_log");
  position_solver_fixed2.set_major_tolerance(0.001);
  position_solver_fixed2.set_minor_tolerance(0.01);

  shared_ptr<MathematicalProgram> prog_position_fixed2 =
      position_solver_fixed1.get_program();

  ASSERT_EQ(position_solver_fixed2.get_major_tolerance(), 0.001);
  ASSERT_EQ(position_solver_fixed2.get_minor_tolerance(), 0.01);

  // Constructor 3 (both bases)
  PositionSolver position_solver_rpy(tree_rpy_, q_rpy_);
  position_solver_rpy.set_filename("position_log");
  position_solver_rpy.set_major_tolerance(0.001);
  position_solver_rpy.set_minor_tolerance(0.01);

  PositionSolver position_solver_quaternion(tree_quaternion_, q_quaternion_);
  position_solver_quaternion.set_filename("position_log");
  position_solver_quaternion.set_major_tolerance(0.001);
  position_solver_quaternion.set_minor_tolerance(0.01);

  shared_ptr<MathematicalProgram> prog_position_rpy =
      position_solver_rpy.get_program();

  shared_ptr<MathematicalProgram> prog_position_quaternion =
      position_solver_quaternion.get_program();

  ASSERT_EQ(position_solver_rpy.get_major_tolerance(), 0.001);
  ASSERT_EQ(position_solver_rpy.get_minor_tolerance(), 0.01);

  ASSERT_EQ(position_solver_quaternion.get_major_tolerance(), 0.001);
  ASSERT_EQ(position_solver_quaternion.get_minor_tolerance(), 0.01);
}

TEST_F(MultibodySolversTest, TestContactSolverBasic) {
  // Testing basic getters and setters
  // Only the floating base models are used for contact
  // Constructor 1
  ContactSolver contact_solver_rpy1(tree_rpy_, contact_info_rpy_);

  ContactSolver contact_solver_quaternion1(tree_quaternion_,
                                           contact_info_quaternion_);

  contact_solver_rpy1.set_filename("contact_log");
  contact_solver_rpy1.set_major_tolerance(0.002);
  contact_solver_rpy1.set_minor_tolerance(0.02);

  contact_solver_quaternion1.set_filename("contact_log");
  contact_solver_quaternion1.set_major_tolerance(0.002);
  contact_solver_quaternion1.set_minor_tolerance(0.02);

  ASSERT_EQ(contact_solver_rpy1.get_major_tolerance(), 0.002);
  ASSERT_EQ(contact_solver_rpy1.get_minor_tolerance(), 0.02);

  ASSERT_EQ(contact_solver_quaternion1.get_major_tolerance(), 0.002);
  ASSERT_EQ(contact_solver_quaternion1.get_minor_tolerance(), 0.02);

  // Testing mathematical program getter
  shared_ptr<MathematicalProgram> prog_contact_rpy1 =
      contact_solver_rpy1.get_program();
  shared_ptr<MathematicalProgram> prog_contact_quaternion1 =
      contact_solver_quaternion1.get_program();

  // Constructor 2
  MatrixXd Q_rpy = MatrixXd::Identity(num_positions_rpy_, num_positions_rpy_);
  ContactSolver contact_solver_rpy2(tree_rpy_, contact_info_rpy_, q_rpy_,
                                    Q_rpy);

  MatrixXd Q_quaternion =
      MatrixXd::Identity(num_positions_quaternion_, num_positions_quaternion_);
  ContactSolver contact_solver_quaternion2(
      tree_quaternion_, contact_info_quaternion_, q_quaternion_, Q_quaternion);

  contact_solver_rpy2.set_filename("contact_log");
  contact_solver_rpy2.set_major_tolerance(0.002);
  contact_solver_rpy2.set_minor_tolerance(0.02);

  contact_solver_quaternion2.set_filename("contact_log");
  contact_solver_quaternion2.set_major_tolerance(0.002);
  contact_solver_quaternion2.set_minor_tolerance(0.02);

  ASSERT_EQ(contact_solver_rpy2.get_major_tolerance(), 0.002);
  ASSERT_EQ(contact_solver_rpy2.get_minor_tolerance(), 0.02);

  ASSERT_EQ(contact_solver_quaternion2.get_major_tolerance(), 0.002);
  ASSERT_EQ(contact_solver_quaternion2.get_minor_tolerance(), 0.02);

  // Testing mathematical program getter
  shared_ptr<MathematicalProgram> prog_contact_rpy2 =
      contact_solver_rpy2.get_program();
  shared_ptr<MathematicalProgram> prog_contact_quaternion2 =
      contact_solver_quaternion2.get_program();

  // Constructor 3
  ContactSolver contact_solver_rpy3(tree_rpy_, contact_info_rpy_, q_rpy_);

  ContactSolver contact_solver_quaternion3(
      tree_quaternion_, contact_info_quaternion_, q_quaternion_);

  contact_solver_rpy3.set_filename("contact_log");
  contact_solver_rpy3.set_major_tolerance(0.002);
  contact_solver_rpy3.set_minor_tolerance(0.02);

  contact_solver_quaternion3.set_filename("contact_log");
  contact_solver_quaternion3.set_major_tolerance(0.002);
  contact_solver_quaternion3.set_minor_tolerance(0.02);

  ASSERT_EQ(contact_solver_rpy3.get_major_tolerance(), 0.002);
  ASSERT_EQ(contact_solver_rpy3.get_minor_tolerance(), 0.02);

  ASSERT_EQ(contact_solver_quaternion3.get_major_tolerance(), 0.002);
  ASSERT_EQ(contact_solver_quaternion3.get_minor_tolerance(), 0.02);

  // Testing mathematical program getter
  shared_ptr<MathematicalProgram> prog_contact_rpy3 =
      contact_solver_rpy3.get_program();
  shared_ptr<MathematicalProgram> prog_contact_quaternion3 =
      contact_solver_quaternion3.get_program();
}

TEST_F(MultibodySolversTest, TestFixedPointSolverBasic) {
  // Testing basic getters and setters
  // Fixed base
  // Constructor 1
  FixedPointSolver fp_solver_fixed1(tree_fixed_);
  fp_solver_fixed1.set_filename("fp_log");
  fp_solver_fixed1.set_major_tolerance(0.003);
  fp_solver_fixed1.set_minor_tolerance(0.03);

  shared_ptr<MathematicalProgram> prog_fp_fixed1 =
      fp_solver_fixed1.get_program();

  ASSERT_EQ(fp_solver_fixed1.get_major_tolerance(), 0.003);
  ASSERT_EQ(fp_solver_fixed1.get_minor_tolerance(), 0.03);

  // Constructor 2
  MatrixXd Q = MatrixXd::Identity(num_positions_fixed_, num_positions_fixed_);
  MatrixXd U = MatrixXd::Identity(num_efforts_fixed_, num_efforts_fixed_);
  FixedPointSolver fp_solver_fixed2(tree_fixed_, q_fixed_, u_fixed_, Q, U);
  fp_solver_fixed2.set_filename("fp_log");
  fp_solver_fixed2.set_major_tolerance(0.003);
  fp_solver_fixed2.set_minor_tolerance(0.03);

  shared_ptr<MathematicalProgram> prog_fp_fixed2 =
      fp_solver_fixed2.get_program();

  ASSERT_EQ(fp_solver_fixed2.get_major_tolerance(), 0.003);
  ASSERT_EQ(fp_solver_fixed2.get_minor_tolerance(), 0.03);

  // Constructor 3
  FixedPointSolver fp_solver_fixed3(tree_fixed_, q_fixed_, u_fixed_);
  fp_solver_fixed3.set_filename("fp_log");
  fp_solver_fixed3.set_major_tolerance(0.003);
  fp_solver_fixed3.set_minor_tolerance(0.03);

  shared_ptr<MathematicalProgram> prog_fp_fixed3 =
      fp_solver_fixed3.get_program();

  // Floating base constructors
  // Constructor 4
  FixedPointSolver fp_solver_rpy1(tree_rpy_, contact_info_rpy_);

  FixedPointSolver fp_solver_quaternion1(tree_quaternion_,
                                         contact_info_quaternion_);

  fp_solver_rpy1.set_filename("fp_log");
  fp_solver_rpy1.set_major_tolerance(0.003);
  fp_solver_rpy1.set_minor_tolerance(0.03);

  fp_solver_quaternion1.set_filename("fp_log");
  fp_solver_quaternion1.set_major_tolerance(0.003);
  fp_solver_quaternion1.set_minor_tolerance(0.03);

  shared_ptr<MathematicalProgram> prog_fp_rpy1 = fp_solver_rpy1.get_program();

  shared_ptr<MathematicalProgram> prog_fp_quaternion1 =
      fp_solver_quaternion1.get_program();

  ASSERT_EQ(fp_solver_rpy1.get_major_tolerance(), 0.003);
  ASSERT_EQ(fp_solver_rpy1.get_minor_tolerance(), 0.03);

  ASSERT_EQ(fp_solver_quaternion1.get_major_tolerance(), 0.003);
  ASSERT_EQ(fp_solver_quaternion1.get_minor_tolerance(), 0.03);

  // Constructor 5
  MatrixXd Q_rpy = MatrixXd::Identity(num_positions_rpy_, num_positions_rpy_);
  MatrixXd U_rpy = MatrixXd::Identity(num_efforts_rpy_, num_efforts_rpy_);

  MatrixXd Q_quaternion =
      MatrixXd::Identity(num_positions_quaternion_, num_positions_quaternion_);
  MatrixXd U_quaternion =
      MatrixXd::Identity(num_efforts_quaternion_, num_efforts_quaternion_);

  FixedPointSolver fp_solver_rpy2(tree_rpy_, contact_info_rpy_, q_rpy_, u_rpy_,
                                  Q_rpy, U_rpy);

  FixedPointSolver fp_solver_quaternion2(
      tree_quaternion_, contact_info_quaternion_, q_quaternion_, u_quaternion_,
      Q_quaternion, U_quaternion);

  fp_solver_rpy2.set_filename("fp_log");
  fp_solver_rpy2.set_major_tolerance(0.003);
  fp_solver_rpy2.set_minor_tolerance(0.03);

  fp_solver_quaternion2.set_filename("fp_log");
  fp_solver_quaternion2.set_major_tolerance(0.003);
  fp_solver_quaternion2.set_minor_tolerance(0.03);

  shared_ptr<MathematicalProgram> prog_fp_rpy2 = fp_solver_rpy2.get_program();

  shared_ptr<MathematicalProgram> prog_fp_quaternion2 =
      fp_solver_quaternion2.get_program();

  ASSERT_EQ(fp_solver_rpy2.get_major_tolerance(), 0.003);
  ASSERT_EQ(fp_solver_rpy2.get_minor_tolerance(), 0.03);

  ASSERT_EQ(fp_solver_quaternion2.get_major_tolerance(), 0.003);
  ASSERT_EQ(fp_solver_quaternion2.get_minor_tolerance(), 0.03);

  // Constructor 6
  FixedPointSolver fp_solver_rpy3(tree_rpy_, contact_info_rpy_, q_rpy_, u_rpy_);

  FixedPointSolver fp_solver_quaternion3(
      tree_quaternion_, contact_info_quaternion_, q_quaternion_, u_quaternion_);

  fp_solver_rpy3.set_filename("fp_log");
  fp_solver_rpy3.set_major_tolerance(0.003);
  fp_solver_rpy3.set_minor_tolerance(0.03);

  fp_solver_quaternion3.set_filename("fp_log");
  fp_solver_quaternion3.set_major_tolerance(0.003);
  fp_solver_quaternion3.set_minor_tolerance(0.03);

  shared_ptr<MathematicalProgram> prog_fp_rpy3 = fp_solver_rpy3.get_program();

  shared_ptr<MathematicalProgram> prog_fp_quaternion3 =
      fp_solver_quaternion3.get_program();

  ASSERT_EQ(fp_solver_rpy3.get_major_tolerance(), 0.003);
  ASSERT_EQ(fp_solver_rpy3.get_minor_tolerance(), 0.03);

  ASSERT_EQ(fp_solver_quaternion3.get_major_tolerance(), 0.003);
  ASSERT_EQ(fp_solver_quaternion3.get_minor_tolerance(), 0.03);
}

TEST_F(MultibodySolversTest, TestPositionSolverSolution) {
  // Fixed base
  PositionSolver position_solver_fixed(tree_fixed_, q_fixed_);
  position_solver_fixed.SetInitialGuessQ(q_fixed_);
  position_solver_fixed.AddJointLimitConstraint(0.001);

  MathematicalProgramResult program_result_fixed =
      position_solver_fixed.Solve();

  cout << "Position solver result (Fixed base): "
       << program_result_fixed.get_solution_result() << endl;

  VectorXd q_sol_fixed = position_solver_fixed.GetSolutionQ();

  // Solution dimension check
  ASSERT_EQ(q_sol_fixed.size(), num_positions_fixed_);
  // Checking if the solution constraints have been satisfied
  ASSERT_TRUE(position_solver_fixed.CheckConstraint(q_sol_fixed));

  // Floating base
  // RPY base
  PositionSolver position_solver_rpy(tree_rpy_, q_rpy_);
  position_solver_rpy.SetInitialGuessQ(q_rpy_);
  position_solver_rpy.AddFixedJointsConstraint(fixed_joints_map_rpy_);
  position_solver_rpy.AddJointLimitConstraint(0.001);

  MathematicalProgramResult program_result_rpy = position_solver_rpy.Solve();

  cout << "Position solver result (Rpy floating base): "
       << program_result_rpy.get_solution_result() << endl;

  VectorXd q_sol_rpy = position_solver_rpy.GetSolutionQ();

  // Solution dimension check
  ASSERT_EQ(q_sol_rpy.size(), num_positions_rpy_);
  // Checking if the solution constraints have been satisfied
  ASSERT_TRUE(position_solver_rpy.CheckConstraint(q_sol_rpy));

  // Quaternion base
  PositionSolver position_solver_quaternion(tree_quaternion_, q_quaternion_);
  position_solver_quaternion.SetInitialGuessQ(q_quaternion_);
  position_solver_quaternion.AddUnitQuaternionConstraint(
      position_map_quaternion_["base_qw"], position_map_quaternion_["base_qx"],
      position_map_quaternion_["base_qy"], position_map_quaternion_["base_qz"]);
  position_solver_quaternion.AddFixedJointsConstraint(
      fixed_joints_map_quaternion_);
  position_solver_quaternion.AddJointLimitConstraint(0.001);

  MathematicalProgramResult program_result_quaternion =
      position_solver_quaternion.Solve();

  cout << "Position solver result (Quaternion floating base): "
       << program_result_quaternion.get_solution_result() << endl;

  VectorXd q_sol_quaternion = position_solver_quaternion.GetSolutionQ();

  // Solution dimension check
  ASSERT_EQ(q_sol_quaternion.size(), num_positions_quaternion_);
  // Checking if the solution constraints have been satisfied
  ASSERT_TRUE(position_solver_quaternion.CheckConstraint(q_sol_quaternion));
}

TEST_F(MultibodySolversTest, TestContactSolverSolution) {
  // Contact solver for the floating base models
  // RPY base
  ContactSolver contact_solver_rpy(tree_rpy_, contact_info_rpy_, q_rpy_);
  contact_solver_rpy.SetInitialGuessQ(q_rpy_);
  contact_solver_rpy.AddFixedJointsConstraint(fixed_joints_map_rpy_);
  contact_solver_rpy.AddJointLimitConstraint(0.001);

  MathematicalProgramResult program_result_rpy = contact_solver_rpy.Solve();

  std::cout << "Contact solver result (Rpy Floating base): "
            << program_result_rpy.get_solution_result() << std::endl;

  VectorXd q_sol_rpy = contact_solver_rpy.GetSolutionQ();

  // Solution dimension check
  ASSERT_EQ(q_sol_rpy.size(), num_positions_rpy_);
  // Checking if the solution constraints have been satisfied
  ASSERT_TRUE(contact_solver_rpy.CheckConstraint(q_sol_rpy));

  // Quaternion base
  ContactSolver contact_solver_quaternion(
      tree_quaternion_, contact_info_quaternion_, q_quaternion_);
  contact_solver_quaternion.SetInitialGuessQ(q_quaternion_);
  contact_solver_quaternion.AddUnitQuaternionConstraint(
      position_map_quaternion_["base_qw"], position_map_quaternion_["base_qx"],
      position_map_quaternion_["base_qy"], position_map_quaternion_["base_qz"]);
  contact_solver_quaternion.AddFixedJointsConstraint(
      fixed_joints_map_quaternion_);
  contact_solver_quaternion.AddJointLimitConstraint(0.001);

  MathematicalProgramResult program_result_quaternion =
      contact_solver_quaternion.Solve();

  std::cout << "Contact solver result (Quaternion Floating base): "
            << program_result_quaternion.get_solution_result() << std::endl;

  VectorXd q_sol_quaternion = contact_solver_quaternion.GetSolutionQ();

  // std::cout << "q: " << std::endl << q_sol_quaternion << std::endl;

  // Solution dimension check
  ASSERT_EQ(q_sol_quaternion.size(), num_positions_quaternion_);
  // Checking if the solution constraints have been satisfied
  ASSERT_TRUE(contact_solver_quaternion.CheckConstraint(q_sol_quaternion));
}

TEST_F(MultibodySolversTest, TestFixedPointSolverSolution) {
  // Fixed base
  FixedPointSolver fp_solver_fixed(tree_fixed_, q_fixed_, u_fixed_);
  fp_solver_fixed.SetInitialGuess(q_fixed_, u_fixed_, lambda_fixed_);
  fp_solver_fixed.AddJointLimitConstraint(0.001);

  MathematicalProgramResult program_result_fixed = fp_solver_fixed.Solve();

  cout << "Fixed point solver result (Fixed base): "
       << program_result_fixed.get_solution_result() << endl;

  VectorXd q_sol_fixed = fp_solver_fixed.GetSolutionQ();
  VectorXd u_sol_fixed = fp_solver_fixed.GetSolutionU();
  VectorXd lambda_sol_fixed = fp_solver_fixed.GetSolutionLambda();

  // Solution dimension check
  ASSERT_EQ(q_sol_fixed.size(), num_positions_fixed_);
  ASSERT_EQ(u_sol_fixed.size(), num_efforts_fixed_);
  ASSERT_EQ(lambda_sol_fixed.size(), num_position_constraints_fixed_);
  // Solution constraints check
  ASSERT_TRUE(fp_solver_fixed.CheckConstraint(q_sol_fixed, u_sol_fixed,
                                              lambda_sol_fixed, 1e-8));

  // Rpy base
  FixedPointSolver fp_solver_rpy(tree_rpy_, contact_info_rpy_, q_rpy_, u_rpy_);
  fp_solver_rpy.SetInitialGuess(q_rpy_, u_rpy_, lambda_rpy_);
  fp_solver_rpy.AddSpreadNormalForcesCost();
  fp_solver_rpy.AddFrictionConeConstraint(0.8);
  fp_solver_rpy.AddFixedJointsConstraint(fixed_joints_map_rpy_);
  fp_solver_rpy.AddJointLimitConstraint(0.001);

  MathematicalProgramResult program_result_rpy = fp_solver_rpy.Solve();

  cout << "Fixed point solver result (Rpy base): "
       << program_result_rpy.get_solution_result() << endl;

  VectorXd q_sol_rpy = fp_solver_rpy.GetSolutionQ();
  VectorXd u_sol_rpy = fp_solver_rpy.GetSolutionU();
  VectorXd lambda_sol_rpy = fp_solver_rpy.GetSolutionLambda();

  // Solution dimension check
  ASSERT_EQ(q_sol_rpy.size(), num_positions_rpy_);
  ASSERT_EQ(u_sol_rpy.size(), num_efforts_rpy_);
  ASSERT_EQ(lambda_sol_rpy.size(), num_forces_rpy_);
  // Solution constraints check
  ASSERT_TRUE(
      fp_solver_rpy.CheckConstraint(q_sol_rpy, u_sol_rpy, lambda_sol_rpy));

  // Quaternion base
  FixedPointSolver fp_solver_quaternion(
      tree_quaternion_, contact_info_quaternion_, q_quaternion_, u_quaternion_);
  fp_solver_quaternion.SetInitialGuess(q_quaternion_, u_quaternion_,
                                       lambda_quaternion_);
  fp_solver_quaternion.AddSpreadNormalForcesCost();
  fp_solver_quaternion.AddUnitQuaternionConstraint(
      position_map_quaternion_["base_qw"], position_map_quaternion_["base_qx"],
      position_map_quaternion_["base_qy"], position_map_quaternion_["base_qz"]);
  fp_solver_quaternion.AddFrictionConeConstraint(0.8);
  fp_solver_quaternion.AddFixedJointsConstraint(fixed_joints_map_quaternion_);
  fp_solver_quaternion.AddJointLimitConstraint(0.001);

  MathematicalProgramResult program_result_quaternion =
      fp_solver_quaternion.Solve();

  cout << "Fixed point solver result (Quaternion base): "
       << program_result_quaternion.get_solution_result() << endl;

  VectorXd q_sol_quaternion = fp_solver_quaternion.GetSolutionQ();
  VectorXd u_sol_quaternion = fp_solver_quaternion.GetSolutionU();
  VectorXd lambda_sol_quaternion = fp_solver_quaternion.GetSolutionLambda();

  // Solution dimension check
  ASSERT_EQ(q_sol_quaternion.size(), num_positions_quaternion_);
  ASSERT_EQ(u_sol_quaternion.size(), num_efforts_quaternion_);
  ASSERT_EQ(lambda_sol_quaternion.size(), num_forces_quaternion_);
  // Solution constraints check
  ASSERT_TRUE(fp_solver_quaternion.CheckConstraint(
      q_sol_quaternion, u_sol_quaternion, lambda_sol_quaternion));
}

}  // namespace
}  // namespace multibody
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
