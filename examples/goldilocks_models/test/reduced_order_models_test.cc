#include "examples/goldilocks_models/reduced_order_models.h"

#include <list>
#include <memory>
#include <set>

#include <boost/any.hpp>
#include <gtest/gtest.h>

#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/controller/cassie_rom_planner_system.h"
#include "examples/goldilocks_models/controller/control_parameters.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "multibody/multibody_utils.h"

#include "drake/multibody/plant/multibody_plant.h"

using drake::MatrixX;
using drake::VectorX;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::System;
using drake::systems::SystemOutput;
using std::cout;
using std::endl;
using std::make_unique;
using std::set;
using std::string;
using std::unique_ptr;
using std::vector;

namespace dairlib {
namespace goldilocks_models {
namespace {

// TODO: Need to add a unit test for MirroredReducedOrderModel

// TODO: Need a unit test for B matrix in find_model::DynamicsConstraint
//  Example in traj_opt_given_weigths.cc

// Numerically calculate the Jacobian of a reduced order model position y = r(q)
MatrixX<double> CalcJByNumericalDiff(
    VectorX<double> q, const drake::multibody::MultibodyPlant<double>& plant,
    const ReducedOrderModel& rom, bool wrt_qdot = false) {
  auto context = plant.CreateDefaultContext();

  // Central differencing
  double dx = 4e-4;
  VectorX<double> r_f, r_i;
  MatrixX<double> J_wrt_qdot =
      MatrixX<double>(rom.n_y(), plant.num_positions());
  for (int i = 0; i < q.size(); i++) {
    q(i) += dx / 2;
    plant.SetPositions(context.get(), q);
    r_f = rom.EvalMappingFunc(q, *context);
    q(i) -= dx;
    plant.SetPositions(context.get(), q);
    r_i = rom.EvalMappingFunc(q, *context);
    q(i) += dx / 2;
    J_wrt_qdot.col(i) = (r_f - r_i) / dx;
  }

  if (wrt_qdot) {
    return J_wrt_qdot;
  } else {
    return multibody::JwrtqdotToJwrtv(q, J_wrt_qdot);
  }
}

// Numerically calculate JdotV of a reduced order model position y = r(q)
VectorX<double> CalcJdotVByNumericalDiff(
    VectorX<double> q, const VectorX<double>& v,
    const drake::multibody::MultibodyPlant<double>& plant,
    const ReducedOrderModel& rom) {
  VectorX<double> qdot(plant.num_positions());
  auto context = plant.CreateDefaultContext();
  plant.SetPositions(context.get(), q);
  plant.MapVelocityToQDot(*context, v, &qdot);

  VectorX<double> JdotV(rom.n_y());

  // Central differencing
  double dx = 4e-4;
  VectorX<double> Jv_0(rom.n_y());
  VectorX<double> Jv_i(rom.n_y());
  MatrixX<double> grad_Jv((rom.n_y()), q.size());
  for (int i = 0; i < q.size(); i++) {
    q(i) += dx / 2;
    Jv_i = CalcJByNumericalDiff(q, plant, rom, true) * qdot;
    q(i) -= dx;
    Jv_0 = CalcJByNumericalDiff(q, plant, rom, true) * qdot;
    q(i) += dx / 2;
    grad_Jv.col(i) = (Jv_i - Jv_0) / dx;
  }
  JdotV = grad_Jv * qdot;

  return JdotV;
}

class MonomialFeatureTest : public ::testing::Test {
 protected:
  MonomialFeatureTest(){};
};

TEST_F(MonomialFeatureTest, FeatureLength) {
  EXPECT_TRUE(MonomialFeatures(2, 2).length() == 6);
  // 1 + 4 + (4Choose2 + 4) = 1 + 4 + 10 = 15
  EXPECT_TRUE(MonomialFeatures(2, 4).length() == 15);
  // 1 + 6 + (6Choose2 + 6) = 1 + 6 + 21 = 28
  EXPECT_TRUE(MonomialFeatures(2, 6).length() == 28);
  // 1 + 8 + (8Choose2 + 8) = 1 + 8 + 36 = 45
  EXPECT_TRUE(MonomialFeatures(2, 8).length() == 45);
}

TEST_F(MonomialFeatureTest, SecondOrder) {
  /*
    order = 2 and n_q = 3

    Features =
      row index : symbolic term
      0: ()
      1: (0)
      2: (0, 0)
      3: (0, 1)
      4: (0, 2)
      5: (1)
      6: (1, 1)
      7: (1, 2)
      8: (2)
      9: (2, 2)
    First order partial derivatives =
      Key ==> Term
      1, (0) ==> 1, ()
      2, (0) ==> 2, (0)
      3, (0) ==> 1, (1)
      3, (1) ==> 1, (0)
      4, (0) ==> 1, (2)
      4, (2) ==> 1, (0)
      5, (1) ==> 1, ()
      6, (1) ==> 2, (1)
      7, (1) ==> 1, (2)
      7, (2) ==> 1, (1)
      8, (2) ==> 1, ()
      9, (2) ==> 2, (2)
    Second order partial derivatives =
      Key ==> Term
      2, (0, 0) ==> 2, ()
      3, (0, 1) ==> 2, ()
      4, (0, 2) ==> 2, ()
      6, (1, 1) ==> 2, ()
      7, (1, 2) ==> 2, ()
      9, (2, 2) ==> 2, ()
   */

  MonomialFeatures features(2, 3);
  EXPECT_TRUE(features.length() == 10);

  VectorX<double> q(3);
  q << 0, 1, 0.5;
  VectorX<double> qdot(3);
  qdot << 0.3, 0, 0;

  VectorX<double> expected_feature(10);
  expected_feature << 1, 0, 0, 0, 0, 1, 1, 0.5, 0.5, 0.25;
  EXPECT_TRUE((features.Eval(q) - expected_feature).norm() == 0);

  VectorX<double> expected_JV(10);
  expected_JV << 0, 0.3, 0, 0.3, 0.15, 0, 0, 0, 0, 0;
  EXPECT_TRUE((features.EvalJV(q, qdot) - expected_JV).norm() == 0);

  VectorX<double> expected_JdotV(10);
  expected_JdotV << 0, 0, 0.18, 0, 0, 0, 0, 0, 0, 0;
  EXPECT_TRUE((features.EvalJdotV(q, qdot) - expected_JdotV).norm() == 0);

  //  features.PrintSymbolicFeatures();
  //  features.PrintSymbolicPartialDerivatives(1);
  //  features.PrintSymbolicPartialDerivatives(2);
  //  cout << "==================\n";
  //  cout << features.Eval(q) << endl;
  //  cout << "==================\n";
  //  cout << features.EvalJV(q, qdot) << endl;
  //  cout << "==================\n";
  //  cout << features.EvalJdotV(q, qdot) << endl;
}

TEST_F(MonomialFeatureTest, HighOrder) {
  /*
    order = 3 and n_q = 2

    Features =
      row index : symbolic term
      0: ()
      1: (0)
      2: (0, 0)
      3: (0, 0, 0)
      4: (0, 0, 1)
      5: (0, 1)
      6: (0, 1, 1)
      7: (1)
      8: (1, 1)
      9: (1, 1, 1)
    First order partial derivatives =
      Key ==> Term
      1, (0) ==> 1, ()
      2, (0) ==> 2, (0)
      3, (0) ==> 3, (0, 0)
      4, (0) ==> 2, (0, 1)
      4, (1) ==> 1, (0, 0)
      5, (0) ==> 1, (1)
      5, (1) ==> 1, (0)
      6, (0) ==> 1, (1, 1)
      6, (1) ==> 2, (0, 1)
      7, (1) ==> 1, ()
      8, (1) ==> 2, (1)
      9, (1) ==> 3, (1, 1)
    Second order partial derivatives =
      Key ==> Term
      2, (0, 0) ==> 2, ()
      3, (0, 0) ==> 6, (0)
      4, (0, 0) ==> 2, (1)
      4, (0, 1) ==> 4, (0)
      5, (0, 1) ==> 2, ()
      6, (0, 1) ==> 4, (1)
      6, (1, 1) ==> 2, (0)
      8, (1, 1) ==> 2, ()
      9, (1, 1) ==> 6, (1)
   */

  MonomialFeatures features(3, 2);
  EXPECT_TRUE(features.length() == 10);

  VectorX<double> q(2);
  q << 0, 0.5;
  VectorX<double> qdot(2);
  qdot << 0.3, 0;

  VectorX<double> expected_feature(10);
  expected_feature << 1, 0, 0, 0, 0, 0, 0, 0.5, 0.25, 0.125;
  EXPECT_TRUE((features.Eval(q) - expected_feature).norm() == 0);

  VectorX<double> expected_JV(10);
  expected_JV << 0, 0.3, 0, 0, 0, 0.15, 0.075, 0, 0, 0;
  EXPECT_TRUE((features.EvalJV(q, qdot) - expected_JV).norm() == 0);

  VectorX<double> expected_JdotV(10);
  expected_JdotV << 0, 0, 0.18, 0, 0.09, 0, 0, 0, 0, 0;
  EXPECT_TRUE((features.EvalJdotV(q, qdot) - expected_JdotV).norm() == 0);

  // Check the order of the features. We hard coded here in case c++ internally
  // change the order of set<>
  // clang-format off
  std::vector<std::multiset<int>> handcoded_features = {
      {},
      {0},
      {0, 0},
      {0, 0, 0},
      {0, 0, 1},
      {0, 1},
      {0, 1, 1},
      {1},
      {1, 1},
      {1, 1, 1}};
  // clang-format on

  int i = 0;
  for (const auto& feat_i : features.features()) {
    cout << "  " << i << ": ";
    features.PrintMultiset(feat_i);
    cout << "\n";
    EXPECT_TRUE(feat_i == handcoded_features.at(i));
    i++;
  }

  //  features.PrintSymbolicFeatures();
  //  features.PrintSymbolicPartialDerivatives(1);
  //  features.PrintSymbolicPartialDerivatives(2);
}

TEST_F(MonomialFeatureTest, SkipIndices) {
  /*
    order = 2, n_q = 3 and skip the second element of q

    Features =
      row index : symbolic term
      0: ()
      1: (0)
      2: (0, 0)
      3: (0, 2)
      4: (2)
      5: (2, 2)
    First order partial derivatives =
      Key ==> Term
      1, (0) ==> 1, ()
      2, (0) ==> 2, (0)
      3, (0) ==> 1, (2)
      3, (2) ==> 1, (0)
      4, (2) ==> 1, ()
      5, (2) ==> 2, (2)
    Second order partial derivatives =
      Key ==> Term
      2, (0, 0) ==> 2, ()
      3, (0, 2) ==> 2, ()
      5, (2, 2) ==> 2, ()
   */
  MonomialFeatures features(2, 3, {1});

  EXPECT_TRUE(features.length() == 6);

  VectorX<double> q(3);
  q << 0, 1, 0.5;
  VectorX<double> qdot(3);
  qdot << 0.3, 0, 0;

  VectorX<double> expected_feature(6);
  expected_feature << 1, 0, 0, 0, 0.5, 0.25;
  EXPECT_TRUE((features.Eval(q) - expected_feature).norm() == 0);

  VectorX<double> expected_JV(6);
  expected_JV << 0, 0.3, 0, 0.15, 0, 0;
  EXPECT_TRUE((features.EvalJV(q, qdot) - expected_JV).norm() == 0);

  VectorX<double> expected_JdotV(6);
  expected_JdotV << 0, 0, 0.18, 0, 0, 0;
  EXPECT_TRUE((features.EvalJdotV(q, qdot) - expected_JdotV).norm() == 0);
}

class ReducedOrderModelTest : public ::testing::Test {
 protected:
  ReducedOrderModelTest()
      : plant_(drake::multibody::MultibodyPlant<double>(1e-3)) {
    addCassieMultibody(&plant_, nullptr, true /*floating base*/,
                       "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                       false /*spring model*/, false /*loop closure*/);
    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();
    n_q_ = plant_.num_positions();
    n_v_ = plant_.num_velocities();

    // Initialize states for testing
    // 1. Random state
    VectorX<double> x = VectorX<double>::Random(n_q_ + n_v_);
    x.head(4).normalize();
    x_samples_.push_back(x);
    // 2. Double support phase of Cassie walking
    x << 0.990065, 0.000339553, 0.00444831, 0.00085048, 0.00836164,
        -0.000249535, 1.03223, -0.000810813, 6.8811e-05, 0.00177426,
        -0.00514383, 0.447568, 0.44727, -1.01775, -1.01819, 1.29924, 1.30006,
        -1.56023, -1.56018, 0.156632, -0.0502397, 0.101071, 0.232441, -0.296125,
        -0.0559459, -0.663525, 0.116557, -0.0264677, -0.107556, 2.18153,
        -0.0230963, -1.65117, -1.02961, 1.75789, -0.0410481, -1.46269, 0.482573;
    x_samples_.push_back(x);
    // 3. Left single support phase of Cassie walking
    x << 0.989849, -0.000815987, -0.017933, -0.0111588, 0.344537, -0.148108,
        1.00902, -0.0357916, -0.0422061, -0.0068692, -0.0355008, 0.274222,
        0.644396, -1.00482, -1.50496, 1.36746, 1.73074, -1.45868, -0.936994,
        -0.110601, -0.0521661, -0.00286609, 0.910837, -0.0174017, -0.00158473,
        0.124156, 0.8427, 0.0224065, 0.0678774, -1.22403, 2.89698, 0.32455,
        2.21075, -0.333968, -2.51737, 1.36041, -4.312;
    x_samples_.push_back(x);
  }

  drake::multibody::MultibodyPlant<double> plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  int n_q_;
  int n_v_;
  std::vector<VectorX<double>> x_samples_;
};

TEST_F(ReducedOrderModelTest, SecondOrderFeatures) {
  // Create ROM with random parameters
  MonomialFeatures mapping_basis(2, n_q_, {3, 4, 5}, "mapping basis");
  MonomialFeatures dynamic_basis;  // We don't test dynamics evaluation

  std::unique_ptr<ReducedOrderModel> rom =
      std::make_unique<testing::Com>(plant_, mapping_basis, dynamic_basis);
  int n_theta = rom->n_theta();
  rom->SetTheta(VectorX<double>::Random(n_theta));

  // Initialize a random state
  for (const auto& x : x_samples_) {
    VectorX<double> q = x.head(n_q_);
    VectorX<double> v = x.tail(n_v_);

    plant_.SetPositionsAndVelocities(context_.get(), x);

    // Test ydot
    VectorX<double> qdot_numerical = CalcJByNumericalDiff(q, plant_, *rom) * v;
    VectorX<double> qdot_analytical = rom->EvalMappingFuncJV(q, v, *context_);
    EXPECT_TRUE((qdot_numerical - qdot_analytical).norm() < 1e-5);

    // Test Jacobian
    MatrixX<double> J_numerical = CalcJByNumericalDiff(q, plant_, *rom);
    MatrixX<double> J_analytical = rom->EvalMappingFuncJ(q, *context_);
    EXPECT_TRUE((J_numerical - J_analytical).norm() < 1e-5);

    // Test JdotV
    VectorX<double> JdotV_numerical =
        CalcJdotVByNumericalDiff(q, v, plant_, *rom);
    VectorX<double> JdotV_analytical =
        rom->EvalMappingFuncJdotV(q, v, *context_);
    EXPECT_TRUE((JdotV_numerical - JdotV_analytical).norm() < 1e-5);
  }

  // Benchmark of computation time (in millisecond) on 8750H CPU:
  //   qdot_numerical = 0.349023
  //   qdot_analytical = 0.040516
  //   J_numerical = 0.322259
  //   J_analytical = 0.059275
  //   JdotV_numerical = 12.7207
  //   JdotV_analytical = 0.087186
}

TEST_F(ReducedOrderModelTest, ThirdOrderFeatures) {
  // Create ROM with random parameters
  MonomialFeatures mapping_basis(3, n_q_, {3, 4, 5}, "mapping basis");
  MonomialFeatures dynamic_basis;  // We don't test dynamics evaluation

  std::unique_ptr<ReducedOrderModel> rom =
      std::make_unique<testing::Com>(plant_, mapping_basis, dynamic_basis);
  int n_theta = rom->n_theta();
  rom->SetTheta(VectorX<double>::Random(n_theta));

  // Initialize a random state
  for (const auto& x : x_samples_) {
    VectorX<double> q = x.head(n_q_);
    VectorX<double> v = x.tail(n_v_);

    plant_.SetPositionsAndVelocities(context_.get(), x);

    // Test ydot
    VectorX<double> qdot_numerical = CalcJByNumericalDiff(q, plant_, *rom) * v;
    VectorX<double> qdot_analytical = rom->EvalMappingFuncJV(q, v, *context_);
    EXPECT_TRUE((qdot_numerical - qdot_analytical).norm() < 1e-5);

    // Test Jacobian
    MatrixX<double> J_numerical = CalcJByNumericalDiff(q, plant_, *rom);
    MatrixX<double> J_analytical = rom->EvalMappingFuncJ(q, *context_);
    EXPECT_TRUE((J_numerical - J_analytical).norm() < 1e-5);

    // Test JdotV
    VectorX<double> JdotV_numerical =
        CalcJdotVByNumericalDiff(q, v, plant_, *rom);
    VectorX<double> JdotV_analytical =
        rom->EvalMappingFuncJdotV(q, v, *context_);
    EXPECT_TRUE((JdotV_numerical - JdotV_analytical).norm() < 1e-5);
  }

  // Benchmark of computation time (in millisecond) on 8750H CPU:
  //   qdot_numerical = 1.13087
  //   qdot_analytical = 0.217754
  //   J_numerical = 1.0687
  //   J_analytical = 0.407693
  //   JdotV_numerical = 41.3167
  //   JdotV_analytical = 0.238603
}

class ReducedOrderModelOptionTest : public ::testing::Test {
 protected:
  ReducedOrderModelOptionTest()
      : plant_(drake::multibody::MultibodyPlant<double>(1e-3)) {
    addCassieMultibody(&plant_, nullptr, true /*floating base*/,
                       "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                       false /*spring model*/, false /*loop closure*/);
    plant_.Finalize();
  };
  drake::multibody::MultibodyPlant<double> plant_;
};

TEST_F(ReducedOrderModelOptionTest, AllOptions) {
  int robot_option = 1;
  std::unique_ptr<ReducedOrderModel> rom;

  // Right leg joints indices (swing leg)
  vector<int> swing_leg_inds = {};
  for (auto& pair : multibody::makeNameToPositionsMap(plant_)) {
    if (pair.first.find("right") != std::string::npos) {
      swing_leg_inds.push_back(pair.second);
    }
  }
  EXPECT_TRUE(swing_leg_inds == vector<int>({16, 12, 8, 10, 14, 18}));

  // clang-format off

  rom = CreateRom(0, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 4);
  EXPECT_TRUE(rom->name() == "2D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({}));

  rom = CreateRom(1, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->name() == "2D lipm with swing foot");
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 8);
  EXPECT_TRUE(rom->invariant_elements() == set<int>({}));

  rom = CreateRom(2, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 2);
  EXPECT_TRUE(rom->name() == "Fixed COM vertical acceleration");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({}));

  rom = CreateRom(3, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "Fixed COM vertical acceleration + 2D swing foot");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({}));

  rom = CreateRom(4, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({}));

  rom = CreateRom(5, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->name() == "3D lipm with swing foot");
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 12);
  EXPECT_TRUE(rom->invariant_elements() == set<int>({}));

  rom = CreateRom(6, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->name() == "3D lipm with swing foot");
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 12);
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1, 2}));

  /*rom = CreateRom(7, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 0);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1, 2}));*/

  /*rom = CreateRom(8, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D GIP");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({}));*/

  rom = CreateRom(9, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(10, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm (pelvis)");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(11, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm (pelvis)");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1, 2}));

  rom = CreateRom(12, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 4);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm (pelvis)");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(13, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 4);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm (pelvis)");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1, 2}));

  rom = CreateRom(14, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 4);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 4);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm (pelvis)");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(15, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 4);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 4);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm (pelvis)");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(16, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 4);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 4);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(17, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(18, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({}));

  rom = CreateRom(19, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 6);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 6);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm (pelvis)");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(20, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 4);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 6);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm (pelvis)");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(21, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm (pelvis)");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(22, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm (pelvis)");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1, 2}));

  rom = CreateRom(23, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm (pelvis)");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(24, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1}));

  rom = CreateRom(25, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 12);
  EXPECT_TRUE(rom->name() == "3D lipm with swing foot");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({}));

  rom = CreateRom(26, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 12);
  EXPECT_TRUE(rom->name() == "3D lipm with swing foot");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1, 2}));

  rom = CreateRom(27, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 0);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 4);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1, 2}));

  rom = CreateRom(28, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 4);
  EXPECT_TRUE(rom->name() == "2D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({}));

  rom = CreateRom(29, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 2);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 4);
  EXPECT_TRUE(rom->name() == "2D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0}));

  rom = CreateRom(30, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 0);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 2);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1, 2}));

  rom = CreateRom(31, robot_option, plant_, false);
  EXPECT_TRUE(rom->mapping_basis().skip_inds() == vector<int>({0, 1, 2, 3, 4, 5, 6, 16, 12, 8, 10, 14, 18}));
  EXPECT_TRUE(rom->mapping_basis().n_order() == 0);
  EXPECT_TRUE(rom->mapping_basis().n_q() == plant_.num_positions());
  EXPECT_TRUE(rom->dynamic_basis().n_order() == 1);
  EXPECT_TRUE(rom->dynamic_basis().n_q() == 6);
  EXPECT_TRUE(rom->name() == "3D lipm");
  EXPECT_TRUE(rom->invariant_elements() == set<int>({0, 1, 2}));

  // clang-format on
}

class CassieRomPlannerSystemTest : public ::testing::Test {
 protected:
  CassieRomPlannerSystemTest()
      : plant_(drake::multibody::MultibodyPlant<double>(1e-3)) {
    addCassieMultibody(&plant_, nullptr, true /*floating base*/,
                       "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                       false /*spring model*/, false /*loop closure*/);
    plant_.Finalize();
  };
  drake::multibody::MultibodyPlant<double> plant_;
  int knots_per_mode_ = 10;

  PlannerSetting ConstructPlannerSetting(
      double single_support_duration, double double_support_duration,
      bool constant_rom_vel_during_double_support) {
    // Read-in the parameters
    RomWalkingGains gains;
    const YAML::Node& root =
        YAML::LoadFile(FindResourceOrThrow(GAINS_FILENAME));
    drake::yaml::YamlReadArchive(root).Accept(&gains);
    gains.left_support_duration = single_support_duration;
    gains.double_support_duration = double_support_duration;
    gains.constant_rom_vel_during_double_support =
        constant_rom_vel_during_double_support;

    PlannerSetting param;
    param.rom_option = 24;
    param.iter = 1;
    param.sample = 40;
    param.n_step = 2;
    param.n_step_lipm = 0;
    param.knots_per_mode = knots_per_mode_;
    param.zero_touchdown_impact = true;
    param.use_double_contact_points = true;
    param.equalize_timestep_size = true;
    param.fix_duration = true;
    param.feas_tol = 1e-2;
    param.opt_tol = 1e-2;
    param.max_iter = 200;
    param.use_ipopt = false;
    param.switch_to_snopt_after_first_loop = true;
    param.log_solver_info = false;
    param.time_limit = 1;
    param.realtime_rate_for_time_limit = 1;
    param.dir_model = "examples/goldilocks_models/test/rom24/";
    param.dir_data = "";
    param.init_file = "";
    param.dir_and_prefix_FOM = "";
    param.solve_idx_for_read_from_file = -1;
    param.gains = gains;

    return param;
  };

  std::unique_ptr<CassiePlannerWithMixedRomFom> RunDetermineNumberOfKnotPoints(
      double single_support_duration, double double_support_duration,
      double init_phase, bool constant_rom_vel_during_double_support) {
    PlannerSetting param = ConstructPlannerSetting(
        single_support_duration, double_support_duration,
        constant_rom_vel_during_double_support);
    double stride_period = single_support_duration + double_support_duration;

    std::set<int> relax_index = {5};                 //{3, 4, 5};
    vector<int> initialize_with_rom_state = {2, 5};  // for state, not only pos
    std::set<int> idx_const_rom_vel_during_double_support = {3, 4};
    if (!constant_rom_vel_during_double_support) {
      idx_const_rom_vel_during_double_support.clear();
    }
    auto rom_planner = std::make_unique<CassiePlannerWithMixedRomFom>(
        plant_, stride_period, param, relax_index, initialize_with_rom_state,
        idx_const_rom_vel_during_double_support, false, false, 0);
    double first_mode_duration = stride_period * (1 - init_phase);
    double remaining_single_support_duration =
        std::max(0.0, first_mode_duration - double_support_duration);
    int first_mode_knot_idx = rom_planner->DetermineNumberOfKnotPoints(
        init_phase, first_mode_duration, remaining_single_support_duration);
    cout << "first_mode_knot_idx = " << first_mode_knot_idx << endl;

    return rom_planner;
  }
};

TEST_F(CassieRomPlannerSystemTest, TestNumberOfKnotPointsWithoutDoubleSupport) {
  double single_support_duration = 0.35;
  double double_support_duration = 0.0;

  std::unique_ptr<CassiePlannerWithMixedRomFom> rom_planner;

  // Inputs
  std::vector<double> init_phases = {0,   0.1, 0.2, 0.3, 0.4,  0.5,
                                     0.6, 0.7, 0.8, 0.9, 0.999};
  // Expected outputs
  std::vector<int> n_knots_first_mode = {10, 10, 9, 8, 7, 6, 5, 4, 3, 2, 2};
  std::vector<double> min_dt = {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2,
                                1e-2, 1e-2, 1e-2, 1e-3, 1e-3};

  EXPECT_TRUE(init_phases.size() == n_knots_first_mode.size());
  EXPECT_TRUE(init_phases.size() == min_dt.size());
  for (int i = 0; i < init_phases.size(); i++) {
    double init_phase = init_phases[i];
    rom_planner = RunDetermineNumberOfKnotPoints(
        single_support_duration, double_support_duration, init_phase, false);
    EXPECT_TRUE(rom_planner->num_time_samples()[0] == n_knots_first_mode[i]);
    EXPECT_TRUE(rom_planner->min_dt()[0] == min_dt[i]);
  }
}

TEST_F(CassieRomPlannerSystemTest, TestNumberOfKnotPointsWithDoubleSupport1) {
  double single_support_duration = 0.30;
  double double_support_duration = 0.05;

  std::unique_ptr<CassiePlannerWithMixedRomFom> rom_planner;

  // Inputs
  std::vector<double> init_phases = {0,   0.1, 0.2, 0.3, 0.4,  0.5,
                                     0.6, 0.7, 0.8, 0.9, 0.999};
  // Expected outputs
  std::vector<int> n_knots_first_mode = {10, 10, 9, 8, 7, 6, 6, 5, 4, 3, 2};
  std::vector<int> n_knots_first_ds = {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2};
  std::vector<double> min_dt = {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2,
                                1e-2, 1e-2, 1e-3, 1e-2, 1e-3};
  // Note that currently I use the same min_dt for both single support and
  // double support, and I just make min_dt small when there are only two knot
  // points in the single support phase.
  // TODO: We should create two min_dt's but since we are not using this
  //  variable in our current MPC, it's not urgent.

  // Print outs
  double stride_duration = single_support_duration + double_support_duration;
  for (double init_phase : init_phases) {
    cout << stride_duration * init_phase << endl;
  }
  //  0
  //  0.035
  //  0.07
  //  0.105
  //  0.14
  //  0.175
  //  0.21
  //  0.245
  //  0.28
  //  0.315
  //  0.34965

  EXPECT_TRUE(init_phases.size() == n_knots_first_mode.size());
  EXPECT_TRUE(init_phases.size() == n_knots_first_ds.size());
  EXPECT_TRUE(init_phases.size() == min_dt.size());
  for (int i = 0; i < init_phases.size(); i++) {
    double init_phase = init_phases[i];
    rom_planner = RunDetermineNumberOfKnotPoints(
        single_support_duration, double_support_duration, init_phase, true);
    /*cout << "==rom_planner->num_time_samples()[0] vs n_knots_first_mode[i]: "
         << rom_planner->num_time_samples()[0] << "==" << n_knots_first_mode[i]
         << endl;
    cout << "==rom_planner->num_time_samples_ds()[0] vs n_knots_first_ds[i]: "
         << rom_planner->num_time_samples_ds()[0] << "==" << n_knots_first_ds[i]
         << endl;
    cout << "==rom_planner->min_dt()[0] vs min_dt[i]: "
         << rom_planner->min_dt()[0] << "==" << min_dt[i] << endl;*/
    EXPECT_TRUE(rom_planner->num_time_samples()[0] == n_knots_first_mode[i]);
    EXPECT_TRUE(rom_planner->num_time_samples_ds()[0] == n_knots_first_ds[i]);
    EXPECT_TRUE(rom_planner->min_dt()[0] == min_dt[i]);
  }
}

TEST_F(CassieRomPlannerSystemTest, TestNumberOfKnotPointsWithDoubleSupport2) {
  double stride_duration = 0.35;

  // Inputs
  std::vector<double> single_support_durations = {0.35, 0.3, 0.25, 0.2,
                                                  0.15, 0.1, 0.05, 0.001};
  // Expected outputs
  std::vector<int> knots_per_single_support = {10, 8, 7, 6, 5, 4, 3, 2};

  EXPECT_TRUE(single_support_durations.size() ==
              knots_per_single_support.size());

  // Construct knots_per_double_support
  std::vector<int> knots_per_double_support;
  //  cout << "knots_per_double_support = \n";
  for (int i = 0; i < single_support_durations.size(); i++) {
    EXPECT_TRUE(single_support_durations[i] !=
                0);  // we don't allow the test case of 0 single duration
    if (single_support_durations[i] == stride_duration) {
      knots_per_double_support.push_back(0);
    } else {
      knots_per_double_support.push_back(knots_per_mode_ -
                                         knots_per_single_support[i] + 1);
    }
    //    cout << knots_per_double_support.back() << endl;
  }
  EXPECT_TRUE(single_support_durations.size() ==
              knots_per_double_support.size());

  std::unique_ptr<CassiePlannerWithMixedRomFom> rom_planner;
  for (int i = 0; i < single_support_durations.size(); i++) {
    double single_support_duration = single_support_durations[i];
    double double_support_duration = stride_duration - single_support_duration;
    double init_phase = 0;
    EXPECT_TRUE(std::abs(single_support_duration + double_support_duration -
                         stride_duration) < 1e-15);
    rom_planner = RunDetermineNumberOfKnotPoints(
        single_support_duration, double_support_duration, init_phase, true);

    //    cout << "rom_planner->knots_per_single_support() = "
    //         << rom_planner->knots_per_single_support() << endl;
    EXPECT_TRUE(rom_planner->knots_per_single_support() ==
                knots_per_single_support[i]);
    if (single_support_durations[i] == stride_duration) {
      EXPECT_TRUE(rom_planner->knots_per_double_support() == 0);
      EXPECT_TRUE(rom_planner->knots_per_single_support() +
                      rom_planner->knots_per_double_support() ==
                  knots_per_mode_);
    } else {
      EXPECT_TRUE(rom_planner->knots_per_double_support() ==
                  knots_per_mode_ - knots_per_single_support[i] + 1);
      EXPECT_TRUE(rom_planner->knots_per_single_support() +
                      rom_planner->knots_per_double_support() ==
                  knots_per_mode_ + 1);
    }
  }
}

}  // namespace
}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
