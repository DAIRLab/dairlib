#include <gtest/gtest.h>

#include "examples/goldilocks_models/find_models/initial_guess.h"

namespace dairlib::goldilocks_models {

/// ROM for testing
class DummyRom : public ReducedOrderModel {
 public:
  static const int kDimension = 1;

  DummyRom(const MonomialFeatures& mapping_basis,
           const MonomialFeatures& dynamic_basis)
      : ReducedOrderModel(kDimension, 0,
                          drake::MatrixX<double>::Zero(kDimension, 0),
                          mapping_basis.length(), dynamic_basis.length(),
                          mapping_basis, dynamic_basis, {}, "Dummy ROM") {
    // Always check dimension after model construction
    CheckModelConsistency();
  };

  // Copy constructor for the Clone() method
  DummyRom(const DummyRom& old_obj) : ReducedOrderModel(old_obj){};

  // Use covariant return type for Clone method. It's more useful.
  std::unique_ptr<ReducedOrderModel> Clone() const override {
    return std::make_unique<DummyRom>(*this);
  }

  // Evaluators for features of y, yddot, y's Jacobian and y's JdotV
  drake::VectorX<double> EvalMappingFeat(
      const drake::VectorX<double>& q,
      const drake::systems::Context<double>& context) const final {
    return mapping_basis().Eval(q);
  };
  drake::VectorX<double> EvalDynamicFeat(
      const drake::VectorX<double>& y, const drake::VectorX<double>& ydot,
      const drake::VectorX<double>& tau) const final {
    throw std::runtime_error("not implemented");
    return drake::VectorX<double>::Zero(0);
  };
  drake::VectorX<double> EvalMappingFeatJV(
      const drake::VectorX<double>& q, const drake::VectorX<double>& v,
      const drake::systems::Context<double>& context) const final {
    throw std::runtime_error("not implemented");
    return drake::VectorX<double>::Zero(0);
  };
  drake::VectorX<double> EvalMappingFeatJdotV(
      const drake::VectorX<double>& q, const drake::VectorX<double>& v,
      const drake::systems::Context<double>& context) const final {
    throw std::runtime_error("not implemented");
    return drake::VectorX<double>::Zero(0);
  };
  drake::MatrixX<double> EvalMappingFeatJ(
      const drake::VectorX<double>& q,
      const drake::systems::Context<double>& context) const final {
    return mapping_basis().EvalJwrtqdot(q);
  };
};

class InitialGuessTest : public ::testing::Test {};

int test_initial_guess(int iter, int sample, int robot) {
  // create test data and save it
  int use_database = false;
  // create task_gen
  GridTasksGenerator task_gen_grid;
  if (robot == 0) {
    task_gen_grid = GridTasksGenerator(
        3, {"stride length", "ground incline", "velocity"}, {2, 2, 2},
        {0.25, 0, 0.4}, {0.015, 0.05, 0.02}, {true, true, true});
  } else {
    task_gen_grid = GridTasksGenerator(
        4, {"stride length", "ground incline", "velocity", "turning rate"},
        {2, 2, 2, 2}, {0.3, 0, 0.5, 0}, {0.015, 0.05, 0.04, 0.125},
        {true, true, true, true});
  }
  TasksGenerator* task_gen = &task_gen_grid;
  int total_sample_num = task_gen->total_sample_number();
  // create task
  Task task(task_gen->names());
  task.set(task_gen->NewTask(sample));
  // create rom
  MonomialFeatures basis(1, 2, {});
  DummyRom dummy_rom(basis, basis);
  ReducedOrderModel* rom = &dummy_rom;
  int n_theta_y = rom->n_theta_y();
  int n_theta_yddot = rom->n_theta_yddot();
  VectorXd current_theta_y = VectorXd::Random(n_theta_y);
  VectorXd current_theta_yddot = VectorXd::Random(n_theta_yddot);
  rom->SetThetaY(current_theta_y);
  rom->SetThetaYddot(current_theta_yddot);
  // dummy decision variable size
  int n_w = 20;

  const string dir =
      "../dairlib_data/goldilocks_models/find_models/robot_1_test/";
  if (!CreateFolderIfNotExist(dir, false)) return 0;

  // for each iteration, create theta_s and theta_sDDot
  int iteration = 0;
  for (iteration = 0; iteration <= iter; iteration++) {
    VectorXd theta_y = current_theta_y + 1e-4 * VectorXd::Random(n_theta_y);
    VectorXd theta_yddot =
        current_theta_yddot + 1e-4 * VectorXd::Random(n_theta_yddot);
    writeCSV(dir + to_string(iteration) + string("_theta_y.csv"), theta_y);
    writeCSV(dir + to_string(iteration) + string("_theta_yddot.csv"),
             theta_yddot);
    // for each sample, create gamma, is_success and w
    int sample = 0;
    int dim = 0;
    for (sample = 0; sample <= total_sample_num; sample++) {
      string prefix = to_string(iteration) + "_" + to_string(sample) + "_";
      VectorXd gamma(task_gen->dim());
      for (dim = 0; dim < task_gen->dim(); dim++) {
        double min = task_gen->task_min(task_gen->names()[dim]);
        double max = task_gen->task_min(task_gen->names()[dim]);
        std::uniform_real_distribution<double> dist(min, max);
        std::default_random_engine re;
        gamma[dim] = dist(re);
      }
      writeCSV(dir + prefix + string("task.csv"), gamma);
      bool is_success = 1;
      writeCSV(dir + prefix + string("is_success.csv"),
               is_success * MatrixXd::Ones(1, 1));
      VectorXd w = VectorXd::Random(n_w);
      writeCSV(dir + prefix + string("w.csv"), w);
    }
  }

  string initial_file = SetInitialGuessByInterpolation(
      dir, iter, sample, task_gen, task, *rom, use_database, robot);
  return 1;
}

TEST_F(InitialGuessTest, DifferentIter) {
  EXPECT_EQ(1, test_initial_guess(10, 0, 0));
  EXPECT_EQ(1, test_initial_guess(15, 0, 0));
}
TEST_F(InitialGuessTest, DifferentRobot) {
  EXPECT_EQ(1, test_initial_guess(10, 0, 1));
  EXPECT_EQ(1, test_initial_guess(15, 0, 1));
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
