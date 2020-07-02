#include <gtest/gtest.h>

#include "examples/goldilocks_models/find_models/initial_guess.h"

namespace dairlib::goldilocks_models {

class InitialGuessTest : public ::testing::Test {};

int test_initial_guess(int iter, int sample, int robot) {
  // create test data and save it
  int use_database = false;
  // create task_gen
  GridTasksGenerator task_gen_grid;
  if (robot == 0) {
    task_gen_grid = GridTasksGenerator(
        3, {"stride length", "ground incline", "velocity"}, {10, 5, 5},
        {0.25, 0, 0.4}, {0.015, 0.05, 0.02}, true);
  } else {
    task_gen_grid = GridTasksGenerator(
        4, {"stride length", "ground incline", "velocity", "turning rate"},
        {10, 5, 5, 5}, {0.3, 0, 0.5, 0}, {0.015, 0.05, 0.04, 0.125}, true);
  }
  TasksGenerator* task_gen = &task_gen_grid;
  int total_sample_num = task_gen->total_sample_number();
  // create task
  Task task(task_gen->names());
  task.set(task_gen->NewTask(sample));
  // create rom
  RomData rom = RomData(1, 2, 2, 2);
  VectorXd current_theta = VectorXd::Random(4);
  rom.SetTheta(current_theta);

  const string dir =
      "../dairlib_data/goldilocks_models/find_models/robot_1_test/";
  if (!CreateFolderIfNotExist(dir, false)) return 0;

  // for each iteration, create theta_s and theta_sDDot
  int iteration = 0;
  for (iteration = 0; iteration <= iter; iteration++) {
    VectorXd theta_y = VectorXd::Random(2);
    VectorXd theta_yDDot = VectorXd::Random(2);
    writeCSV(dir + to_string(iteration) + string("_theta_y.csv"), theta_y);
    writeCSV(dir + to_string(iteration) + string("_theta_yDDot.csv"),
             theta_yDDot);
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
      writeCSV(dir + prefix + string("_task.csv"), gamma);
      bool is_success = 1;
      writeCSV(dir + prefix + string("is_success.csv"),
               is_success * MatrixXd::Ones(1, 1));
      VectorXd w = VectorXd::Random(20);
      writeCSV(dir + prefix + string("w.csv"), w);
    }
  }

  string initial_file = SetInitialGuessByInterpolation(
      dir, iter, sample, task_gen, task, rom, use_database, robot);
  return 0;
}

TEST_F(InitialGuessTest, DifferentIter) {
  EXPECT_EQ(0, test_initial_guess(10, 0, 0));
  EXPECT_EQ(0, test_initial_guess(15, 0, 0));
  EXPECT_EQ(0, test_initial_guess(20, 0, 0));
}
TEST_F(InitialGuessTest, DifferentRobot) {
  EXPECT_EQ(0, test_initial_guess(10, 0, 1));
  EXPECT_EQ(0, test_initial_guess(15, 0, 1));
  EXPECT_EQ(0, test_initial_guess(20, 0, 1));
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
