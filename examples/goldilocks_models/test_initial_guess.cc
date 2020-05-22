//
// Created by jianshu on 4/8/20.
//
#include <gflags/gflags.h>
#include <stdio.h>  // For removing files
#include <thread>  // multi-threading
#include <chrono>
#include <ctime>
#include <queue>  // First in first out
#include <deque>  // queue with feature of finding elements
#include <utility>  // std::pair, std::make_pair
#include <sys/stat.h>  // Check the existence of a file/folder
#include <cstdlib>  // System call to create folder (and also parent directory)


#include "common/find_resource.h"
#include "examples/goldilocks_models/initial_guess.h"
#include "systems/goldilocks_models/file_utils.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::pair;
using std::string;
using std::to_string;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXcd;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using dairlib::FindResourceOrThrow;


namespace dairlib::goldilocks_models {
    int test_initial_guess(int argc, char* argv[]){
      //create test data and save it
      int iter = 10;
      int sample = 0;
      int total_sample_num = 27;
      double min_sl = 0.1625;
      double max_sl = 0.2375;
      double min_gi = -0.125;
      double max_gi = 0.125;
      double min_tr = -0.3125;
      double max_tr = 0.3125;
      const string dir = "../dairlib_data/goldilocks_models/find_models/robot_1_test/";

      bool use_created_data = true;
      if(use_created_data) {
        //for each iteration, create theta_s and theta_sDDot
        int iteration = 0;
        for (iteration = 0; iteration <= iter; iteration++) {
          VectorXd theta_s = VectorXd::Random(70);
          VectorXd theta_sDDot = VectorXd::Random(7);
          writeCSV(dir + to_string(iteration) + string("_theta_s.csv"),
                   theta_s);
          writeCSV(dir + to_string(iteration) + string("_theta_sDDot.csv"),
                   theta_sDDot);
          //for each sample, create gamma, is_success and w
          int sample = 0;
          for (sample = 0; sample <= total_sample_num; sample++) {
            string
                prefix = to_string(iteration) + "_" + to_string(sample) + "_";
            MatrixXd
                stride_length = (max_sl + min_sl) / 2 * MatrixXd::Ones(1, 1) +
                (max_sl - min_sl) / 2 * MatrixXd::Random(1, 1);
            writeCSV(dir + prefix + string("stride_length.csv"),
                     stride_length);
            MatrixXd
                ground_incline = (max_gi + min_gi) / 2 * MatrixXd::Ones(1, 1)
                + (max_gi - min_gi) / 2 * MatrixXd::Random(1, 1);
            writeCSV(dir + prefix + string("ground_incline.csv"),
                     ground_incline);
            MatrixXd turning_rate = (max_tr + min_tr) / 2 * MatrixXd::Ones(1, 1)
                + (max_tr - min_tr) / 2 * MatrixXd::Random(1, 1);
            writeCSV(dir + prefix + string("turning_rate.csv"),
                     turning_rate);
            bool is_success = 1;
            writeCSV(dir + prefix + string("is_success.csv"),
                     is_success * MatrixXd::Ones(1, 1));
            VectorXd w = VectorXd::Random(1478);
            writeCSV(dir + prefix + string("w.csv"), w);
          }
        }
      }
      string initial_file = set_initial_guess(dir, iter, sample, total_sample_num,
          min_sl, max_sl, min_gi, max_gi, min_tr, max_tr);
      return 0;
    }
}


int main(int argc, char* argv[]) {
    return dairlib::goldilocks_models::test_initial_guess(argc, argv);
}