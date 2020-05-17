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
        const string dir = "../dairlib_data/goldilocks_models/find_models/robot_0/";
        int iter = 2;
        int sample = 0;
        int total_sample_num = 27;
        double min_sl = 0.1625;
        double max_sl = 0.2375;
        double min_gi = -0.125;
        double max_gi = 0.125;
        double min_tr = -0.3125;
        double max_tr = 0.3125;
        for (sample = 0;sample<25;sample++) {
            string initial_file = set_initial_guess(dir, iter, sample, total_sample_num, min_sl, max_sl, min_gi,
                                                    max_gi);
        }
        return 0;
    }
}


int main(int argc, char* argv[]) {
    return dairlib::goldilocks_models::test_initial_guess(argc, argv);
}