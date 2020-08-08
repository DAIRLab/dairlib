#pragma once

#include <string>
#include <Eigen/Dense>
#include "examples/goldilocks_models/find_models/goldilocks_model_traj_opt.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/task.h"

namespace dairlib {
namespace goldilocks_models {

void trajOptGivenWeights(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_autoDiff,
    const ReducedOrderModel& rom, InnerLoopSetting inner_loop_setting,
    Task task, const SubQpData& QPs,
    const vector<std::shared_ptr<int>>& thread_finished_vec,
    bool is_get_nominal, bool extend_model, int sample_idx, int n_rerun,
    double cost_threshold_for_update, int N_rerun, int rom_option,
    int robot_option,bool currently_find_mediate_sample,int total_number_sample);

void addRegularization(bool is_get_nominal, double eps_reg,
                       GoldilocksModelTrajOpt& gm_traj_opt);
void setInitialGuessFromFile(const string& directory, const string& init_file,
                             GoldilocksModelTrajOpt& gm_traj_opt);
void augmentConstraintToFixThetaScaling(MatrixXd& B, MatrixXd& A, VectorXd& y,
                                        VectorXd& lb, VectorXd& ub, int n_s,
                                        int n_feature_s,
                                        const VectorXd& theta_s,
                                        int sample_idx);

}  // namespace goldilocks_models
}  // namespace dairlib
