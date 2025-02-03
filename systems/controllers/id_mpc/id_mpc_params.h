#pragma once

#include "Eigen/Dense"
#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_io.h"


namespace dairlib::systems::controllers::id_mpc {

struct IDMPCParams {
  int N;
  double dt;
  int num_full_torque_knots;
  int num_intervals_between_impacts = 0;
  double mu = 1.0;

  // cost weights
  Eigen::MatrixXd Wq;
  Eigen::MatrixXd Wrot;
  Eigen::MatrixXd Wv;
  Eigen::MatrixXd Wlambda;
  Eigen::MatrixXd Wu;
  Eigen::MatrixXd Wq_final;
  Eigen::MatrixXd Wrot_final;
  Eigen::MatrixXd Wv_final;
};

struct IDMPCParamsLoader {
  int N;
  double dt;
  int num_full_torque_knots;
  double mu = 1.0;

  // cost weights
  std::vector<double> Wq;
  std::vector<double> Wrot;
  std::vector<double> Wv;
  std::vector<double> Wlambda;
  std::vector<double> Wu;
  std::vector<double> Wq_final;
  std::vector<double> Wrot_final;
  std::vector<double> Wv_final;

  IDMPCParams params_out;

  template<typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(N));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(num_full_torque_knots));
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(Wq));
    a->Visit(DRAKE_NVP(Wrot));
    a->Visit(DRAKE_NVP(Wv));
    a->Visit(DRAKE_NVP(Wlambda));
    a->Visit(DRAKE_NVP(Wu));
    a->Visit(DRAKE_NVP(Wq_final));
    a->Visit(DRAKE_NVP(Wrot_final));
    a->Visit(DRAKE_NVP(Wv_final));

    params_out.N = N;
    params_out.dt = dt;
    params_out.num_full_torque_knots = num_full_torque_knots;
    params_out.mu = mu;
    Eigen::VectorXd Wq_diag = Eigen::Map<Eigen::VectorXd>(Wq.data(), Wq.size());
    Eigen::VectorXd Wrot_diag = Eigen::Map<Eigen::VectorXd>(Wrot.data(), Wrot.size());
    Eigen::VectorXd Wv_diag = Eigen::Map<Eigen::VectorXd>(Wv.data(), Wv.size());
    Eigen::VectorXd Wlambda_diag = Eigen::Map<Eigen::VectorXd>(Wlambda.data(), Wlambda.size());
    Eigen::VectorXd Wu_diag = Eigen::Map<Eigen::VectorXd>(Wu.data(), Wu.size());
    Eigen::VectorXd Wq_final_diag = Eigen::Map<Eigen::VectorXd>(Wq_final.data(), Wq_final.size());
    Eigen::VectorXd Wrot_final_diag = Eigen::Map<Eigen::VectorXd>(Wrot_final.data(), Wrot_final.size());
    Eigen::VectorXd Wv_final_diag = Eigen::Map<Eigen::VectorXd>(Wv_final.data(), Wv_final.size());

    params_out.Wq = Wq_diag.asDiagonal();
    params_out.Wrot = Wrot_diag.asDiagonal();
    params_out.Wv = Wv_diag.asDiagonal();
    params_out.Wlambda = Wlambda_diag.asDiagonal();
    params_out.Wu = Wu_diag.asDiagonal();
    params_out.Wq_final = Wq_final_diag.asDiagonal();
    params_out.Wrot_final = Wrot_final_diag.asDiagonal();
    params_out.Wv_final = Wv_final_diag.asDiagonal();
  }

};

inline IDMPCParams LoadIDMPCParamsFromYaml(const std::string& filename) {
  const auto archive = drake::yaml::LoadYamlFile<IDMPCParamsLoader>(filename);
  return archive.params_out;
}

}