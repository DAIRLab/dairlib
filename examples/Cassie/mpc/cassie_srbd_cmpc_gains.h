#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/yaml/yaml_read_archive.h"
#include "include/yaml-cpp/yaml.h"
#include "include/_usr_include_eigen3/Eigen/src/Core/Matrix.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct MpcGains {
  double mu;
  double ReachabilityLim;
  std::vector<double> StateW;
  std::vector<double> FinalStateW;
  std::vector<double> InputW;
  std::vector<double> FlatGroundW;
  std::vector<double> StanceFootW;
  std::vector<double> KinReachW;

  VectorXd q;
  VectorXd qf;
  VectorXd r;
  MatrixXd W_flat_ground;
  MatrixXd W_stance_foot;
  MatrixXd W_kin_reach;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(ReachabilityLim));
    a->Visit(DRAKE_NVP(StateW));
    a->Visit(DRAKE_NVP(FinalStateW));
    a->Visit(DRAKE_NVP(InputW));
    a->Visit(DRAKE_NVP(FlatGroundW));
    a->Visit(DRAKE_NVP(StanceFootW));
    a->Visit(DRAKE_NVP(KinReachW));


    q = Eigen::Map<Eigen::VectorXd>(this->StateW.data(), 12);
    qf = Eigen::Map<Eigen::VectorXd>(this->FinalStateW.data(), 12);
    r = Eigen::Map<Eigen::VectorXd>(this->InputW.data(), 6);
    W_flat_ground = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->FlatGroundW.data(), 1,1);
    W_stance_foot = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->StanceFootW.data(), 2, 2);
    W_kin_reach = Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        this->KinReachW.data(), 2, 2);
  }
};
