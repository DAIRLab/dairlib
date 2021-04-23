#include "drake/common/yaml/yaml_read_archive.h"
#include "yaml-cpp/yaml.h"
#include "include/_usr_include_eigen3/Eigen/src/Core/Matrix.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct MpcGains {
  std::vector<double> StateW;
  std::vector<double> InputW;

  VectorXd q;
  VectorXd r;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(StateW));
    a->Visit(DRAKE_NVP(InputW));

    q = Eigen::Map<Eigen::VectorXd>(this->StateW.data(), 12);
    r = Eigen::Map<Eigen::VectorXd>(this->StateW.data(), 6);
  }
};
