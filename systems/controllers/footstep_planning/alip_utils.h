#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace dairlib::systems::controllers::alip_utils {

inline std::vector<std::vector<int>> cartesian_product(unsigned long range,
                                                       int sets) {
  auto products = std::vector<std::vector<int>>();
  for (int i = 0; i < pow(range, sets); i++) {
    products.emplace_back(std::vector<int>(sets, 0));
  }
  auto counter = std::vector<int>(sets, 0); // array of zeroes
  for (auto &product : products) {
    product = counter;

    // counter increment and wrapping/carry over
    counter.back()++;
    for (size_t i = counter.size() - 1; i != 0; i--) {
      if (counter[i] == range) {
        counter[i] = 0;
        counter[i - 1]++;
      } else break;
    }
  }
  return products;
}

typedef std::pair<const Eigen::Vector3d,
                  const drake::multibody::Frame<double>&> PointOnFramed;

void CalcAlipState(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context,
                   const Eigen::VectorXd &x,
                   const std::vector<PointOnFramed>& stance_locations,
                   const std::vector<double>& cop_weights,
                   const drake::EigenPtr<Eigen::Vector3d> &CoM_p,
                   const drake::EigenPtr<Eigen::Vector3d> &L_p,
                   const drake::EigenPtr<Eigen::Vector3d> &stance_pos_p);

Eigen::Matrix<double, 4, 8> CalcResetMap(double com_z, double m, double Tds);

Eigen::Vector4d CalcReset(
    double com_z, double m, double Tds,
    const Eigen::Vector4d& x, const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);

Eigen::Matrix4d CalcA(double com_z, double m);
Eigen::Matrix4d CalcAd(double com_z, double m, double t);

double XImpactTime(double t_start, double H, double m, double x, double Ly,
                   double x_impact);

double YImpactTime(double t_start, double H, double m, double y, double Lx,
                   double y_impact);

}