#pragma once

#include "solvers/nonlinear_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace dairlib::systems::controllers::alip_utils {

// Enum value also represents the sign of the y component of
// a nominal footstep during that stance period
// (So during left stance, the next footstep will be in the -y direction)
enum class Stance { kLeft = -1, kRight = 1 };
enum class ResetDiscretization{ kZOH, kFOH, kSPLIT };

struct AlipGaitParams {
  double height;
  double mass;
  double single_stance_duration;
  double double_stance_duration;
  double stance_width;
  Eigen::Vector2d desired_velocity;
  Stance initial_stance_foot;
  ResetDiscretization reset_discretization_method;
  friend std::ostream& operator<<(std::ostream& os, const AlipGaitParams& data);
};

inline std::ostream& operator<<(std::ostream& os, const AlipGaitParams& data) {

  os << "Periodic Alip Gait Parameters:"
        "\nHeight: " << data.height <<
     "\nMass: " << data.mass <<
     "\nT_ss: " << data.single_stance_duration <<
     "\nT_ds: " << data.double_stance_duration <<
     "\nStance Width: " << data.stance_width <<
     "\nVdes: " << data.desired_velocity.transpose() <<
     "\nInitial stance foot: " <<
     (data.initial_stance_foot == Stance::kLeft ? "kLeft" : "kRight") <<
     "\nReset Discretization method: " <<
        (data.reset_discretization_method == ResetDiscretization::kZOH ?
         "kZOH" : "kFOH") << "\n";

  return os;
}

typedef std::pair<const Eigen::Vector3d,
                  const drake::multibody::Frame<double>&> PointOnFramed;


/// Calculates parameters used to construct the ALIP state from the state of a
/// MultibodyPlant
/// \param plant the plant for which the ALIP state is being calculated
/// \param context context for plant
/// \param x plant state
/// \param stance_locations list of PointOnFramed representing all of the
/// contact points used to calculate the ALIP state. The ALIP contact point is
/// calculated as a weighted average of stance_locations using cop_weights
/// \param cop_weights see above
/// \param CoM_p pointer to a Vector3d which will store the calculated COM
/// position in the plant's world frame
/// \param L_p pointer to a Vector3d which will store the calculated angular
/// momentum of the plant about the contact point
/// \param stance_pos_p pointer to a vector3d which will store the location of
/// the calculated contact point in the plant's world frame
void CalcAlipState(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context,
                   const Eigen::VectorXd &x,
                   const std::vector<PointOnFramed>& stance_locations,
                   const std::vector<double>& cop_weights,
                   const drake::EigenPtr<Eigen::Vector3d> &CoM_p,
                   const drake::EigenPtr<Eigen::Vector3d> &L_p,
                   const drake::EigenPtr<Eigen::Vector3d> &stance_pos_p);


/// Calculates a reset map, A, which relates the pre and post double stance
/// ALIP states to the pre-and-port double stance stance leg locations via the
/// linear map x+ = A [x-, p-, p+]^{T}. Assumes
/// \param com_z height of the ALIP
/// \param m mass of the ALIP
/// \param Tds double stance time
/// \return A as described above
Eigen::Matrix<double, 4, 8> CalcResetMap(
    double com_z, double m, double Tds,
    ResetDiscretization discretization = ResetDiscretization::kZOH);

Eigen::Matrix<double, 4, 8> CalcMassNormalizedResetMap(
    double com_z, double Tds,
    ResetDiscretization discretization = ResetDiscretization::kZOH);

/// Step to step alip dynamics where the state is the ALIP state at the end of
/// double stance. returns A_s2s, B_s2s
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> AlipStepToStepDynamics(
    double com_z, double m, double Tss, double Tds,
    ResetDiscretization discretization);

/// Applies the reset map described in CalcResetMap to the pre-impact
/// ALIP state x
Eigen::Vector4d CalcReset(double com_z, double m, double Tds,
                          const Eigen::Vector4d& x, const Eigen::Vector3d& p0,
                          const Eigen::Vector3d& p1,
                          ResetDiscretization reset_discretization);

Eigen::Matrix4d CalcA(double com_z, double m);
Eigen::Matrix4d CalcAd(double com_z, double m, double t);
Eigen::Vector4d CalcBd(double com_z, double m, double t);
Eigen::Matrix4d CalcMassNormalizedA(double com_z);
Eigen::Matrix4d CalcMassNormalizedAd(double com_z, double t);

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
MassNormalizedAlipStepToStepDynamics(
    double com_z, double Tss, double Tds,
    ResetDiscretization discretization
);
Eigen::Matrix4d SolveDareTwoStep(
    const Eigen::Matrix4d& Q,
    double com_z, double m, double tss, double tds, int knots_per_mode,
    ResetDiscretization discretization);

std::pair<Eigen::Vector4d, Eigen::Vector4d> MakePeriodicAlipGait(
    const AlipGaitParams& gait_params);

std::vector<Eigen::VectorXd> MakePeriodicAlipGaitTrajectory(
    const AlipGaitParams& gait_params, int nmodes, int knots_per_mode);

}