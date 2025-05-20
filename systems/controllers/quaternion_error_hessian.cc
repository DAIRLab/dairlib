#include "quaternion_error_hessian.h"
#include <iostream>


using Eigen::VectorXd;
using Eigen::MatrixXd;


namespace dairlib{
namespace systems{

Eigen::MatrixXd hessian_of_squared_quaternion_angle_difference(
    const Eigen::VectorXd& quat, const Eigen::VectorXd& quat_desired)
{
    // Check the inputs are of expected shape.
    DRAKE_DEMAND(quat.size() == 4);
    DRAKE_DEMAND(quat_desired.size() == 4);

    // Extract the quaternion components.
    double q_w = quat(0);
    double q_x = quat(1);
    double q_y = quat(2);
    double q_z = quat(3);

    double r_w = quat_desired(0);
    double r_x = quat_desired(1);
    double r_y = quat_desired(2);
    double r_z = quat_desired(3);
    
    // Define reusable expressions.
    double exp_1 = std::atan2(
        std::sqrt(std::pow(q_w*r_x - q_x*r_w + q_y*r_z - q_z*r_y, 2) +
        std::pow(q_w*r_y - q_x*r_z - q_y*r_w + q_z*r_x, 2) +
        std::pow(q_w*r_z + q_x*r_y - q_y*r_x - q_z*r_w, 2)),
        q_w*r_w + q_x*r_x + q_y*r_y + q_z*r_z);
    double exp_2 = std::pow(q_w, 2)*std::pow(r_x, 2) +
        std::pow(q_w, 2)*std::pow(r_y, 2) + std::pow(q_w, 2)*std::pow(r_z, 2) -
        2*q_w*q_x*r_w*r_x - 2*q_w*q_y*r_w*r_y - 2*q_w*q_z*r_w*r_z +
        std::pow(q_x, 2)*std::pow(r_w, 2) + std::pow(q_x, 2)*std::pow(r_y, 2) +
        std::pow(q_x, 2)*std::pow(r_z, 2) -
        2*q_x*q_y*r_x*r_y - 2*q_x*q_z*r_x*r_z;
    double exp_3 = std::pow(q_y, 2)*std::pow(r_w, 2) +
        std::pow(q_y, 2)*std::pow(r_x, 2) + std::pow(q_y, 2)*std::pow(r_z, 2) -
        2*q_y*q_z*r_y*r_z + std::pow(q_z, 2)*std::pow(r_w, 2) +
        std::pow(q_z, 2)*std::pow(r_x, 2) + std::pow(q_z, 2)*std::pow(r_y, 2);
    double exp_4 = std::pow(q_w, 2) + std::pow(q_x, 2) +
        std::pow(q_y, 2) + std::pow(q_z, 2);
    double exp_5 = std::pow(exp_4, 2)*std::pow(exp_2 + exp_3, 5/2);
    double exp_6 = std::pow(exp_2 + exp_3, 3/2);
    double exp_7 = q_w*q_x*r_x + q_w*q_y*r_y + q_w*q_z*r_z -
        std::pow(q_x, 2)*r_w - std::pow(q_y, 2)*r_w - std::pow(q_z, 2)*r_w;
    double exp_8 = std::pow(q_w, 2)*r_y - q_w*q_y*r_w + std::pow(q_x, 2)*r_y -
        q_x*q_y*r_x - q_y*q_z*r_z + std::pow(q_z, 2)*r_y;
    double exp_9 = std::pow(q_w, 2)*r_x - q_w*q_x*r_w - q_x*q_y*r_y -
        q_x*q_z*r_z + std::pow(q_y, 2)*r_x + std::pow(q_z, 2)*r_x;
    double exp_10 = q_w*r_w*r_z + q_x*r_x*r_z + q_y*r_y*r_z -
        q_z*std::pow(r_w, 2) - q_z*std::pow(r_x, 2) - q_z*std::pow(r_y, 2);
    double exp_11 = std::pow(q_w, 2)*r_z - q_w*q_z*r_w + std::pow(q_x, 2)*r_z -
        q_x*q_z*r_x + std::pow(q_y, 2)*r_z - q_y*q_z*r_y;
    double exp_12 = q_w*r_w*r_y + q_x*r_x*r_y - q_y*std::pow(r_w, 2) -
        q_y*std::pow(r_x, 2) - q_y*std::pow(r_z, 2) + q_z*r_y*r_z;
    double exp_13 = q_w*r_w*r_x - q_x*std::pow(r_w, 2) - q_x*std::pow(r_y, 2) -
        q_x*std::pow(r_z, 2) + q_y*r_x*r_y + q_z*r_x*r_z;

    // Define the Hessian elements.
    double H_ww = 8*(-(2*q_w*(exp_7)*(exp_2 + exp_3) -
        (q_x*r_x + q_y*r_y + q_z*r_z)*(exp_4)*(exp_2 + exp_3) +
        (exp_4)*(q_w*std::pow(r_x, 2) + q_w*std::pow(r_y, 2) +
        q_w*std::pow(r_z, 2) - q_x*r_w*r_x - q_y*r_w*r_y - q_z*r_w*r_z)*
        (exp_7))*(exp_2 + exp_3)*exp_1 + std::pow(exp_7, 2)*(exp_6))/(exp_5);
    double H_xx = 8*((2*q_x*(exp_9)*(exp_2 + exp_3) +
        (q_w*r_w + q_y*r_y + q_z*r_z)*(exp_4)*(exp_2 + exp_3) -
        (exp_4)*(exp_9)*(exp_13))*(exp_2 + exp_3)*exp_1 +
        std::pow(exp_9, 2)*(exp_6))/(exp_5);
    double H_yy = 8*((2*q_y*(exp_8)*(exp_2 + exp_3) +
        (q_w*r_w + q_x*r_x + q_z*r_z)*(exp_4)*(exp_2 + exp_3) -
        (exp_4)*(exp_8)*(exp_12))*(exp_2 + exp_3)*exp_1 +
        std::pow(exp_8, 2)*(exp_6))/(exp_5);
    double H_zz = 8*((2*q_z*(exp_11)*(exp_2 + exp_3) +
        (q_w*r_w + q_x*r_x + q_y*r_y)*(exp_4)*(exp_2 + exp_3) -
        (exp_4)*(exp_11)*((exp_10)))*(exp_2 + exp_3)*exp_1 +
        std::pow(exp_11, 2)*(exp_6))/(exp_5);
    double H_wx = 8*((-2*q_x*(exp_7)*(exp_2 + exp_3) +
        (q_w*r_x - 2*q_x*r_w)*(exp_4)*(exp_2 + exp_3) +
        (exp_4)*(exp_7)*(exp_13))*(exp_2 + exp_3)*exp_1 -
        (exp_9)*(exp_7)*(exp_6))/(exp_5);
    double H_wy = 8*((-2*q_y*(exp_7)*(exp_2 + exp_3) +
        (q_w*r_y - 2*q_y*r_w)*(exp_4)*(exp_2 + exp_3) +
        (exp_4)*(exp_7)*(exp_12))*(exp_2 + exp_3)*exp_1 -
        (exp_8)*(exp_7)*(exp_6))/(exp_5);
    double H_wz = 8*((-2*q_z*(exp_7)*(exp_2 + exp_3) +
        (q_w*r_z - 2*q_z*r_w)*(exp_4)*(exp_2 + exp_3) +
        (exp_4)*(exp_7)*((exp_10)))*(exp_2 + exp_3)*exp_1 -
        (exp_11)*(exp_7)*(exp_6))/(exp_5);
    double H_xy = 8*((2*q_y*(exp_9)*(exp_2 + exp_3) +
        (q_x*r_y - 2*q_y*r_x)*(exp_4)*(exp_2 + exp_3) -
        (exp_4)*(exp_9)*(exp_12))*(exp_2 + exp_3)*exp_1 +
        (exp_9)*(exp_8)*(exp_6))/(exp_5);
    double H_xz = 8*((2*q_z*(exp_9)*(exp_2 + exp_3) +
        (q_x*r_z - 2*q_z*r_x)*(exp_4)*(exp_2 + exp_3) -
        (exp_4)*(exp_9)*((exp_10)))*(exp_2 + exp_3)*exp_1 +
        (exp_9)*(exp_11)*(exp_6))/(exp_5);
    double H_yz = 8*((2*q_z*(exp_8)*(exp_2 + exp_3) +
        (q_y*r_z - 2*q_z*r_y)*(exp_4)*(exp_2 + exp_3) -
        (exp_4)*(exp_8)*((exp_10)))*(exp_2 + exp_3)*exp_1 +
        (exp_8)*(exp_11)*(exp_6))/(exp_5);

    // Define the Hessian.
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(4, 4);
    hessian.col(0) << H_ww, H_wx, H_wy, H_wz;
    hessian.col(1) << H_wx, H_xx, H_xy, H_xz;
    hessian.col(2) << H_wy, H_xy, H_yy, H_yz;
    hessian.col(3) << H_wz, H_xz, H_yz, H_zz;

    return hessian;
}

} // namespace systems
} // namespace dairlib
