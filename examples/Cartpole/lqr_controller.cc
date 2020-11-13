//
// Created by brian on 10/16/20.
//
#include <drake/systems/controllers/linear_quadratic_regulator.h>
#include "lqr_controller.h"

dairlib::LQRController::LQRController(const drake::multibody::MultibodyPlant<double>* plant,
                                      Eigen::MatrixXd A, Eigen::MatrixXd B,
                                      Eigen::MatrixXd Q, Eigen::MatrixXd R,
                                      Eigen::VectorXd offset) {

    lqr_input_port_ = this->DeclareVectorInputPort("lqr_input", drake::systems::BasicVector<double>(
            plant->num_positions() + plant->num_velocities())).get_index();

    lqr_output_port_ = this->DeclareVectorOutputPort("lqr_output", drake::systems::BasicVector<double>(1),
            &LQRController::CalcOutputForce).get_index();

    lqr_result_ = drake::systems::controllers::LinearQuadraticRegulator(A, B, Q, R);

    Eigen::IOFormat OctaveFmt(5, 0, ", ", ";\n", "", "", "[", "]");
    std::cout << "LQR GAINS: " << lqr_result_.K.format(OctaveFmt) << std::endl;

    state_offset_ = offset;

}

void dairlib::LQRController::CalcOutputForce(const drake::systems::Context<double> &context,
                                             drake::systems::BasicVector<double> *output) const {
    drake::VectorX<double> state = this->EvalVectorInput(context, lqr_input_port_)->CopyToVector() - state_offset_;
    output->set_value(-lqr_result_.K*state);
}