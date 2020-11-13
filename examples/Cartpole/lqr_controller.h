//
// Created by brian on 10/16/20.
//
#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {

    class LQRController : public drake::systems::LeafSystem<double> {
    public:
        LQRController(const drake::multibody::MultibodyPlant<double> *plant,
                      Eigen::MatrixXd A, Eigen::MatrixXd B,
                      Eigen::MatrixXd Q, Eigen::MatrixXd R,
                      Eigen::VectorXd offset);

        const drake::systems::OutputPort<double>& get_lqr_output_port() const {
            return this->get_output_port(lqr_output_port_);
        }
        const drake::systems::InputPort<double>& get_lqr_input_port() const {
            return this->get_input_port(lqr_input_port_);
        }

    private:

        void CalcOutputForce(const drake::systems::Context<double>& context,
                             drake::systems::BasicVector<double>* output) const;

        int lqr_output_port_;
        int lqr_input_port_;
        Eigen::VectorXd state_offset_;
        drake::systems::controllers::LinearQuadraticRegulatorResult lqr_result_;
    };
}
