//
// Created by brian on 11/10/20.
//

#include <memory>
#include <utility>
#include "drake/common/autodiff.h"
#include "drake/math/autodiff.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "multibody/multibody_utils.h"
#include "multibody/coriolis_matrix_calculator.h"
#include "common/find_resource.h"

namespace dairlib {
    namespace multibody {
        using drake::AutoDiffXd;
        using drake::AutoDiffVecXd;
        using drake::MatrixX;
        using drake::VectorX;
        using drake::multibody::MultibodyPlant;
        using drake::multibody::Parser;
        using drake::geometry::SceneGraph;

        int CoriolisTestMain(int argc, char **argv) {
            SceneGraph<double> scene_graph;
            std::string full_name = FindResourceOrThrow(
                    "examples/Cassie/urdf/cassie_v2.urdf");

            MultibodyPlant<double> plant(0);
            Parser parser(&plant, &scene_graph);
            parser.AddModelFromFile(full_name);
            plant.Finalize();

            std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
                    drake::systems::System<double>::ToAutoDiffXd(plant);

            int nv = plant_ad->num_velocities();
            int nq = plant_ad->num_positions();

            CoriolisMatrixCalculator coriolis(*plant_ad);

            auto context_ad = plant_ad->CreateDefaultContext();
            auto context = plant.CreateDefaultContext();

            MatrixX<AutoDiffXd> C = Eigen::MatrixXd::Zero(nv, nv);
            VectorX<double> xd = 3*VectorX<double>::Random(nq+nv);
            xd[0] = 1;
            xd[1] = 0;
            xd[2] = 0;
            xd[3] = 0;

            context->SetContinuousState(xd);

            VectorX<AutoDiffXd> x = VectorX<AutoDiffXd>::Zero(nv+nq);
            x.topRows(nq) = xd.topRows(nq);
            x.bottomRows(nv) = drake::math::initializeAutoDiff(xd.bottomRows(nv));

            for (int i = 0; i < 3; i++) {
                auto start = std::chrono::high_resolution_clock::now();
                context_ad->SetContinuousState(x);
                coriolis.CalcCoriolisAutoDiff(context_ad, C);
                auto stop = std::chrono::high_resolution_clock::now();

                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                std::cout << "Calculation Time :" << duration.count() << " microseconds\n";
            }
            VectorX<double> cv = VectorX<double>::Zero(nv);
            plant.CalcBiasTerm(*context, &cv);

            VectorX<AutoDiffXd> cv_ad = VectorX<AutoDiffXd>::Zero(nv);
            cv_ad = C*x.bottomRows(nv);

            std::cout << "C matrix:" << std::endl;
            std::cout << C << std::endl;

            std::cout << "Cv:\n";
            std::cout << cv << std::endl;
            std::cout << "C*v\n";
            std::cout << cv_ad << std::endl;

            std::cout << "Difference: " << std::endl;
            std::cout << cv - cv_ad << std::endl;


            return 0;
        }


    }
}

int main(int argc, char **argv) {
    return dairlib::multibody::CoriolisTestMain(argc, argv);
}