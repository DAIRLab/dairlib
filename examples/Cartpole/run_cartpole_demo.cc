#include <memory>

#include <gflags/gflags.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/geometry_visualization.h>

#include "drake/common/drake_assert.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/primitives/affine_system.h"
#include "common/find_resource.h"
#include "examples/Cartpole/lqr_controller.h"

DEFINE_double(target_realtime_rate, 1.0, "Simulation rate relative to realtime");
DEFINE_double(simulation_time, 10, "How long to simulate");
DEFINE_double(x0, 0.0, "initial x value");
DEFINE_double(theta0, 0.0, "Initial angle");

namespace dairlib {

    using std::cout;
    using std::endl;

    using Eigen::Matrix3d;
    using Eigen::MatrixXd;
    using Eigen::Vector3d;
    using Eigen::VectorXd;

    using drake::geometry::SceneGraph;
    using drake::multibody::MultibodyPlant;
    using drake::multibody::Parser;
    using drake::systems::DiagramBuilder;
    using drake::systems::Context;
    using drake::systems::Simulator;
    using drake::systems::LinearSystem;


    int DoMain(int argc, char* argv[]) {
        // Parse command line flags
        gflags::ParseCommandLineFlags(&argc, &argv, false);

        // Simulation time should be greater than 0
        DRAKE_DEMAND(FLAGS_simulation_time > 0);

        std::cout << "Building Diagram ... \n";

        // Add drake diagram builder
        DiagramBuilder<double> builder;

        SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
        scene_graph.set_name("scene_graph");

        MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant<double>>(0);
        plant.set_name("cart_pole");
        plant.RegisterAsSourceForSceneGraph(&scene_graph);

        std::string full_name = FindResourceOrThrow("examples/Cartpole/urdf/cartpole.sdf");
        Parser parser(&plant, &scene_graph);
        parser.AddModelFromFile(full_name);
        std::cout << "model added... \n";

        std::cout << "Plant welded... \n";

        plant.Finalize();
        std::cout << "Plant finalized... \n";
        auto context = plant.CreateDefaultContext();

        // Setup LQR controller
        Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);
        plant.get_actuation_input_port().FixValue(context.get(), u0);

        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
        x0[0] = FLAGS_x0;
        x0[1] = FLAGS_theta0 + M_PI;
        context->SetContinuousState(x0);

        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4);
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);
        Eigen::MatrixXd N;
        Q(0, 0) = 1.0;
        Q(1, 1) = 100.0;
        Q(3, 3) = 10.0;

        const double M = 1.0;
        const double m = 1.0;
        const double g = 9.81;
        const double L = 1.0;

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 4);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4,1);
        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);
        Eigen::VectorXd offset = Eigen::VectorXd::Zero(4);
        offset[1] = M_PI;


        A(0,2) = 1;
        A(1,3) = 1;
        A(2, 1) = m*g/M;
        A(2, 2) = 1;
        A(3,1) = (M+m)*g/(M*L);

        B(2,0) = 1/(M);
        B(3, 0) = 1/(M*L);


        LQRController *lqr;
        lqr = builder.AddSystem<LQRController>(&plant, A, B, Q, R, offset);

        std::cout << "Connecting LQR controller... \n";
        builder.Connect(plant.get_state_output_port(), lqr->get_lqr_input_port());
        builder.Connect(lqr->get_lqr_output_port(), plant.get_actuation_input_port());

        std::cout << "Checking plant source ID...\n";
        DRAKE_DEMAND(!!plant.get_source_id());

        builder.Connect(plant.get_geometry_poses_output_port(), scene_graph.get_source_pose_port(plant.get_source_id().value()));
        builder.Connect(scene_graph.get_query_output_port(), plant.get_geometry_query_input_port());

        drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);

        auto diagram = builder.Build();
        std::unique_ptr<Context<double>> diagram_context = diagram->CreateDefaultContext();

        Context<double>& plant_context = diagram->GetMutableSubsystemContext(plant, diagram_context.get());

        Eigen::VectorXd pos = Eigen::VectorXd::Zero(2);
        pos[0] = FLAGS_x0;
        pos[1] = FLAGS_theta0 + M_PI;
        plant.SetPositions(&plant_context, pos);

        Simulator<double> simulator(*diagram, std::move(diagram_context));
        simulator.set_publish_every_time_step(false);
        simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
        simulator.Initialize();
        simulator.AdvanceTo(FLAGS_simulation_time);
        return 0;
    }
}

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }