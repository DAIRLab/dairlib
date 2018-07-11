#include "cassie_sysid_utils.h"

namespace dairlib{

using drake::trajectories::PiecewisePolynomial;

// Simulation parameters.
DEFINE_double(timestep, 1e-5, "The simulator time step (s)");
DEFINE_double(youngs_modulus, 1e8, "The contact model's Young's modulus (Pa)");
DEFINE_double(us, 0.7, "The static coefficient of friction");
DEFINE_double(ud, 0.7, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(dissipation, 2, "The contact model's dissipation (s/m)");
DEFINE_double(contact_radius, 2e-4,
              "The characteristic scale of contact patch (m)");
DEFINE_string(simulation_type, "compliant", "The type of simulation to use: "
              "'compliant' or 'timestepping'");
DEFINE_double(dt, 1e-3, "The step size to use for "
              "'simulation_type=timestepping' (ignored for "
              "'simulation_type=compliant'");

//TODO(maki): Generalize it so that we're not relying on makeFixedBaseCassieTreePointer();

void setPlantMaterialProperties(drake::systems::RigidBodyPlant<double> *rbp){
	drake::systems::CompliantMaterial default_material;
	default_material.set_youngs_modulus(FLAGS_youngs_modulus)
		.set_dissipation(FLAGS_dissipation)
		.set_friction(FLAGS_us, FLAGS_ud);
	rbp->set_default_compliant_material(default_material);
	drake::systems::CompliantContactModelParameters model_parameters;
	model_parameters.characteristic_radius = FLAGS_contact_radius;
	model_parameters.v_stiction_tolerance = FLAGS_v_tol;
	rbp->set_contact_model_parameters(model_parameters);
}

PiecewisePolynomial<double> getSimulationTrajectoryOverTime(PiecewisePolynomial<double> traj_input, double simulationTime){

	std::unique_ptr<RigidBodyTree<double>> tree = makeFixedBaseCassieTreePointer();
	drake::systems::DiagramBuilder<double> builder;

	//Add Cassie RigidBodyPlant System
	auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(tree), FLAGS_dt);
	setPlantMaterialProperties(plant);

	// Create state publisher following trajectory
	auto traj = builder.AddSystem<drake::systems::TrajectorySource<double>>(traj_input);
	auto logger = builder.AddSystem<drake::systems::SignalLogger<double>>(plant->get_output_port(0).size());
	logger->set_publish_period(0.001);

	//connecting trajectory state to Cassie
	builder.Connect(traj->get_output_port(), plant->get_input_port(0));
	builder.Connect(plant->state_output_port(), logger->get_input_port());

	auto diagram = builder.Build();

	drake::systems::Simulator<double> simulator(*diagram);

	simulator.set_publish_every_time_step(false);
	simulator.set_publish_at_initialization(false);
	simulator.set_target_realtime_rate(1.0);
	simulator.Initialize();

	simulator.StepTo(simulationTime);

	const auto& dataTimes = logger->sample_times();
	const auto& data = logger->data();

	return PiecewisePolynomial<double>::FirstOrderHold(dataTimes, data); 
}

drake::VectorX<double> getDerivativePredictionAtTime(double time, PiecewisePolynomial<double> trajectory, Eigen::Matrix<double, 32, 1> x){
	//x should be a vector of 16 generalized positions and 16 generalized velocities.

	std::unique_ptr<RigidBodyTree<double>> tree = makeFixedBaseCassieTreePointer();
	drake::systems::RigidBodyPlant<double> rbp(std::move(tree));
	setPlantMaterialProperties(&rbp);

	auto context_rbp = rbp.CreateDefaultContext();
	auto derivatives = rbp.AllocateTimeDerivatives();

    context_rbp->get_mutable_continuous_state_vector().SetFromVector(x);
   	auto& u_rbp = context_rbp->FixInputPort(0, Eigen::Matrix<double, 10, 1>::Zero());

	u_rbp.GetMutableVectorData<double>()->SetFromVector(trajectory.value(time));
	rbp.CalcTimeDerivatives(*context_rbp, derivatives.get());

	return derivatives->CopyToVector();
}

}	//Namespace dairlib