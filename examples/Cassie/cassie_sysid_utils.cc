#include "cassie_sysid_utils.h"

using drake::trajectories::PiecewisePolynomial;
using std::string;

using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;

namespace dairlib{
using systems::lcm::VectorAggregator;

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

PiecewisePolynomial<double> getSimulationTrajectoryOverTime(PiecewisePolynomial<double> traj_input, double simulationTime, Eigen::Matrix<double, 32, 1> initX){

	drake::lcm::DrakeLcm lcm;

	std::unique_ptr<RigidBodyTree<double>> tree = makeFixedBaseCassieTreePointer();
	drake::systems::DiagramBuilder<double> builder;

	if (FLAGS_simulation_type != "timestepping")
	   FLAGS_dt = 0.0;

	//Add Cassie RigidBodyPlant System
	auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(tree), FLAGS_dt);
	setPlantMaterialProperties(plant);


	// Create state publisher following trajectory
	auto traj = builder.AddSystem<drake::systems::TrajectorySource<double>>(traj_input);
	auto logger = builder.AddSystem<drake::systems::SignalLogger<double>>(plant->get_output_port(0).size());
	logger->set_publish_period(0.001);

	auto state_pub = builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>("CASSIE_STATE", &lcm));
  	auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant->get_rigid_body_tree());
  	state_pub->set_publish_period(1.0/200.0);

 	builder.Connect(plant->state_output_port(), state_sender->get_input_port_state());

  	builder.Connect(state_sender->get_output_port(0),
                  state_pub->get_input_port());

	//connecting trajectory state to Cassie
	builder.Connect(traj->get_output_port(), plant->get_input_port(0));
	builder.Connect(plant->state_output_port(), logger->get_input_port());

	auto diagram = builder.Build();

	drake::systems::Simulator<double> simulator(*diagram);

	drake::systems::Context<double>& context_plant = diagram->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());
	context_plant.get_mutable_continuous_state().SetFromVector(initX);

	simulator.set_publish_every_time_step(false);
	simulator.set_publish_at_initialization(false);
	simulator.set_target_realtime_rate(5.0);
	simulator.Initialize();

	simulator.StepTo(simulationTime);

	const auto& dataTimes = logger->sample_times();
	const auto& data = logger->data();

	std::cout << dataTimes.size() << std::endl;
	std::cout << data.cols() << std::endl;

	return PiecewisePolynomial<double>::FirstOrderHold(dataTimes, data); 
}

PiecewisePolynomial<double> getDerivativePredictionAtTime (double time, PiecewisePolynomial<double> utrajectory, PiecewisePolynomial<double> xtrajectory){
	//x should be a vector of 16 generalized positions and 16 generalized velocities.

	std::unique_ptr<RigidBodyTree<double>> tree = makeFixedBaseCassieTreePointer();
	drake::systems::RigidBodyPlant<double> rbp(std::move(tree));
	setPlantMaterialProperties(&rbp);

	auto context_rbp = rbp.CreateDefaultContext();
	auto derivatives = rbp.AllocateTimeDerivatives();

	double timestamp = 0.0;
	
	Eigen::Matrix<double, 32, Eigen::Dynamic> trajectoryMatrix; 
	Eigen::Matrix<double, 1, Eigen::Dynamic> timeMatrix;
	
	auto& u_rbp = context_rbp->FixInputPort(0, Eigen::Matrix<double, 10, 1>::Zero());

	while(timestamp < utrajectory.end_time()){
	    context_rbp->get_mutable_continuous_state_vector().SetFromVector(xtrajectory.value(timestamp).block(0, 0, 32, 1));
	   	//std::cout << "xtrajectory state" << std::endl;
	   	//std::cout << xtrajectory.value(timestamp) << std::endl;
		u_rbp.GetMutableVectorData<double>()->SetFromVector(utrajectory.value(timestamp).block(0, 0, 10, 1));
		//std::cout << "utrajectory state" << std::endl;
		//std::cout << utrajectory.value(timestamp) << std::endl;
		rbp.CalcTimeDerivatives(*context_rbp, derivatives.get());
		//std::cout << "calculated derivatives" << std::endl;
		//std::cout << derivatives->CopyToVector() << std::endl;
		
		Eigen::MatrixXd B(trajectoryMatrix.rows(), trajectoryMatrix.cols() + 1);
		B << trajectoryMatrix, derivatives->CopyToVector();
		trajectoryMatrix = B;

		//std::cout << "/////////////////////" << std::endl;
		//std::cout << derivatives->CopyToVector() << std::endl;

		Eigen::MatrixXd C(timeMatrix.rows(), timeMatrix.cols() + 1);
		C << timeMatrix, timestamp;
		timeMatrix = C;

		timestamp = timestamp + 1e-3;
		//std::cout << timestamp << std::endl;
	}

	std::cout << trajectoryMatrix.cols() << std::endl;
	std::cout << timeMatrix.cols() << std::endl;

	return PiecewisePolynomial<double>::Pchip(timeMatrix, trajectoryMatrix);
}

std::pair<PiecewisePolynomial<double>, PiecewisePolynomial<double>> lcmLogToTrajectory(string filename){

	RigidBodyTree<double> tree;
	buildFixedBaseCassieTree(tree);

	drake::lcm::DrakeLcmLog r_log(filename, false);
	drake::systems::DiagramBuilder<double> builder;

	builder.AddSystem<drake::systems::lcm::LcmLogPlaybackSystem>(&r_log);

	auto state_sub = builder.AddSystem(LcmSubscriberSystem::Make<lcmt_robot_output>("CASSIE_STATE", &r_log));

	auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(tree);

	builder.Connect(state_sub->get_output_port(), state_receiver->get_input_port(0));

	auto state_aggregator = builder.AddSystem<VectorAggregator>(state_receiver->get_output_port(0).size() - 1);
	builder.Connect(state_receiver->get_output_port(0), state_aggregator->get_input_port(0));

	builder.AddSystem<drake::systems::lcm::LcmLogPlaybackSystem>(&r_log);

	auto input_sub = builder.AddSystem(LcmSubscriberSystem::Make<lcmt_robot_input>("CASSIE_INPUT", &r_log));

	auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(tree);

	builder.Connect(input_sub->get_output_port(), input_receiver->get_input_port(0));

	auto input_aggregator = builder.AddSystem<VectorAggregator>(input_receiver->get_output_port(0).size() - 1);
	builder.Connect(input_receiver->get_output_port(0), input_aggregator->get_input_port(0));

	auto diagram = builder.Build();
	drake::systems::Simulator<double> sim(*diagram); 

	while(r_log.GetNextMessageTime() < std::numeric_limits<double>::infinity()){
		sim.StepTo(r_log.GetNextMessageTime()); 
		r_log.DispatchMessageAndAdvanceLog(r_log.GetNextMessageTime());
	}


	auto statematrix = state_aggregator->BuildMatrixFromVectors();
	auto inputmatrix = input_aggregator->BuildMatrixFromVectors();
 	
	auto statetimematrix = (state_aggregator->BuildTimestampVector());
	auto inputtimematrix = (input_aggregator->BuildTimestampVector());

	statetimematrix = statetimematrix.transpose();
	inputtimematrix = inputtimematrix.transpose();
/*

	std::cout << "@@@@@@@@@@@" << std::endl;
	std::cout << matrix.cols() << std::endl;
	std::cout << matrix.rows() << std::endl;
	std::cout << timematrix.cols() << std::endl;
	std::cout << timematrix.rows() << std::endl;
*/


	auto statetraj = PiecewisePolynomial<double>::FirstOrderHold(statetimematrix, statematrix);
	auto inputtraj = PiecewisePolynomial<double>::FirstOrderHold(inputtimematrix, inputmatrix);

	std::cout << "@@@@@@@@@@@" << std::endl;
	std::cout << statetraj.start_time() << std::endl;
	std::cout << statetraj.end_time() << std::endl;
	std::cout << inputtraj.start_time() << std::endl;
	std::cout << inputtraj.end_time() << std::endl;

	std::pair<PiecewisePolynomial<double>, PiecewisePolynomial<double>> retval (statetraj, inputtraj);

	return retval;
}

void PlotSimulationErrorTrajectory(string xLogPath, string uLogPath){
	
	auto trajs = lcmLogToTrajectory(xLogPath);
	auto xtraj = trajs.first;
	auto utraj = trajs.second;

	auto xdottraj = xtraj.derivative(1);
	auto udottraj = utraj.derivative(1);

	std::cout << xdottraj.end_time() - xdottraj.start_time() << std::endl;

	auto xtraj_simulated = getSimulationTrajectoryOverTime(utraj, (xdottraj.end_time() - xdottraj.start_time()) / 1e6, xtraj.value(0));
	auto xdottraj_simulated = xtraj_simulated.derivative();
}

}	//Namespace dairlib