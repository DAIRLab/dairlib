import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ContactVisualizer,
    DiagramBuilder,
    ExternallyAppliedSpatialForce,
    LeafSystem,
    List,
    MeshcatVisualizer,
    ModelVisualizer,
    Parser,
    Simulator,
    SpatialForce,
    StartMeshcat,
    RigidTransform,
    Value,
    InitializeAutoDiff,
    ModelInstanceIndex,
    PiecewisePolynomial,
)

from c3 import *


class Controller(LeafSystem):
    def __init__(self, plant, plant_context):
        LeafSystem.__init__(self)
        forces_cls = Value[List[ExternallyAppliedSpatialForce]]
        self.DeclareVectorOutputPort(
            "controller_output", 1, self.CalcOutput
        )
        self.DeclareVectorInputPort("system_state", plant.num_positions() + plant.num_velocities())
        self.plant = plant
        self.plant_context = plant_context
        self.c3_options = C3Options()

        lcs_builer = DiagramBuilder()
        self.plant_for_lcs, self.scene_graph_for_lcs = AddMultibodyPlantSceneGraph(
            lcs_builer, 0.0)
        lcs_parser = Parser(self.plant_for_lcs)
        self.passive_block_index = lcs_parser.AddModels(
            "bindings/pydairlib/franka/urdf/passive_block_lcs.sdf")[0]
        self.active_block_index = \
        lcs_parser.AddModels("bindings/pydairlib/franka/urdf/active_block.sdf")[
            0]
        self.plant_for_lcs.WeldFrames(self.plant_for_lcs.world_frame(),
                                      self.plant_for_lcs.GetFrameByName("base",
                                                                        self.passive_block_index))
        self.plant_for_lcs.Finalize()

        lcs_diagram = lcs_builer.Build()

        diagram_context = lcs_diagram.CreateDefaultContext()
        self.context_for_lcs = lcs_diagram.GetMutableSubsystemContext(
            self.plant_for_lcs, diagram_context)
        self.plant_ad = self.plant_for_lcs.ToAutoDiffXd()
        self.context_ad = self.plant_ad.CreateDefaultContext()

        passive_block_contact_points = self.plant_for_lcs.GetCollisionGeometriesForBody(
            self.plant_for_lcs.GetBodyByName("passive_block",
                                             self.passive_block_index))
        active_block_contact_points = self.plant_for_lcs.GetCollisionGeometriesForBody(
            self.plant_for_lcs.GetBodyByName("active_block",
                                             self.active_block_index))
        self.contact_geoms = list()
        for geom_id in passive_block_contact_points:
            self.contact_geoms.append((active_block_contact_points[0], geom_id))
        self.num_friction_directions = 1
        self.mu = [0.4, 0.4]
        self.dt = 0.05
        self.N = 10
        # self.contact_model = ContactModel.kAnitescu
        self.contact_model = ContactModel.kStewartAndTrinkle
        self.Q = 50 * np.diag(np.array([1000, 1000, 500, 500, 10, 10, 1, 1]))
        self.R = 10 * np.eye(1)
        # self.G = 0.1 * np.diag(np.hstack((1 * np.ones(4), 1000 * np.ones(4), 1000 * np.ones(4), 1 * np.ones(1))))
        # self.U = 0.1 * np.diag(np.hstack((1 * np.ones(4), 1 * np.ones(4), 1 * np.ones(4), 10000 * np.ones(1))))

        self.G = 10 * np.diag(np.hstack((1 * np.ones(4), 1000 * np.ones(4), 1000 * np.ones(2), 1000 * np.ones(6), 1 * np.ones(1))))
        self.U = 10 * np.diag(np.hstack((10 * np.ones(4), 1 * np.ones(4), 1 * np.ones(8), 10000000 * np.ones(1))))
        self.c3_options.admm_iter = 5
        self.c3_options.rho = 0
        self.c3_options.rho_scale = 4
        self.c3_options.num_threads = 1
        self.c3_options.delta_option = 1
        # self.c3_options.contact_model = "anitescu"
        self.c3_options.contact_model = "stewart_and_trinkle"
        self.c3_options.warm_start = 1
        self.c3_options.use_predicted_x0 = 1
        self.c3_options.end_on_qp_step = 0
        self.c3_options.use_robust_formulation = 0
        self.c3_options.solve_time_filter_alpha = 0.95
        self.c3_options.publish_frequency = 0
        self.c3_options.u_horizontal_limits = np.array([-100, 100])
        self.c3_options.u_vertical_limits = np.array([0, 0])
        self.c3_options.workspace_limits = [np.array([1, 0, 0, -0.5, 0.5])] # unused
        self.c3_options.workspace_margins = 0.05
        self.c3_options.N = self.N
        self.c3_options.gamma = 1.0
        self.c3_options.mu = self.mu
        self.c3_options.dt = self.dt
        self.c3_options.solve_dt = 0.0
        self.c3_options.num_friction_directions = self.num_friction_directions
        self.c3_options.num_contacts = len(self.mu)
        self.c3_options.Q = self.Q
        self.c3_options.R = self.R
        self.c3_options.G = self.G
        self.c3_options.U = self.U
        self.u = PiecewisePolynomial(np.array([0]))
        self.last_update_time = -1
        self.c3_solver = None
        self.x_des = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.predicted_x1 = self.x_des
        self.lcs_pred = self.x_des
        Qs = []
        for i in range(self.N + 1):
            Qs.append(1.0 ** i * self.Q)
        # costs = CostMatrices((self.N + 1) * [self.Q], self.N * [self.R], self.N * [self.G], self.N * [self.U])
        self.costs = CostMatrices(Qs, self.N * [self.R], self.N * [self.G], self.N * [self.U])

    def CalcOutput(self, context, output):


        if context.get_time() > self.last_update_time + (1.0 * self.dt):
            x = self.EvalVectorInput(context, 0)
            x0 = x.value()
            # u0 = np.zeros(1)
            u0 = self.u.value(self.u.value(self.u.get_segment_times()[0]))[0]
            print("u0: ", u0)
            x_u = np.hstack((x.value(), u0))
            x_u_ad = InitializeAutoDiff(x_u)

            if context.get_time() // 10 % 2 == 0:
                self.x_des =  np.array([0.0, -0.3, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
            else:
                self.x_des =  np.array([0.0, 0.3, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
            print("time: ", context.get_time())
            # print("current cost: ", (x.value() - self.x_des).T @ (x.value() - self.x_des))
            # print("desired x: ", self.x_des)
            print("planned x1: ", self.predicted_x1)
            print("lcs pred x1: ", self.lcs_pred)
            print("actual x1: ", x.value())
            # import pdb; pdb.set_trace()
            self.plant_for_lcs.SetPositionsAndVelocities(self.context_for_lcs, x0)
            self.plant_for_lcs.get_actuation_input_port().FixValue(self.context_for_lcs, u0)
            self.plant_ad.get_actuation_input_port().FixValue(self.context_ad, x_u_ad[-1])
            lcs = LinearizePlantToLCS(self.plant_for_lcs, self.context_for_lcs,
                                      self.plant_ad, self.context_ad, self.contact_geoms,
                                      self.num_friction_directions, self.mu, self.dt, self.N,
                                      self.contact_model)

            if self.c3_solver == None:
                self.c3_solver = C3MIQP(lcs, self.costs, (self.N + 1) * [self.x_des], self.c3_options)
            else:
                self.c3_solver.UpdateLCS(lcs)
                self.c3_solver.UpdateTarget((self.N + 1) * [self.x_des])
            self.c3_solver.Solve(x0)
            u_sol = self.c3_solver.GetInputSolution()
            x_sol = self.c3_solver.GetStateSolution()
            w_sol = self.c3_solver.GetDualWSolution()
            delta_sol = self.c3_solver.GetDualDeltaSolution()
            z_sol = self.c3_solver.GetFullSolution()

            u_sol = np.array(u_sol)
            x_sol = np.array(x_sol)
            w_sol = np.array(w_sol)
            delta_sol = np.array(delta_sol)
            z_sol = np.array(z_sol)

            self.predicted_x1 = x_sol[1, :]
            self.lcs_pred = lcs.Simulate(x0, u_sol[0])

            timestamps = context.get_time() + self.dt * np.arange(self.N)
            self.u = PiecewisePolynomial.ZeroOrderHold(timestamps, u_sol.T)
            # self.u = PiecewisePolynomial.FirstOrderHold(timestamps, np.array(u_sol).T)
            self.last_update_time = context.get_time()
        # import pdb; pdb.set_trace()
        # print(self.u.value(self.u.get_segment_times()[1]))
        output.set_value(self.u.value(self.u.get_segment_times()[0]))
        # output.set_value(np.array([-3]))


def main():
    np.set_printoptions(3, threshold=8, suppress=True)
    # np.set_printoptions(3, threshold=8)
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)
    parser = Parser(plant)
    passive_block_index = \
    parser.AddModels("bindings/pydairlib/franka/urdf/passive_block.sdf")[0]
    active_block_index = \
    parser.AddModels("bindings/pydairlib/franka/urdf/active_block.sdf")[0]
    offset = RigidTransform(np.array([0.0, 0.0, 0.0]))
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("base", passive_block_index), offset)
    # plant.WeldFrames(plant.world_frame(),
    #                  plant.GetFrameByName("base", active_block_index))
    plant.Finalize()
    plant_context = plant.CreateDefaultContext()
    meshcat = StartMeshcat()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    ContactVisualizer.AddToBuilder(builder, plant, meshcat)
    controller = builder.AddSystem(Controller(plant, plant_context))
    builder.Connect(controller.get_output_port(),
                    plant.get_actuation_input_port())
    builder.Connect(plant.get_state_output_port(),
                    controller.get_input_port())
    diagram = builder.Build()

    meshcat.SetCameraPose(np.array([0, -3.0, 0.5]), np.array([0, 0.0, 0.0]))
    simulator = Simulator(diagram)
    context = simulator.get_context()
    plant.SetPositions(diagram.GetMutableSubsystemContext(plant,
                                                          simulator.get_mutable_context()),
                       np.array([0.0, 0.0, 0.1, 0]))  # active x, passive: x, z, theta
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    meshcat.StartRecording()
    simulator.AdvanceTo(40.0)
    meshcat.StopRecording()
    meshcat.PublishRecording()
    with open("planar_box_visualization.html", "r+") as f:
        f.write(meshcat.StaticHtml())
    print("done simulating")


if __name__ == '__main__':
    main()
