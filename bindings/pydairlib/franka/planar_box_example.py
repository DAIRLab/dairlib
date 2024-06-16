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
            # contact_geoms.append([geom_id, passive_block_contact_points[0]])
            self.contact_geoms.append((active_block_contact_points[0], geom_id))
        self.num_friction_directions = 2
        self.mu = [0.4, 0.4]
        self.dt = 0.5
        self.N = 5
        self.contact_model = ContactModel.kAnitescu
        # self.contact_model = ContactModel.kStewartAndTrinkle
        self.Q = np.diag(np.array([100, 100, 1, 1, 1, 1, 1, 1]))
        self.R = 10 * np.eye(1)
        self.G = 0.5 * np.eye(8 + 8 + 1)
        self.U = 0.5 * np.eye(8 + 8 + 1)
        # self.G = np.eye(8 + 12 + 1)
        # self.U = np.eye(8 + 12 + 1)
        self.c3_options.admm_iter = 8
        self.c3_options.rho = 2
        self.c3_options.rho_scale = 2
        self.c3_options.num_threads = 4
        self.c3_options.delta_option = 1
        self.c3_options.contact_model = "anitescu"
        self.c3_options.warm_start = 0
        self.c3_options.use_predicted_x0 = 0
        self.c3_options.end_on_qp_step = 0
        self.c3_options.use_robust_formulation = 0
        self.c3_options.solve_time_filter_alpha = 0.95
        self.c3_options.publish_frequency = 0
        self.c3_options.u_horizontal_limits = np.array([-10, 10])
        self.c3_options.u_vertical_limits = np.array([0, 0])
        self.c3_options.workspace_limits = [np.array([1, 1, 1, -1000, 1000])]
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


    def CalcOutput(self, context, output):
        x = self.EvalVectorInput(context, 0)
        x_u = np.hstack((x.value(), np.array([0])))
        x_u_ad = InitializeAutoDiff(x_u)
        self.plant_for_lcs.SetPositionsAndVelocities(self.context_for_lcs, x.value())
        self.plant_for_lcs.get_actuation_input_port().FixValue(self.context_for_lcs, np.array([0]))
        self.plant_ad.get_actuation_input_port().FixValue(self.context_ad, x_u_ad[-1])
        # if context.get_time() > self.last_update_time + (self.N * self.dt):
        if context.get_time() > self.last_update_time + (self.dt):
            print("cost: ", (x.value() - self.x_des).T @ self.Q @ (x.value() - self.x_des))
            print((x.value() - self.x_des)[:2])
            print("time: ", context.get_time())
            # import pdb; pdb.set_trace()
            lcs = LinearizePlantToLCS(self.plant_for_lcs, self.context_for_lcs,
                                      self.plant_ad, self.context_ad, self.contact_geoms,
                                      self.num_friction_directions, self.mu, self.dt, self.N,
                                      self.contact_model)
            Qs = []
            for i in range(self.N + 1):
                Qs.append(1.0 ** i * self.Q)
            # costs = CostMatrices((self.N + 1) * [self.Q], self.N * [self.R], self.N * [self.G], self.N * [self.U])
            costs = CostMatrices(Qs, self.N * [self.R], self.N * [self.G], self.N * [self.U])
            if self.c3_solver == None:
                self.c3_solver = C3MIQP(lcs, costs, (self.N + 1) * [self.x_des], self.c3_options)
            else:
                print("updating target")
                self.c3_solver.UpdateTarget((self.N + 1) * [self.x_des])
            self.c3_solver.Solve(x.value())
            u_sol = self.c3_solver.GetInputSolution()
            x_sol = self.c3_solver.GetStateSolution()
            print(np.array(x_sol)[:, 0])
            print(np.array(x_sol)[:, 1])
            timestamps = context.get_time() + self.dt * np.arange(self.N)
            self.u = PiecewisePolynomial.ZeroOrderHold(timestamps, np.array(u_sol).T)
            self.last_update_time = context.get_time()
        output.set_value(1 * self.u.value(context.get_time()))


def main():
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
                       np.array([0.0, 0.3, 0.1, 0]))  # active x, passive: x, z, theta
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(60.0)


if __name__ == '__main__':
    main()
