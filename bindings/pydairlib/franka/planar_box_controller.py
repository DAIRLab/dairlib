import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ContactVisualizer,
    DiagramBuilder,
    LeafSystem,
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
    VectorLogSink,
    EventStatus
)

from c3 import *
from parameters.c3_options import get_c3_options

class Controller(LeafSystem):
    def __init__(self, plant, plant_context):
        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort(
            "controller_output", 1, self.CalcOutput
        )
        self.DeclareVectorInputPort("system_state",
                                    plant.num_positions() + plant.num_velocities())
        self.plant = plant
        self.plant_context = plant_context
        lcs_builer = DiagramBuilder()
        self.plant_for_lcs, self.scene_graph_for_lcs = AddMultibodyPlantSceneGraph(
            lcs_builer, 0.0)
        lcs_parser = Parser(self.plant_for_lcs)
        self.passive_block_index = lcs_parser.AddModels(
            "bindings/pydairlib/franka/urdf/passive_block_lcs.sdf")[0]
        self.active_block_index = \
            lcs_parser.AddModels(
                "bindings/pydairlib/franka/urdf/active_block.sdf")[
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


        self.c3_options = get_c3_options()
        if self.c3_options.contact_model == 'anitescu':
            self.contact_model = ContactModel.kAnitescu
        else:
            self.contact_model = ContactModel.kStewartAndTrinkle

        self.num_friction_directions = self.c3_options.num_friction_directions
        self.mu = self.c3_options.mu
        self.dt = self.c3_options.dt
        self.N = self.c3_options.N

        self.u = PiecewisePolynomial(np.array([0]))
        self.last_update_time = -1
        self.c3_solver = None
        self.x_des = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.predicted_x1 = self.x_des
        self.lcs_pred = self.x_des
        Qs = []
        for i in range(self.N + 1):
            Qs.append(1.1 ** i * self.c3_options.Q)
        self.costs = CostMatrices(Qs, self.N * [self.c3_options.R], self.N * [self.c3_options.G],
                                  self.N * [self.c3_options.U])



    def CalcOutput(self, context, output):

        if context.get_time() > self.last_update_time + (0.99 * self.dt):
            x = self.EvalVectorInput(context, 0)
            x0 = x.value()
            u0 = self.u.value(self.u.value(self.u.get_segment_times()[0]))[0]

            x_u = np.hstack((x.value(), u0))
            x_u_ad = InitializeAutoDiff(x_u)

            if context.get_time() // 10 % 2 == 0:  # setting the target
                self.x_des = np.array([0.0, -0.5, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
            else:
                self.x_des = np.array([0.0, 0.5, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0])

            # print("time: ", context.get_time())
            # print("u: ", u0)
            # print("planned x1: ", self.predicted_x1)
            # print("lcs pred x1: ", self.lcs_pred)
            # print("actual x1: ", x.value())
            self.plant_for_lcs.SetPositionsAndVelocities(self.context_for_lcs,
                                                         x0)
            self.plant_for_lcs.get_actuation_input_port().FixValue(
                self.context_for_lcs, u0)
            self.plant_ad.get_actuation_input_port().FixValue(self.context_ad,
                                                              x_u_ad[-1])
            lcs = LinearizePlantToLCS(self.plant_for_lcs, self.context_for_lcs,
                                      self.plant_ad, self.context_ad,
                                      self.contact_geoms,
                                      self.c3_options.num_friction_directions, self.c3_options.mu,
                                      self.c3_options.dt, self.c3_options.N,
                                      self.contact_model)

            if self.c3_solver == None:
                self.c3_solver = C3MIQP(lcs, self.costs,
                                        (self.N + 1) * [self.x_des],
                                        self.c3_options)
                self.c3_solver.UpdateLCS(lcs)
                self.c3_solver.AddLinearConstraint(np.array([[1]]), np.array([self.c3_options.u_horizontal_limits[0]]), np.array([self.c3_options.u_horizontal_limits[1]]), 2)
                self.c3_solver.AddLinearConstraint(np.array([[1, 0, 0, 0, 0, 0, 0, 0]]), np.array([-1.0]), np.array([1.0]), 1)
            else:
                self.c3_solver.UpdateLCS(lcs)
                self.c3_solver.UpdateTarget((self.N + 1) * [self.x_des])
            self.c3_solver.Solve(x0)
            u_sol = self.c3_solver.GetInputSolution()
            x_sol = self.c3_solver.GetStateSolution()
            u_sol = np.array(u_sol)
            x_sol = np.array(x_sol)

            # C3 predicted state at next timestep
            self.predicted_x1 = x_sol[1, :]
            # LCS predicted state at next timestep
            self.lcs_pred = lcs.Simulate(x0, u_sol[0])

            timestamps = context.get_time() + self.dt * np.arange(self.N)
            self.u = PiecewisePolynomial.ZeroOrderHold(timestamps, u_sol.T)
            self.last_update_time = context.get_time()
        output.set_value(self.u.value(context.get_time()))
