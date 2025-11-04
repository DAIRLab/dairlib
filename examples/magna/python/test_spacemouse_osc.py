import numpy as np
import time
from pydrake.all import (
    PiecewisePolynomial,
    DiagramBuilder,
    TrajectorySource,
    LcmPublisherSystem,
    DrakeLcm,
    AbstractValue,
    Simulator,
    ZeroOrderHold,
    TriggerType,
    MultibodyPlant,
    Parser,
    RigidTransform,
    RotationMatrix,
    RollPitchYaw,
    FixedOffsetFrame,
)
import lcm
from pydrake.all import LeafSystem, BasicVector
from dairlib import lcmt_robot_output, lcmt_franka_cartesian_pose


class TrajectoryToCartesianPose(LeafSystem):
    """
    A system that converts a trajectory to a lcmt_franka_cartesian_pose message.
    """

    def __init__(self, num_joints):
        LeafSystem.__init__(self)
        self.num_joints = num_joints
        self.DeclareVectorInputPort("trajectory", BasicVector(self.num_joints))
        self.DeclareAbstractOutputPort(
            "franka_cartesian_pose", lambda: AbstractValue.Make(lcmt_franka_cartesian_pose()), self.CalcOutput
        )

    def CalcOutput(self, context, output):
        trajectory = self.get_input_port().Eval(context)
        command = lcmt_franka_cartesian_pose()
        command.utime = int(time.time() * 1e6)
        command.target_cartesian_pose = trajectory
        output.set_value(command)

robot_output = None
def my_handler(channel, data):
    global robot_output
    robot_output = lcmt_robot_output.decode(data)

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
sub = lc.subscribe("FRANKA_STATE", my_handler)
lc.handle()

# Define the times and control points for the trajectory.
times = [0.0, 10.0, 20.0]

plant = MultibodyPlant(0.0)
parser = Parser(plant)
parser.AddModelsFromUrl("package://drake_models/franka_description/urdf/panda_arm.urdf")
plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("panda_link0"), RigidTransform.Identity()
)
plant.AddFrame(
    FixedOffsetFrame(
        "end_effector_frame", plant.GetBodyByName("panda_link7"), RigidTransform(RotationMatrix(RollPitchYaw(np.pi, 0, 0)), np.array([0, 0, 0.107]))
    )
)
plant.Finalize()

context = plant.CreateDefaultContext()
plant.SetPositions(context, robot_output.position)
ee_pose = plant.CalcRelativeTransform(
    context,
    plant.world_frame(),
    plant.GetFrameByName("end_effector_frame"),
)
J0 = np.concatenate([ee_pose.translation(), ee_pose.rotation().ToRollPitchYaw().vector()]).reshape(-1, 1)
J1 = J0 + np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.5]).reshape(-1, 1)


control_points = np.hstack((J0, J1, J0))
print(control_points)

# Create the trajectory object.
trajectory = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
    times, control_points
)  # Take the derivative to get velocity.

# Create a source that outputs the trajectory.
builder = DiagramBuilder()
traj_source = builder.AddSystem(TrajectorySource(trajectory))
franka_cartesian_pose = builder.AddSystem(TrajectoryToCartesianPose(num_joints=6))
builder.Connect(traj_source.get_output_port(), franka_cartesian_pose.get_input_port())
l = DrakeLcm()
pub = LcmPublisherSystem.Make(
    channel="TARGET_CARTESIAN_POSE",
    lcm_type=lcmt_franka_cartesian_pose,
    lcm=l,
    publish_triggers={TriggerType.kPeriodic},
    publish_period=1/1000.0,
    # use_cpp_serializer=True,
)

# Add a ZeroOrderHold system for state updates.
position_zero_order_hold = builder.AddSystem(ZeroOrderHold(1 / 1000.0, 6))
builder.Connect(
    traj_source.get_output_port(), position_zero_order_hold.get_input_port()
)

builder.AddSystem(pub)
builder.Connect(franka_cartesian_pose.get_output_port(), pub.get_input_port())

diagram = builder.Build()

# Create a default context for the diagram.
diagram_context = diagram.CreateDefaultContext()

simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.Initialize()
simulator.AdvanceTo(20.0)
