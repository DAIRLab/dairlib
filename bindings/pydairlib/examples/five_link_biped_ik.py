import numpy as np
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph, CoulombFriction
from pydairlib.common import FindResourceOrThrow
from scipy.spatial.transform import Rotation as R
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
from pydrake.geometry import SceneGraph, DrakeVisualizer, HalfSpace, Box
from pydairlib.multibody import *
from pydrake.math import RigidTransform

def main():
    builder = DiagramBuilder()
    drake_sim_dt = 1e-5
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, drake_sim_dt)
    Parser(plant).AddModelFromFile(
        FindResourceOrThrow('examples/impact_invariant_control/five_link_biped.urdf'))
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"), RigidTransform())
    plant.Finalize()

    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()

    world = plant.world_frame()
    context = plant.CreateDefaultContext()

    pos_map = makeNameToPositionsMap(plant)


    plant.num_positions()
    plant.num_velocities()

    ik_solver = InverseKinematics(plant, context, with_joint_limits=True)
    left_foot_frame = plant.GetBodyByName('left_foot').body_frame()
    right_foot_frame = plant.GetBodyByName('right_foot').body_frame()
    torso_frame = plant.GetBodyByName('torso').body_frame()
    left_foot_position_W = np.array([0, 0, 0])
    right_foot_position_W = np.array([0.1, 0, 0.1])
    torso_position = np.array([0, 0, 0.7])
    # ik_solver.AddPositionConstraint(torso_frame, np.array([0, 0, 0]), world, torso_position, torso_position)
    ik_solver.prog().AddLinearEqualityConstraint(ik_solver.q()[pos_map['planar_roty']], np.array([0.25]))
    ik_solver.AddPositionConstraint(left_foot_frame, np.zeros(3), world, left_foot_position_W, left_foot_position_W)
    ik_solver.AddPositionConstraint(torso_frame, np.array([0, 0, 0]), world, torso_position, torso_position)
    # ik_solver.AddPositionConstraint(right_foot_frame, np.zeros(3), world, right_foot_position_W, right_foot_position_W)
    ik_solver.prog().AddLinearConstraint(ik_solver.q()[pos_map['left_knee_pin']], np.array([0.1]), np.array([5]))
    ik_solver.prog().AddLinearConstraint(ik_solver.q()[pos_map['right_knee_pin']], np.array([0.2]), np.array([5]))
    ik_solver.prog().AddQuadraticCost(np.eye(7), np.zeros(7), ik_solver.q())
    result = Solve(ik_solver.prog())
    print(result.is_success())
    q_sol = plant.GetPositions(context)
    print(q_sol)
    visualizer = MultiposeVisualizer('examples/impact_invariant_control/five_link_biped.urdf', 1, "base")
    visualizer.DrawPoses(plant.GetPositions(context))







if __name__ == '__main__':
    main()