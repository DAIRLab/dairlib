import numpy as np
import pydairlib.lcm_trajectory
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.tree import JacobianWrtVariable


def main():
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
    Parser(plant).AddModelFromFile("/home/yangwill/Documents/research/dairlib/examples/five_link_biped/five_link_biped.urdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    plant.mutable_gravity_field().set_gravity_vector(-9.81 * np.array([0, 0, 1]))
    plant.Finalize()

    world = plant.world_frame()

    l_contact_frame = plant.GetBodyByName("left_foot").body_frame()
    r_contact_frame = plant.GetBodyByName("right_foot").body_frame()

    x_trajs = pydairlib.lcm_trajectory.LcmTrajectory()
    x_trajs.loadFromFile(
        "/home/yangwill/Documents/research/projects/hybrid_lqr/saved_trajs/2_step_walking_1_20")

    TXZ = np.array([[1, 0, 0], [0, 0, 1]])
    x0 = np.array([0, 0.778109, 0, -.3112, -.231, 0.427, 0.4689,
    0, 0, 0, 0, 0, 0, 0])

    context = plant.CreateDefaultContext()
    plant.SetPositionsAndVelocities(context, x0)
    pt_on_body = np.zeros(3)

    J_r = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, r_contact_frame, pt_on_body, world, world)
    J_l = TXZ @ plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable.kV, l_contact_frame, pt_on_body, world, world)

    phi_l = plant.CalcPointsPositions(context, l_contact_frame, pt_on_body, world)
    phi_r = plant.CalcPointsPositions(context, l_contact_frame, pt_on_body,
                                      world)

    import pdb; pdb.set_trace()




if __name__ == "__main__":
    main()
