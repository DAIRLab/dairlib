import numpy as np

from pydairlib.cassie.cassie_utils import addCassieMultibody

from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder


from cassie_lip_ik import CassieLipIk

cassie_urdf = "examples/Cassie/urdf/cassie_v2.urdf"
cassie_urdf_no_springs = "examples/Cassie/urdf/cassie_fixed_springs.urdf"


def make_plant_and_context(floating_base=True, springs=True):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    if (springs):
        addCassieMultibody(plant, scene_graph,
                           floating_base, cassie_urdf, True, True)
    else:
        addCassieMultibody(plant, scene_graph,
                           floating_base, cassie_urdf_no_springs, False, True)

    plant.Finalize()
    return plant, plant.CreateDefaultContext()


def get_toe_frames_and_points(plant):
    front_contact_pt = np.array((-0.0457, 0.112, 0))
    rear_contact_pt = np.array((0.088, 0, 0))
    mid_contact_pt = 0.5 * (front_contact_pt + rear_contact_pt)

    left_frame = plant.GetBodyByName("toe_left").body_frame()
    right_frame = plant.GetBodyByName("toe_right").body_frame()

    frames = {"left": left_frame, "right": right_frame}
    pts = {"rear": rear_contact_pt, "mid": mid_contact_pt,
           "front": front_contact_pt}

    return frames, pts


def main():
    plant, context = make_plant_and_context()
    frames, pts = get_toe_frames_and_points(plant)
    ik_problem = CassieLipIk(plant, context)
    ik_problem.SetSwingFoot(frames['right'], pts['mid'])
    ik_problem.SetStanceFoot(frames['left'], pts['mid'])
    ik_problem.Solve(0.5, -0.15, 0.65, 0.35, num_knots=10)


if __name__ == '__main__':
    main()
