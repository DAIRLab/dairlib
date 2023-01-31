import numpy as np

# dairlib python binding imports
from pydairlib.cassie.cassie_utils import AddCassieMultibody

from pydairlib.multibody import MakeNameToPositionsMap, \
    MakeNameToVelocitiesMap, MakeNameToActuatorsMap, \
    CreateStateNameVectorFromMap, CreateActuatorNameVectorFromMap

# drake imports

from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.solvers import Constraint
from pydrake.autodiffutils import ExtractGradient, ExtractValue, InitializeAutoDiff


def make_plant_and_context():
    urdf = "examples/Cassie/urdf/cassie_v2.urdf"
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    AddCassieMultibody(plant, scene_graph, False, urdf, True, False, True)
    plant.Finalize()
    return plant, plant.CreateDefaultContext()


class PlantHarness:
    def __init__(self):
        self.plant, self.context = make_plant_and_context()
        self.pos_map = MakeNameToPositionsMap(self.plant)
        self.plant_ad = self.plant.ToAutoDiffXd()
        self.context_ad = self.plant_ad.CreateDefaultContext()
        self.nq = self.plant.num_positions()
        self.nv = self.plant.num_velocities()

        state_names = CreateStateNameVectorFromMap(self.plant)
        self.pos_names = state_names[:self.nq]

    def make_random_pose(self, symmetric: bool = True) -> np.ndarray:
        q = np.random.uniform(low=self.plant.GetPositionLowerLimits().ravel(),
                              high=self.plant.GetPositionUpperLimits().ravel())

        # Set the right to be a mirror of the left if symmetric
        # (arbitrary choice)
        if symmetric:
            for name in self.pos_names:
                s = -1.0 if 'hip_roll' in name or 'hip_yaw' in name else 1.0
                if 'left' in name:
                    q[self.pos_map[name.replace('left', 'right')]] = \
                        s * q[self.pos_map[name]]

        # TODO: solve the fourbar to get valid positions

        return q

    def get_centroidal_momentum_matrix(self, q: np.ndarray) -> np.ndarray:
        qad = InitializeAutoDiff(q)
        vad = InitializeAutoDiff(np.ones(self.nv,))
        self.plant_ad.SetPositions(self.context_ad, qad)
        self.plant_ad.SetVelocities(self.context_ad, vad)
        com = self.plant_ad.CalcCenterOfMassPositionInWorld(self.context_ad)
        L = self.plant_ad.CalcSpatialMomentumInWorldAboutPoint(self.context_ad, com).rotational()
        A = ExtractGradient(L)
        import pdb; pdb.set_trace()
        return A[:, self.nq:]


def main():
    plant = PlantHarness()
    q = plant.make_random_pose()
    A = plant.get_centroidal_momentum_matrix(q)
    import pdb; pdb.set_trace()


if __name__ == "__main__":
    main()