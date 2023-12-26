from pydrake.systems.all import (
    Value,
    State,
    Context,
    LeafSystem,
    EventStatus
)

from grid_map import GridMap
from scipy.ndimage import sobel


class TerrainSegmentationSystem(LeafSystem):

    def __init__(self):
        super().__init__()

        self.input_port_grid_map = self.DeclareAbstractInputPort(
            "elevation_map", Value(GridMap())
        ).get_index()

        self.segmentation_state_idx = self.DeclareAbstractState(
            Value(GridMap(["safe_probabilities", "segmentation"]))
        )
        self.DeclareStateOutputPort("segmentation", self.segmentation_state_idx)

        self.DeclareForcedUnrestrictedUpdateEvent(self.UpdateTerrainSegmentation)

    def UpdateTerrainSegmentation(self, context: Context, state: State):
        elevation_map = self.get_input_port(self.input_port_grid_map).Eval(context)
        elevation_map.convertToDefaultStartIndex()
        hmap = elevation_map['elevation']

        segmented = state.get_mutable_abstract_state(self.segmentation_state_idx).get_value()

        if segmented.getSize()[0] == 0:
            segmented.setGeometry(
                elevation_map.getLength(),
                elevation_map.getResolution(),
                elevation_map.getPosition()
            )

        segmented['safe_probabilities'][:] = sobel(hmap, axis=0) / elevation_map.getResolution()

