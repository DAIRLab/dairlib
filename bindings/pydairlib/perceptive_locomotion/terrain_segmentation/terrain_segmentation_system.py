from pydrake.systems.all import (
    Value,
    State,
    Context,
    LeafSystem,
    EventStatus
)

from grid_map import GridMap


class TerrainSegmentationSystem(LeafSystem):

    def __init__(self):
        super().__init__()

        self.input_port_grid_map = self.DeclareAbstractInputPort(
            "elevation_map", Value(GridMap())
        ).get_index()

        self.segmentation_state_idx = self.DeclareAbstractState(
            Value(GridMap())
        )

        self.DeclareForcedUnrestrictedUpdateEvent(self.UpdateTerrainSegmentation)

    def UpdateTerrainSegmentation(self, context: Context, state: State):
        elevation_map = self.get_input_port(self.input_port_grid_map).Eval(context)
        hmap = elevation_map['elevation']
        import cv2
        import numpy as np
        from scipy.ndimage import sobel, gaussian_filter
        import pdb; pdb.set_trace()

