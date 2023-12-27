from pydrake.systems.all import (
    Value,
    State,
    Context,
    LeafSystem,
    EventStatus
)

from grid_map import GridMap
from scipy.ndimage import sobel, gaussian_filter
from scipy.signal import convolve2d
import numpy as np
import cv2

from pydairlib.perceptive_locomotion.terrain_segmentation.vision_utils import edges

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

        self.laplacian_kernel = np.array(
            [[1.0, 1.0, 1.0], [1.0, -8.0, 1.0], [1.0, 1.0, 1.0]])

        self.alpha = 0.55

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

        segmented.move(elevation_map.getPosition())
        segmented.convertToDefaultStartIndex()

        prev_segmentation = segmented['segmentation']
        prev_segmentation[np.isnan(prev_segmentation)] = 0.0

        median = cv2.medianBlur(hmap, 5)
        blurred_narrow = gaussian_filter(median / elevation_map.getResolution(), 1.0, truncate=3)
        blurred_wide = gaussian_filter(median / elevation_map.getResolution(), 2.0, truncate=3)
        curvature = convolve2d(blurred_wide, self.laplacian_kernel, mode='same')
        below_edges = np.maximum(curvature, np.zeros_like(curvature))
        below_edges_safe_prob = np.exp(-3.0 * below_edges)

        grad = edges(blurred_narrow)
        grad_safe_prob = np.exp(-0.5 * grad)
        segmented['safe_probabilities'][:] = below_edges_safe_prob * grad_safe_prob + self.alpha * prev_segmentation
        segmented['segmentation'][:] = (segmented['safe_probabilities'] > 0.6).astype(float)



