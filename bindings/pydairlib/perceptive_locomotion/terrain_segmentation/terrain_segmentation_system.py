from pydrake.systems.all import (
    Value,
    State,
    Context,
    LeafSystem,
    EventStatus
)

from grid_map import GridMap, InpaintWithMinimumValues

from scipy.ndimage import sobel, gaussian_filter
from scipy.signal import convolve2d
import numpy as np
import cv2

from pydairlib.perceptive_locomotion.terrain_segmentation.\
    vision_utils import edges


class TerrainSegmentationSystem(LeafSystem):

    def __init__(self):
        super().__init__()

        self.input_port_grid_map = self.DeclareAbstractInputPort(
            "elevation_map", Value(GridMap())
        ).get_index()
        self.segmentation_state_idx = self.DeclareAbstractState(
            Value(GridMap(["elevation", "safety_score", "segmentation"]))
        )
        self.DeclareStateOutputPort(
            "safe_terrain", self.segmentation_state_idx
        )
        self.DeclareForcedUnrestrictedUpdateEvent(
            self.UpdateTerrainSegmentation
        )
        self.laplacian_kernel = np.array(
            [[1.0, 1.0, 1.0],
             [1.0, -8.0, 1.0],
             [1.0, 1.0, 1.0]]
        )
        self.safety_hysteresis = 0.6

    def get_raw_safety_score(self, elevation: np.ndarray, resolution: float):
        # only use a small amount of blur for first order safety criterion
        blurred_narrow = gaussian_filter(
            elevation / resolution, 1.0, truncate=3
        )
        image_gradient_magnitude = edges(blurred_narrow)
        first_order_safety_score = np.exp(-0.25 * image_gradient_magnitude)

        # use a larger blur for second order safety criterion
        blurred_wide = gaussian_filter(
            elevation / resolution, 2.0, truncate=3
        )
        curvature = convolve2d(blurred_wide, self.laplacian_kernel, mode='same')
        below_edges = np.maximum(curvature, np.zeros_like(curvature))
        second_order_safety_score = np.exp(-3.0 * below_edges)

        # treat safety scores like independent probabilities
        return second_order_safety_score * first_order_safety_score

    def UpdateTerrainSegmentation(self, context: Context, state: State):
        # Get the elevation map and undo any wrapping before image processing
        elevation_map = self.get_input_port(
            self.input_port_grid_map
        ).Eval(context)
        elevation_map.convertToDefaultStartIndex()

        # get previous segmentation
        segmented_map = state.get_mutable_abstract_state(
            self.segmentation_state_idx
        ).get_value()
        segmented_map.convertToDefaultStartIndex()

        # make the previous segmentation match the geometry of the elevation map
        if segmented_map.getSize()[0] == 0:
            segmented_map.setGeometry(
                elevation_map.getLength(),
                elevation_map.getResolution(),
                elevation_map.getPosition()
            )

        segmented_map.move(elevation_map.getPosition())
        segmented_map.convertToDefaultStartIndex()

        segmented_map['elevation'][:] = elevation_map['elevation']

        prev_segmentation = segmented_map['segmentation']
        prev_segmentation[np.isnan(prev_segmentation)] = 0.0

        raw_safety_score = self.get_raw_safety_score(
            elevation_map['elevation'], elevation_map.getResolution()
        )

        segmented_map['safety_score'][:] =\
            raw_safety_score + self.safety_hysteresis * prev_segmentation

        safe = (segmented_map['safety_score'] > 0.7).astype(float)

        # clean up small holes in the safe regions
        kernel = np.ones((3, 3), np.uint8)
        safe = cv2.morphologyEx(safe, cv2.MORPH_CLOSE, kernel)
        safe = cv2.morphologyEx(safe, cv2.MORPH_OPEN, kernel)

        segmented_map['segmentation'][:] = safe
