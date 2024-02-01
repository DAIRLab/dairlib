from pydrake.systems.all import (
    Value,
    State,
    Context,
    LeafSystem,
    EventStatus
)

from grid_map import GridMap, InpaintWithMinimumValues

from scipy.ndimage import (sobel, gaussian_filter, gaussian_laplace,
                           gaussian_gradient_magnitude)
from scipy.signal import convolve2d
import numpy as np
import cv2


class TerrainSegmentationSystem(LeafSystem):

    def __init__(self):
        super().__init__()

        self.input_port_grid_map = self.DeclareAbstractInputPort(
            "elevation_map", Value(GridMap())
        ).get_index()
        self.segmentation_state_idx = self.DeclareAbstractState(
            Value(
                GridMap(
                    [
                        "elevation",
                        "raw_safety_score",
                        "safety_score",
                        "segmentation",
                        "interpolated",
                        "segmented_elevation"
                    ]
                )
            )
        )
        self.DeclareStateOutputPort(
            "safe_terrain", self.segmentation_state_idx
        )
        self.DeclareForcedUnrestrictedUpdateEvent(
            self.UpdateTerrainSegmentation
        )
        self.safety_hysteresis = 0.3

        self.kernel_size = 0.17

        # gaussian blur in meters
        self.variance_blur = self.kernel_size / 2
        self.laplacian_blur = self.kernel_size / 4
        self.safe_inf_norm = 0.05  # max 5 cm difference
        self.below_edge_factor = 6.0

    def get_raw_safety_score(self, elevation: np.ndarray,
                             elevation_inpainted,
                             resolution: float):

        down_sampled = cv2.resize(
            elevation_inpainted,
            (0, 0),
            fx=2.0 * resolution / self.kernel_size,
            fy=2.0 * resolution / self.kernel_size,
            interpolation=cv2.INTER_LINEAR
        )

        up_sampled = cv2.resize(
            down_sampled,
            elevation_inpainted.shape,
            interpolation=cv2.INTER_LINEAR
        )

        stddev = gaussian_filter(
            np.abs(elevation_inpainted - up_sampled),
            self.variance_blur
        )

        dilation_kernel_size_int = int(self.kernel_size / resolution)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (dilation_kernel_size_int, dilation_kernel_size_int)
        )
        stddev = cv2.dilate(stddev, kernel)

        var_safety_score = np.minimum(
            np.ones_like(stddev),
            np.exp(25 * (self.safe_inf_norm - stddev))
        )

        curvature = gaussian_laplace(
            elevation_inpainted,
            sigma=int(self.laplacian_blur / resolution + 0.5)
        )
        below_edges = np.maximum(curvature, np.zeros_like(curvature))

        second_order_safety_score = np.exp(
            (-self.below_edge_factor / resolution) * below_edges
        )

        second_order_safety_score = np.minimum(
            second_order_safety_score,
            np.ones_like(second_order_safety_score)
        )

        # treat safety scores like independent probabilities
        raw_safety = var_safety_score * second_order_safety_score
        raw_safety[np.isnan(elevation)] = 0

        return raw_safety, up_sampled

    def UpdateTerrainSegmentation(self, context: Context, state: State):
        # Get the elevation map and undo any wrapping before image processing
        elevation_map = self.EvalAbstractInput(
            context, self.input_port_grid_map
        ).get_value()
        elevation_map.convertToDefaultStartIndex()

        InpaintWithMinimumValues(
            elevation_map, "elevation", "elevation_inpainted"
        )

        # get previous segmentation
        segmented_map = state.get_mutable_abstract_state(
            self.segmentation_state_idx
        ).get_value()

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

        raw_safety_score, upsampled = self.get_raw_safety_score(
            elevation_map['elevation'],
            elevation_map['elevation_inpainted'],
            elevation_map.getResolution()
        )
        segmented_map['interpolated'][:] = upsampled

        segmented_map["raw_safety_score"][:] = raw_safety_score
        final_safety_score = np.minimum(
            np.ones_like(raw_safety_score),
            raw_safety_score + self.safety_hysteresis * prev_segmentation
        )
        segmented_map['safety_score'][:] = final_safety_score

        safe = (segmented_map['safety_score'] > 0.8).astype(float)

        # clean up small holes in the safe regions
        kernel = np.ones((3, 3), np.uint8)
        safe = cv2.morphologyEx(safe, cv2.MORPH_CLOSE, kernel)
        safe = cv2.morphologyEx(safe, cv2.MORPH_OPEN, kernel)

        segmented_map['segmentation'][:] = safe

        safe_elevation = np.copy(elevation_map['elevation'])
        safe_elevation[~(safe > 0)] = np.nan

        segmented_map['segmented_elevation'][:] = safe_elevation
