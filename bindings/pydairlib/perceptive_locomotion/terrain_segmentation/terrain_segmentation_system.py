from pydrake.systems.all import (
    Value,
    State,
    Context,
    LeafSystem,
    EventStatus
)

from grid_map import GridMap, InpaintWithMinimumValues
import numpy as np
import cv2
import time

from typing import Tuple


def initialize_from_input_map(init_map: GridMap, input_map: GridMap) -> None:
    init_map.setGeometry(
        input_map.getLength(),
        input_map.getResolution(),
        input_map.getPosition()
    )
    # Initialize to all safe terrain
    init_map["segmentation"][:] = np.ones(init_map.getSize())


def clopen(img: np.ndarray):
    kernel = np.ones((3, 3), np.uint8)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    return img


class TerrainSegmentationSystem(LeafSystem):

    def __init__(self, safety_callbacks, profiling=None):
        super().__init__()
        self.set_name("S3")

        self.input_port_grid_map = self.DeclareAbstractInputPort(
            "elevation_map", Value(GridMap())
        ).get_index()
        self.segmentation_state_idx = self.DeclareAbstractState(
            Value(
                GridMap(
                    [
                        "elevation",
                        "elevation_inpainted",
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
        self.safety_hysteresis = 0.4
        self.kernel_length = 0.17
        self.erosion_kernel_length = self.kernel_length / 1.2
        self.safety_threshold = 0.7

        self.safety_criterion_callbacks = safety_callbacks
        self.profiling = profiling
        self.debug = False
        self.safety_scores = {}
        self.inpaint_unseen_terrain = True
        self.opencv_inpaint = True

    def get_raw_safety_score(
            self, elevation: np.ndarray, denoised_and_inpainted_map: np.ndarray,
            resolution: float) -> np.ndarray:

        raw_safety = np.ones_like(denoised_and_inpainted_map)
        kernel = self.get_kernel_size(resolution)

        i = 0
        for name, callback in self.safety_criterion_callbacks.items():
            start = time.time()
            raw_safety = raw_safety * callback(
                denoised_and_inpainted_map, kernel, resolution
            )
            end = time.time()
            if self.profiling:
                self.profiling['seg_callbacks'][i].append(end - start)
            i += 1
            if self.debug:
                self.safety_scores[name] = callback(
                    denoised_and_inpainted_map, kernel, resolution)

        raw_safety = np.power(raw_safety, 1./len(self.safety_criterion_callbacks))
        
        if not self.inpaint_unseen_terrain:
            raw_safety[np.isnan(elevation)] = 0

        if self.debug:
            self.safety_scores['combined'] = raw_safety

        return raw_safety

    def get_kernel_size(self, resolution: float, length=None) -> Tuple[int, int]:
        length = self.kernel_length if length is None else length
        ksize_int = int(length / resolution + 0.5)
        return ksize_int, ksize_int

    def cleanup_and_add_hysteresis(
            self, raw_safety_score: np.ndarray,
            prev_segmentation: np.ndarray, resolution: float) -> np.ndarray:
        final_safety_score = np.minimum(
            np.ones_like(raw_safety_score),
            raw_safety_score + self.safety_hysteresis * prev_segmentation
        )
        erosion_ksize = self.get_kernel_size(
            resolution, length=self.erosion_kernel_length
        )
        erosion_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, erosion_ksize
        )
        return cv2.erode(final_safety_score, erosion_kernel)

    def inpaint(self, elevation_map: GridMap):
        raw_map = elevation_map["elevation"]
        mask = np.zeros_like(raw_map, dtype=np.uint8)
        mask[np.isnan(raw_map)] = 255
        if not elevation_map.exists("elevation_inpainted"):
            elevation_map.add("elevation_inpainted")
            
        if self.opencv_inpaint:
            elevation_map["elevation_inpainted"][:] = cv2.inpaint(
                raw_map, mask, 1, flags=cv2.INPAINT_NS)
        else:
            elevation_map["elevation_inpainted"][:] = raw_map

        InpaintWithMinimumValues(
            elevation_map, "elevation_inpainted", "elevation_inpainted"
        )

    def MakeDrivenByStandaloneSimulator(self, dt: float):
        assert(dt > 0)
        self.DeclarePeriodicUnrestrictedUpdateEvent(
            dt, 0.0, self.UpdateTerrainSegmentation
        )

    def UpdateTerrainSegmentation(self, context: Context, state: State):
        # Get the elevation map and undo any wrapping before image processing

        start = time.time()

        elevation_map = self.EvalAbstractInput(
            context, self.input_port_grid_map
        ).get_value()
        elevation_map.convertToDefaultStartIndex()

        self.inpaint(elevation_map)

        # get previous segmentation
        segmented_map = state.get_mutable_abstract_state(
            self.segmentation_state_idx
        ).get_value()

        # make the previous segmentation match the geometry of the elevation map
        if segmented_map.getSize()[0] == 0:
            initialize_from_input_map(segmented_map, elevation_map)

        segmented_map.move(elevation_map.getPosition())
        segmented_map.convertToDefaultStartIndex()

        segmented_map['elevation'][:] = elevation_map['elevation']
        segmented_map['elevation_inpainted'][:] = \
            elevation_map['elevation_inpainted']

        prev_segmentation = segmented_map['segmentation']
        prev_segmentation[np.isnan(prev_segmentation)] = 0.0

        denoised_and_inpainted_map = cv2.medianBlur(
            elevation_map['elevation_inpainted'], 5)

        raw_safety_score = self.get_raw_safety_score(
            elevation_map['elevation'],
            denoised_and_inpainted_map,
            elevation_map.getResolution()
        )

        smoothed = cv2.boxFilter(
            denoised_and_inpainted_map,
            -1,
            self.get_kernel_size(elevation_map.getResolution()),
            normalize=True
        )

        segmented_map['interpolated'][:] = smoothed
        final_safety_score = self.cleanup_and_add_hysteresis(
            raw_safety_score, prev_segmentation, elevation_map.getResolution()
        )

        safe = (final_safety_score > self.safety_threshold).astype(float)

        # clean up small holes in the safe regions
        safe = clopen(safe)

        segmented_map['segmentation'][:] = safe

        safe_elevation = np.copy(elevation_map['elevation_inpainted'])
        safe_elevation[~(safe > 0)] = np.nan
        segmented_map['segmented_elevation'][:] = safe_elevation

        end = time.time()
        if self.profiling:
            self.profiling['segmentation'].append(end - start)
