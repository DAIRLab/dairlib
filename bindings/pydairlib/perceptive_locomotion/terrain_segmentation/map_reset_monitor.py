from dataclasses import dataclass
from pydrake.systems.framework import (
    System,
    Context,
    EventStatus
)


@dataclass
class map_reset_params:
    iou_thresh: float = 0.6
    min_area_thresh: float = 0.5


class MapResetMonitor:
    def __init__(self, params: map_reset_params, segmenter: System, mapper: System):
        self._params = params
        self._segmenter = segmenter
        self._mapper = mapper

    def monitor(self, root_context):
        segmenter_context = self._segmenter.GetMyContextFromRoot(root_context)

