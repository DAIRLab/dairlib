from dataclasses import dataclass
from pydrake.systems.framework import (
    System,
    Context,
    LeafSystem,
    EventStatus
)

from pydairlib.systems.perception import (
    terrain_segmentation_reset_params,
    TerrainSegmentationMonitor,
)


class MapResetMonitor:
    def __init__(self, monitor, mapper):
        self._monitor = monitor
        self._mapper = mapper

    def monitor(self, root_context):
        monitor_context = self._monitor.GetMyContextFromRoot(root_context)

        reset = False
        if self._monitor.NeedsMinValidAreaReset(monitor_context):
            print("Resetting based on low min area")
            reset = True
        elif self._monitor.NeedsIoUReset(monitor_context):
            print("resetting based on IoU")
            reset = True

        if reset:
            gridmap = self._monitor.GetMapForReInitialization(monitor_context)
            self._mapper.ReInitializeElevationMap(
                root_context, gridmap, "interpolated")
            self._monitor.Reset(monitor_context)
            return EventStatus.Succeeded()

        return EventStatus.DidNothing()
