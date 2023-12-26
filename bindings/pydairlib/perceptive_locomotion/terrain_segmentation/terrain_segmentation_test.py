import numpy as np

from pydairlib.perceptive_locomotion.terrain_segmentation.\
    terrain_segmentation_system import TerrainSegmentationSystem


def main():
    data = np.load('../elevation_maps.npz', allow_pickle=True)
    terrain = data['elevation_maps']
    terrain_segmentation = TerrainSegmentationSystem()
    context = terrain_segmentation.CreateDefaultContext()

    for map in terrain:
        terrain_segmentation.get_input_port().FixValue(context, map)
        terrain_segmentation.CalcForcedUnrestrictedUpdate(context, context.get_mutable_state())


if __name__ == '__main__':
    main()
