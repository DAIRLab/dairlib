import time

import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from pydairlib.perceptive_locomotion.terrain_segmentation.\
    terrain_segmentation_system import TerrainSegmentationSystem

from pydairlib.perceptive_locomotion.terrain_segmentation.\
    convex_terrain_decomposition_system import \
    ConvexTerrainDecompositionSystem

from pydrake.systems.all import DiagramBuilder


def main():
    data = np.load('../elevation_maps.npz', allow_pickle=True)
    terrain = data['arr_0']

    builder = DiagramBuilder()

    terrain_segmentation = builder.AddSystem(TerrainSegmentationSystem())
    convex_decomposition = builder.AddSystem(ConvexTerrainDecompositionSystem())

    builder.Connect(
        terrain_segmentation.get_output_port(),
        convex_decomposition.get_input_port()
    )

    builder.ExportInput(terrain_segmentation.get_input_port())
    builder.ExportOutput(convex_decomposition.get_output_port())

    diagram = builder.Build()

    context = diagram.CreateDefaultContext()

    imdata = np.zeros((terrain[0].getSize()))

    # Create a figure and axis
    fig, (ax1, ax2) = plt.subplots(1, 2)

    # Display the initial frame using imshow
    img1 = ax1.imshow(imdata, interpolation='none', aspect=1, vmin=0, vmax=1)
    img2 = ax2.imshow(imdata, interpolation='none', aspect=1, vmin=-0.0, vmax=0.6)

    def anim_callback(i):
        diagram.get_input_port().FixValue(context, terrain[i])
        tic = time.perf_counter()
        diagram.CalcForcedUnrestrictedUpdate(context, context.get_mutable_state())
        polygons = diagram.get_output_port().Eval(context)
        toc = time.perf_counter()
        print(f'processing took: {toc-tic} sec')
        segment = terrain_segmentation.get_output_port().Eval(
            terrain_segmentation.GetMyContextFromRoot(context)
        )
        segment = segment['segmentation']
        img1.set_array(segment)

        terrain[i].convertToDefaultStartIndex()
        img2.set_array(terrain[i]['elevation'])
        return [img1, img2]

    animation = FuncAnimation(
        fig, anim_callback, frames=range(len(terrain)), blit=True, interval=20
    )

    # animation.save('../test_anim.gif', fps=30)
    # Show the animation
    plt.show()


if __name__ == '__main__':
    main()
