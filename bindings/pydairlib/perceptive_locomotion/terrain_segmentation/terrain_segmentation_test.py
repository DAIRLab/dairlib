import time

import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from pydairlib.perceptive_locomotion.terrain_segmentation.\
    terrain_segmentation_system import TerrainSegmentationSystem


def main():
    data = np.load('../elevation_maps.npz', allow_pickle=True)
    terrain = data['arr_0']
    terrain_segmentation = TerrainSegmentationSystem()
    context = terrain_segmentation.CreateDefaultContext()

    imdata = np.zeros((terrain[0].getSize()))

    # Create a figure and axis
    fig, (ax1, ax2) = plt.subplots(1,2)

    # Display the initial frame using imshow
    img1 = ax1.imshow(imdata, interpolation='none', aspect=1, vmin=0, vmax=1)
    img2 = ax2.imshow(imdata, interpolation='none', aspect=1, vmin=-0.1, vmax=0.4)

    def anim_callback(i):
        terrain_segmentation.get_input_port().FixValue(context, terrain[i])
        tic = time.perf_counter()
        terrain_segmentation.CalcForcedUnrestrictedUpdate(context, context.get_mutable_state())
        toc = time.perf_counter()
        print(f'processing took: {toc-tic} sec')
        segment = terrain_segmentation.get_output_port().Eval(context)
        segment = segment['segmentation']
        img1.set_array(segment)

        terrain[i].convertToDefaultStartIndex()
        img2.set_array(terrain[i]['elevation'])
        return [img1, img2]

    animation = FuncAnimation(
        fig, anim_callback, frames=range(len(terrain)), blit=True, interval=200
    )

    # animation.save('../test_anim.gif', fps=30)
    # Show the animation
    plt.show()


if __name__ == '__main__':
    main()
