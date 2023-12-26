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
    fig, ax = plt.subplots()

    # Display the initial frame using imshow
    img = ax.imshow(imdata, interpolation='none', aspect='auto', vmin=-1, vmax=1)

    def anim_callback(i):
        terrain_segmentation.get_input_port().FixValue(context, terrain[i])
        terrain_segmentation.CalcForcedUnrestrictedUpdate(context, context.get_mutable_state())
        segment = terrain_segmentation.get_output_port().Eval(context)
        segment = segment['safe_probabilities']
        img.set_array(segment)
        return [img]

    animation = FuncAnimation(
        fig, anim_callback, frames=range(len(terrain)), blit=True, interval=100
    )

    # animation.save('../test_anim.gif', fps=30)
    # Show the animation
    plt.show()


if __name__ == '__main__':
    main()
