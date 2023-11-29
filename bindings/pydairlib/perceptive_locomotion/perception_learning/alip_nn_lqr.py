import os
import torch
import numpy as np

from pydairlib.perceptive_locomotion.perception_learning.alip_lqr import (
    AlipFootstepLQR,
    AlipFootstepLQROptions
)

from pydrake.common.value import Value
from pydrake.systems.all import (
    Context,
    BasicVector
)

# import network related packages and set path
from pydairlib.perceptive_locomotion.perception_learning.inference.unet \
    import UNet
from pydairlib.perceptive_locomotion.perception_learning.inference.torch_utils \
    import get_device, tile_and_concatenate_inputs

perception_learning_base_folder = \
    "bindings/pydairlib/perceptive_locomotion/perception_learning"
checkpoint_path = os.path.join(
    perception_learning_base_folder, 'tmp/best_model_checkpoint.pth')


class AlipFootstepNNLQR(AlipFootstepLQR):

    def __init__(self, alip_params: AlipFootstepLQROptions):
        super().__init__(alip_params)

        # controller now has internal network module
        # maybe it is not worth putting things on GPU
        self.device = get_device()
        self.residual_unet = UNet()
        self.residual_unet.to(self.device)
        self.residual_unet.load_state_dict(torch.load(checkpoint_path))
        self.residual_unet.eval()

        self.input_port_indices['height_map'] = self.DeclareAbstractInputPort(
            "height_map",
            model_value=Value(np.ndarray(shape=(3, 30, 30)))
        ).get_index()

    def calculate_optimal_footstep(
            self, context: Context, footstep: BasicVector) -> None:
        """
            Calculate the optimal footstep location using NN prediction and
            LQR Q function through grid search
        """
        # get desired state and desired input
        xd_ud = BasicVector(6)
        self.calc_lqr_reference(context, xd_ud)
        xd = xd_ud.value()[:4]
        ud = xd_ud.value()[4:]

        # get current state
        x = BasicVector(4)
        self.calc_discrete_alip_state(context, x)
        x = x.value()

        # get heightmap
        hmap = self.EvalAbstractInput(
            context, self.input_port_indices['height_map']
        ).get_value()
        _, H, W = hmap.shape

        # test ports
        # print("hmap in controller")
        # print(hmap)

        # use utils to tile the state, input and heightmap
        # combined_input = torch.cat([hmap_input, state, input_space_grid]
        # here hmap_input = hmap[-1,:,:], input_space_grid = hmap[:2,:,:]
        # combined_input size: (1, 7, 20, 20) after unsqueeze, additional dimension
        # needed for current implementation
        combined_input = tile_and_concatenate_inputs(
            hmap[-1, :, :], x, hmap[:2, :, :]
        ).to(self.device)
        combined_input = combined_input.unsqueeze(0)

        # use network to predict residual grid
        # residual_grid size: (1, 20, 20) after squeeze
        # transform to numpy and reduce the additional dimension
        # still, maybe not put things onto GPU
        residual_grid = self.residual_unet(combined_input).squeeze(0)
        residual_grid = residual_grid.detach().cpu().numpy().reshape(H,W)

        # calculate the LQR Q function grid
        # Q value = current cost + next_value_function
        # instead of calling current function and use for loop, might be better to do in batch?
        cost_grid = np.zeros((H, W))
        next_value_grid = np.zeros((H, W))
        for i in range(H):
            for j in range(W):
                cost_grid[i,j] = (xd - x).T @ self.params.Q @ (xd - x) + \
                                 (hmap[:2,i,j] - ud).T @ self.params.R @ (hmap[:2,i,j] - ud)
                next_value_grid[i,j] = self.get_next_value_estimate(x,hmap[:2,i,j], xd, ud)

        # sum up the grid values, add select the minimum value index
        final_grid = cost_grid + next_value_grid + residual_grid
        footstep_i, footstep_j = np.unravel_index(np.argmin(final_grid), final_grid.shape)

        # footstep command from corresponding grid
        footstep_command = hmap[:, footstep_i, footstep_j]
        footstep.set_value(footstep_command)
