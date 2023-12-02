import os
import torch
import numpy as np
import time

from pydairlib.perceptive_locomotion.perception_learning.alip_lqr import (
    AlipFootstepLQR,
    AlipFootstepLQROptions
)

from pydairlib.perceptive_locomotion.perception_learning.height_map_server \
import (
    HeightMapQueryObject,
    HeightMapOptions
)

from pydrake.geometry import Rgba
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
    perception_learning_base_folder, 'tmp/morning-pyramid-103.pth')


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
            "height_map_query",
            model_value=Value(HeightMapQueryObject())
        ).get_index()

        # input error cooridinate ue grid precomputation
        self.map_opts = HeightMapOptions()
        H = self.map_opts.nx
        W = self.map_opts.ny
        uxgrid = np.linspace(-self.map_opts.resolution * H / 2,
            self.map_opts.resolution * H / 2, H)
        uygrid = np.linspace(-self.map_opts.resolution * W / 2,
            self.map_opts.resolution * W / 2, W)
        UX, UY = np.meshgrid(uxgrid, uygrid)
        self.ue_grid = np.stack([UX, UY])

        # use double for loop to calculate grid quadratic form and linear coefficient offline
        # u_cost = ue.T @ R @ ue, u_next_value = ue.T @ B.T @ S @ B @ ue, linear_coeff = 2 * A.T @ S @ B @ ue
        self.u_cost_grid = np.zeros((H, W))
        self.u_next_value_grid = np.zeros((H, W))
        self.linear_coeff_grid = np.zeros((self.A.shape[0], H, W))
        for i in range(H):
            for j in range(W):
                self.u_cost_grid[i, j] = (self.ue_grid[:,i,j]).T @ self.params.R @ (self.ue_grid[:,i,j])
                self.u_next_value_grid[i, j] = (self.ue_grid[:,i,j]).T @ self.B.T @ self.S @ self.B @ (self.ue_grid[:,i,j])
                self.linear_coeff_grid[:, i, j] = 2 * self.A.T @ self.S @ self.B @ (self.ue_grid[:,i,j])

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

        # get heightmap query object
        hmap_query = self.EvalAbstractInput(
            context, self.input_port_indices['height_map']
        ).get_value()

        # query for cropped height map at nominal footstep location
        hmap = hmap_query.calc_height_map_stance_frame(
            np.array([ud[0], ud[1],0])
        )
        _, H, W = hmap.shape

        # put the input space in error coordinates, equivalent to hmap[:2] = self.ue_grid
        hmap[:2] -= np.expand_dims(np.expand_dims(ud, 1), 1)

        # use utils to tile the state, input and heightmap
        # combined_input = torch.cat([hmap_input, state, input_space_grid]
        # here hmap_input = hmap[-1, :, :], input_space_grid = hmap[:2, :, :]
        # combined_input size: (1, 7, 20, 20) after unsqueeze, additional dimension
        # needed for current implementation

        with torch.inference_mode():
            combined_input = tile_and_concatenate_inputs(
                hmap[-1, :, :], x - xd, hmap[:2, :, :]
            ).to(self.device)
            combined_input = combined_input.unsqueeze(0)

            # use network to predict residual grid
            # residual_grid size: (1, 20, 20) after squeeze
            # transform to numpy and reduce the additional dimension
            # still, maybe not put things onto GPU

            residual_grid = self.residual_unet(combined_input).squeeze(0)
            residual_grid = residual_grid.detach().cpu().numpy().reshape(H,W)

        # calculate the LQR Q function grid
        # Q value = current cost + next_value_function, after expanding, and neglect the constant term
        # Q value = u_cost_grid  + u_next_value_grid (precomputue, quadratic) + linear_term_grid + residual_grid

        # # # display residual map on meshcat
        residual_map = hmap_query.calc_world_frame_residual_map(
            np.array([ud[0], ud[1],0]), residual_grid
        )
        
        # tensor algebra (batch operation) for calculating linear term grid
        linear_term_grid = np.einsum('n,nhw->hw', (x - xd), self.linear_coeff_grid)

        # sum up the grid values, add select the minimum value index
        final_grid = self.u_cost_grid + self.u_next_value_grid + linear_term_grid + residual_grid
        # final_grid = self.u_cost_grid + self.u_next_value_grid + linear_term_grid
        footstep_i, footstep_j = np.unravel_index(
            np.argmin(final_grid), final_grid.shape
        )

        # footstep command from corresponding grid
        footstep_command = hmap[:, footstep_i, footstep_j]
        # note that now hmap[:2, i, j] is error corrdinate ue since we subtract ud before
        footstep_command[:2] += ud
        footstep.set_value(footstep_command)
