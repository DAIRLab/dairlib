import os
import torch
import numpy as np
import time

from pydairlib.perceptive_locomotion.systems.alip_lqr import (
    AlipFootstepLQR,
    AlipFootstepLQROptions,
    calc_collision_cost_grid
)

from pydairlib.perceptive_locomotion.systems.height_map_server \
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

from matplotlib.cm import ScalarMappable
from matplotlib.colors import LogNorm

perception_learning_base_folder = \
    "bindings/pydairlib/perceptive_locomotion/perception_learning"


class AlipFootstepNNLQR(AlipFootstepLQR):

    def __init__(self, alip_params: AlipFootstepLQROptions,
                 model_path: str = ''):
        super().__init__(alip_params)

        if model_path == '':
            raise RuntimeError('AlipNNLQRFootstepController model path cannot be empty\n')

        # controller now has internal network module
        # maybe it is not worth putting things on GPU
        self.device = get_device()
        self.residual_unet = UNet(7, 1)
        self.residual_unet.to(self.device)
        self.residual_unet.load_state_dict(
            torch.load(model_path)
        )
        self.residual_unet.eval()

        self.input_port_indices['height_map'] = self.DeclareAbstractInputPort(
            "height_map_query",
            model_value=Value(HeightMapQueryObject())
        ).get_index()

        # input error coordinate ue grid precomputation
        self.map_opts = HeightMapOptions()
        H = self.map_opts.nx
        W = self.map_opts.ny
        uxgrid = np.linspace(
            -self.map_opts.resolution * H / 2,
            self.map_opts.resolution * H / 2,
            H
        )
        uygrid = np.linspace(
            -self.map_opts.resolution * W / 2,
            self.map_opts.resolution * W / 2,
            W
        )
        UX, UY = np.meshgrid(uxgrid, uygrid)
        self.ue_grid = np.stack([UX, UY])

        # use double for loop to calculate grid quadratic form and linear coefficient offline
        # u_cost = ue.T @ R @ ue, u_next_value = ue.T @ B.T @ S @ B @ ue, linear_coeff = 2 * A.T @ S @ B @ ue
        self.u_cost_grid = np.zeros((H, W))
        self.u_next_value_grid = np.zeros((H, W))
        self.linear_coeff_grid = np.zeros((self.A.shape[0], H, W))
        for i in range(H):
            for j in range(W):
                self.u_cost_grid[i, j] = \
                    self.ue_grid[:, i, j].T @ self.params.R @ \
                    self.ue_grid[:, i, j]
                self.u_next_value_grid[i, j] = \
                    self.ue_grid[:, i, j].T @ self.B.T @ self.S @ self.B @\
                    self.ue_grid[:, i, j]
                self.linear_coeff_grid[:, i, j] = \
                    2 * self.A.T @ self.S @ self.B @ self.ue_grid[:, i, j]

    def calculate_optimal_footstep(
            self, context: Context, footstep: BasicVector) -> None:
        """
            Calculate the optimal footstep location using NN prediction and
            LQR Q function through grid search
        """

        # get control parameters
        xd_ud = self.get_output_port_by_name('lqr_reference').Eval(context)
        xd = xd_ud[:4]
        ud = xd_ud[4:]
        x = self.get_output_port_by_name('x').Eval(context)

        # get heightmap query object
        hmap_query = self.EvalAbstractInput(
            context, self.input_port_indices['height_map']
        ).get_value()

        # query for cropped height map at nominal footstep location
        hmap = hmap_query.calc_height_map_stance_frame(
            np.array([ud[0], ud[1], 0])
        )
        _, H, W = hmap.shape
        collision_cost = calc_collision_cost_grid(hmap[0], hmap[1], ud)

        with torch.inference_mode():
            combined_input = tile_and_concatenate_inputs(
                hmap[-1], x - xd, self.ue_grid
            ).to(self.device)
            combined_input = combined_input.unsqueeze(0)

            residual_grid = self.residual_unet(combined_input).squeeze(0)
            residual_grid = residual_grid.detach().cpu().numpy().reshape(H, W)
        # hmap_edges = edges_blurred(hmap[2], 3.0)
        # residual_grid = 100 * hmap_edges / np.max(hmap_edges)

        # display residual map on meshcat
        residual_grid_world = hmap_query.calc_height_map_world_frame(
            np.array([ud[0], ud[1], 0])
        )
        # residual_grid_world[2] = residual_grid

        # tensor algebra (batch operation) for calculating linear term grid
        linear_term_grid = np.einsum('n,nhw->hw', (x - xd), self.linear_coeff_grid)

        # sum up the grid values, add select the minimum value index
        final_grid_tmp = self.u_cost_grid + self.u_next_value_grid + linear_term_grid + residual_grid
        final_grid = self.u_cost_grid + self.u_next_value_grid + linear_term_grid + residual_grid + collision_cost
        # final_grid = self.u_cost_grid + self.u_next_value_grid + linear_term_grid + collision_cost

        color_mappable = ScalarMappable(cmap='jet')
        colors = color_mappable.to_rgba(residual_grid)

        hmap_query.plot_colored_surface(
            "residual", residual_grid_world[0], residual_grid_world[1],
            residual_grid_world[2], colors[:, :, 0], colors[:, :, 1], colors[:, :, 2]
        )

        footstep_i, footstep_j = np.unravel_index(
            np.argmin(final_grid), final_grid.shape
        )

        # footstep command from corresponding grid
        footstep_command = hmap[:, footstep_i, footstep_j]
        footstep.set_value(footstep_command)
