import numpy as np
import scipy as sp

class poisson:
    def __init__(self, XX, YY):
        # XX, YY is the grid coordinate of the image 
        x_n = XX.shape[0]
        y_n = YY.shape[0]

        laplacian = sp.sparse.linalg.LaplacianNd((x_n, y_n), boundary_conditions = 'dirichlet').tosparse()

        self.x = XX[0, :]
        self.y = YY[:, 0]

        # Get grid spacing
        dx = np.diff(self.x)[0]
        dy = np.diff(self.y)[0]

        # Defining the proper laplacian stencil requires uniform grid spacing
        assert np.isclose(dx, dy), "grid spacing must be uniform"
        self.h = dx

        self.laplacian = laplacian
        self.XX = XX
        self.YY = YY
        self.x_n = x_n
        self.y_n = y_n

    def remove_discon(self, flow, mask):

        flow_zero = flow.copy()
        flow_zero[~mask] = 0
        blurred_flow = sp.ndimage.gaussian_filter(flow_zero, sigma = 0.833)

        flow[~mask] = blurred_flow[~mask]

    def solve(self, flow2):
        flow = flow2.copy()

        mask = ~np.isnan(flow[..., 0]) #inside is true
        flow[~mask] = 0
        #self.remove_discon(flow, mask)
        x = self.x
        y = self.y

        x_n = self.x_n
        y_n = self.y_n

        h = self.h
        
        # Get gradients of vector field
        dudy, dudx = np.gradient(flow[..., 0], y, x, edge_order = 1)
        dvdy, dvdx = np.gradient(flow[..., 1], y, x, edge_order = 1)

        # Get divergence and curl of the vector field
        div_F = dudx + dvdy
        curl_F = dudy - dvdx

        # Set up PDE
        b1 = div_F * h * h
        b2 = curl_F * h * h

        # b1[~mask] = 0
        # b2[~mask] = 0

        phi = sp.sparse.linalg.spsolve(self.laplacian, b1.flatten()).reshape(x_n, y_n)
        A = sp.sparse.linalg.spsolve(self.laplacian, b2.flatten()).reshape(x_n, y_n)

        phi_y, phi_x = np.gradient(phi, y, x, edge_order=2)
        A_y, A_x = np.gradient(A, y, x, edge_order=2)

        phi_x[~mask] = np.nan
        phi_y[~mask] = np.nan
        A_x[~mask] = np.nan
        A_y[~mask] = np.nan

        curl_free = np.stack([phi_x, phi_y], axis = -1)
        div_free = np.stack([A_y, -A_x], axis = -1)

        harm = flow - curl_free - div_free

        return curl_free, div_free, harm, phi
