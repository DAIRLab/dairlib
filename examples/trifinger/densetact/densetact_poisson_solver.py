import numpy as np
import scipy as sp
from sklearn.feature_extraction.image import grid_to_graph

class poisson:
    def __init__(self, XX, YY, mask, rad):
        # XX, YY is the grid coordinate of the image 
        x_n = XX.shape[0]
        y_n = YY.shape[0]

        #laplacian = sp.sparse.linalg.LaplacianNd((x_n, y_n), boundary_conditions = 'dirichlet').tosparse()

        graph = grid_to_graph(x_n, y_n, mask = mask)
        laplacian = sp.sparse.csgraph.laplacian(graph).toarray()
        laplacian[laplacian>1] = 4
        laplacian = sp.sparse.csc_array(laplacian)

        self.x = XX[0, :]
        self.y = YY[:, 0]

        # Get grid spacing
        dx = np.diff(self.x)[0]
        dy = np.diff(self.y)[0]

        # Defining the proper laplacian stencil requires uniform grid spacing
        assert np.isclose(dx, dy), "grid spacing must be uniform"
        self.h = dx

        self.mask = mask
        self.laplacian = laplacian
        self.XX = XX
        self.YY = YY
        self.x_n = x_n
        self.y_n = y_n

        self.rad = rad

    def solve(self, flow2):

        # mask = ~np.isnan(flow[..., 0]) #inside is true
        # flow[~mask] = 0

        flow = np.nan_to_num(flow2, nan = 0)

        XX = self.XX
        YY = self.YY
        x = self.x
        y = self.y
        h = self.h
        mask = self.mask
        rad = self.rad
        
        # Get gradients of vector field
        dudy, dudx = np.gradient(flow[..., 0], y, x, edge_order = 1)
        dvdy, dvdx = np.gradient(flow[..., 1], y, x, edge_order = 1)

        # Get divergence and curl of the vector field
        div_F = dudx + dvdy
        curl_F = dudy - dvdx

        # cx = (x[-1] - x[0])/2
        # cy = (y[-1] - y[0])/2

        # Set up PDE
        #ZZ = np.sqrt(((rad)**2 - (XX-cx)**2 - (YY-cy)**2))
        #ZZ = np.nan_to_num(ZZ, 0)
       
        #dzdy, dzdx = np.gradient(ZZ, y, x, edge_order = 1)
        #c = np.gradient(dzdx, x, axis = 1) + np.gradient(dzdy, y, axis = 0)

        #print(ZZ)
        # xc = XX - cx
        # yc = YY - cy

        #c = -((XX-cx)**2*ZZ**-3 + ZZ**-1) + ((YY-cy)**2*ZZ**-3 + ZZ**-1)
        #print(np.round(np.gradient(dzdy, y, axis = 0),2))

        #sph_div = np.nan_to_num(c, 0)
        #print(np.round((div_F * h * h),2))

        b1 = (div_F * h * h)[mask] #+ sph_div[mask]
        b2 = (curl_F * h * h)[mask] 

        phi_flat = sp.sparse.linalg.spsolve(self.laplacian, b1)
        A_flat = sp.sparse.linalg.spsolve(self.laplacian, b2)

        phi = np.zeros_like(flow[..., 0])
        A = np.zeros_like(flow[..., 0])

        phi[mask] = phi_flat
        A[mask] = A_flat

        phi_y, phi_x = np.gradient(phi, y, x, edge_order=2)
        A_y, A_x = np.gradient(A, y, x, edge_order=2)

        phi_x[~mask] = np.nan
        phi_y[~mask] = np.nan
        A_x[~mask] = np.nan
        A_y[~mask] = np.nan

        phi[~mask] = np.nan
        A[~mask] = np.nan

        curl_free = -np.stack([phi_x, phi_y], axis = -1)
        div_free = -np.stack([A_y, -A_x], axis = -1)

        harm = flow - curl_free - div_free

        return curl_free, div_free, harm, phi, A
