import numpy as np
from scipy.ndimage import sobel, gaussian_filter


def edges(grid: np.ndarray) -> np.ndarray:
    sobelx = sobel(grid, axis=0)
    sobely = sobel(grid, axis=1)
    edge_mat = sobelx ** 2 + sobely ** 2
    return edge_mat


def edges_blurred(grid: np.ndarray, factor: float = 1):
    blurred = gaussian_filter(grid, sigma=factor)
    return edges(blurred)