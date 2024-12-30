from scipy.ndimage import (sobel, gaussian_filter, gaussian_laplace,
                           gaussian_gradient_magnitude)
import numpy as np
import cv2

from typing import Tuple

import pydairlib.perceptive_locomotion.terrain_segmentation.segmentation_utils as utils


def curvature_criterion(
        denoised_and_inpainted_map: np.ndarray, ksize: Tuple[int, int],
        resolution: float) -> np.ndarray:

    # User specified parameters:
    laplacian_blur = 0.045
    scaling = 5.0

    # Calculated parameters
    blur_sigma = int(laplacian_blur / resolution + 0.5)

    # Calc criterion
    curvature = gaussian_laplace(denoised_and_inpainted_map, sigma=blur_sigma)
    below_edges = np.maximum(curvature, np.zeros_like(curvature))
    second_order_safety_score = np.exp((-scaling / resolution) * below_edges)

    return np.minimum(
        second_order_safety_score, np.ones_like(second_order_safety_score)
    )


def inclination_criterion(
        denoised_and_inpainted_map: np.ndarray, ksize: Tuple[int, int],
        resolution: float) -> np.ndarray:

    inclination, _ = \
        utils.CalculateNormalsAndSquaredError(denoised_and_inpainted_map, ksize[0], resolution)
    return np.power(inclination, 2)
