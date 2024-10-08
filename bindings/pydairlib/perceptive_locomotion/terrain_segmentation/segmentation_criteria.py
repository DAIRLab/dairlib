from scipy.ndimage import (sobel, gaussian_filter, gaussian_laplace,
                           gaussian_gradient_magnitude)
import numpy as np
import cv2

from typing import Tuple

import pydairlib.perceptive_locomotion.terrain_segmentation.segmentation_utils as utils


def curvature_criterion(
        elevation_inpainted: np.ndarray, ksize: Tuple[int, int],
        resolution: float) -> np.ndarray:

    # User specified parameters:
    laplacian_blur = 0.045
    scaling = 4.5

    # Calculated parameters
    blur_sigma = int(laplacian_blur / resolution + 0.5)

    # Calc criterion
    median = cv2.medianBlur(elevation_inpainted, 5)
    curvature = gaussian_laplace(median, sigma=blur_sigma)
    below_edges = np.maximum(curvature, np.zeros_like(curvature))
    second_order_safety_score = np.exp((-scaling / resolution) * below_edges)

    return np.minimum(
        second_order_safety_score, np.ones_like(second_order_safety_score)
    )


def variance_criterion(
        elevation_inpainted: np.ndarray, ksize: Tuple[int, int],
        resolution: float) -> np.ndarray:

    # User specified parameters
    variance_blur = 0.045
    safe_inf_norm = 0.04
    scaling = 25.0

    lowpass = cv2.boxFilter(elevation_inpainted, -1, ksize, normalize=True)

    stddev = gaussian_filter(
        np.abs(elevation_inpainted - lowpass), variance_blur
    )

    dilation_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ksize)
    stddev = cv2.dilate(stddev, dilation_kernel)

    var_safety_score = np.minimum(
        np.ones_like(stddev),
        np.exp(scaling * (safe_inf_norm - stddev))
    )

    return var_safety_score


def inclination_criterion(
        elevation_inpainted: np.ndarray, ksize: Tuple[int, int],
        resolution: float) -> np.ndarray:

    median = cv2.medianBlur(elevation_inpainted, 5)
    inclination, _ = \
        utils.CalculateNormalsAndSquaredError(median, ksize[0], resolution)
    return np.power(inclination, 2)
