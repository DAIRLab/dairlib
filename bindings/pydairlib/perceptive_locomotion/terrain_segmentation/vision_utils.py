import cv2
import numpy as np
from scipy.ndimage import sobel, gaussian_filter


def edges(grid: np.ndarray) -> np.ndarray:
    sobelx = sobel(grid, axis=0)
    sobely = sobel(grid, axis=1)
    edge_mat = np.sqrt(sobelx ** 2 + sobely ** 2)
    return edge_mat


def edges_blurred(grid: np.ndarray, factor: float = 1):
    blurred = gaussian_filter(grid, sigma=factor)
    return edges(blurred)


def extract_boundaries(binary_image):
    contours, _ = cv2.findContours(
        binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    boundaries_image = np.zeros_like(binary_image)
    cv2.drawContours(boundaries_image, contours, -1, (255), 1)

    return contours, boundaries_image


def get_safe_terrain(height_map: np.ndarray, resolution):
    height_map = cv2.medianBlur(height_map, 5)
    edge_mask = edges_blurred(height_map, 1.5)
    edge_mask = edge_mask / (np.sqrt(2) * resolution)

    safe = np.logical_and(~np.isnan(edge_mask), edge_mask < 2)
    safe = 255 * (safe.astype(np.uint8))

    kernel = np.ones((3, 3), np.uint8)

    # Perform morphological opening and closing to remove small
    # negative and positive artifacts
    safe_open = cv2.morphologyEx(safe, cv2.MORPH_OPEN, kernel)
    safe_closed = cv2.morphologyEx(safe_open, cv2.MORPH_CLOSE, kernel)
    regions, boundaries_image = extract_boundaries(safe_closed)

    cv2.imshow('safe', safe)
    cv2.imshow('safe_eroded', safe_closed)
    cv2.imshow('boundary', boundaries_image)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
    