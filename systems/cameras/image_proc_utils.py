import skimage
import numpy as np
from PIL import Image, ImageFilter
from skimage.measure import block_reduce
from dataclasses import dataclass
import matplotlib.pyplot as plt
import matplotlib.patches as patches

@dataclass
class AugmentationParameters:
    augment: bool = True
    noise_variance: float = 0.1
    edge_detect_threshold: float = 0.005
    edge_drop_prob: float = 0.4
    edge_noise_variance: float = 0.5
    random_drop_prob: float = 0.001
    blur_sigma: float = 0.3
    downsample: int = 4


def dict_to_params(params_dict):
    return AugmentationParameters(params_dict["augment"],
                                  params_dict["noise_variance"],
                                  params_dict["edge_detect_threshold"],
                                  params_dict["edge_drop_prob"],
                                  params_dict["edge_noise_variance"],
                                  params_dict["random_drop_prob"],
                                  params_dict["blur_sigma"],
                                  params_dict["downsample"])


# Randomly corrupt pixels on the edge
def corrupt_edges(image,
                  edge_detect_threshold,
                  edge_drop_prob,
                  edge_noise_variance):
    edges = skimage.filters.sobel(image)
    edges[edges < edge_detect_threshold] = 0
    edge_idxs = np.nonzero(edges)
    pixels_to_drop = np.random.rand(edge_idxs[0].shape[0]) < edge_drop_prob
    image[edge_idxs[0][pixels_to_drop], edge_idxs[1][pixels_to_drop]] += np.random.randn()*edge_noise_variance
    return image

# Add gaussian noise to image
def add_noise(image, noise_variance):
    return image + np.random.randn(image.shape[0], image.shape[1]) * noise_variance

# Randomly dropout pixels throughout the image
def universal_dropout(image, drop_prob):
    dropout = np.random.rand(image.shape[0], image.shape[1]) < drop_prob
    xx, yy = np.meshgrid(np.arange(image.shape[0]), np.arange(image.shape[1]))
    image[xx.flatten()[dropout.flatten()], yy.flatten()[dropout.flatten()]] = 0
    return image

# blur the image
def blur_image(image, sigma):
    return skimage.filters.gaussian(image, sigma)


def downsample_image(image, factor):
    depth_downsampled = block_reduce(image, block_size=(factor, factor), func=np.mean)
    return depth_downsampled


def process_image(im, target, scale, params, debug=False):
    depth = np.array(im, dtype=float)/scale
    if params.augment:
        depth = corrupt_edges(depth, params.edge_detect_threshold, params.edge_drop_prob, params.edge_noise_variance)
        depth = add_noise(depth, params.noise_variance)
        depth = universal_dropout(depth, params.random_drop_prob)
        depth = blur_image(depth, params.blur_sigma)
    if debug:
        print(f"target pixel = {target_pixel}")
        fig, ax = plt.subplots()
        ax.set_title("Uncropped, Original Res Augmented Image")
        ax.imshow(depth, cmap='gray')
    depth_downsampled = block_reduce(depth_c, block_size=(4,4), func=np.mean)
    return depth_downsampled
