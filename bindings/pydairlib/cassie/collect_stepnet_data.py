import os
import multiprocessing

import numpy as np
from PIL import Image
from torch import save as pt_save
from pydairlib.cassie.cassie_gym.stepnet_data_generator import \
    StepnetDataGenerator, test_data_collection

NMAPS = 4
NSTEPS = 4
NTHREADS = 1

DATASET_DIR = '.learning_data/dataset/'
DEPTH_DIR = DATASET_DIR + 'depth/'
ROBO_DIR = DATASET_DIR + 'robot/'


def collect_data_from_random_map(size):
    env = StepnetDataGenerator.make_randomized_env()
    data = []
    for i in range(size):
        data.append(env.get_stepnet_data_point())
    env.free_sim()
    return data


def collect_and_save_data_from_random_map(i, size):
    data = collect_data_from_random_map(size)
    for j, stp in enumerate(data):
        depth = np.nan_to_num(stp['depth'], posinf=0).squeeze()
        depth = (255 * depth / np.max(depth)).astype('uint8')
        im = Image.fromarray(depth)
        robot = {key: stp[key] for key in ['state', 'target', 'error']}

        im.save(os.path.join(DEPTH_DIR, f'{i}_{j}.png'))
        pt_save(robot, os.path.join(ROBO_DIR, f'{i}_{j}.pt'))
        print('saved!')


def main():
    if not os.path.isdir(DEPTH_DIR):
        os.makedirs(DEPTH_DIR)
    if not os.path.isdir(ROBO_DIR):
        os.makedirs(ROBO_DIR)
    with multiprocessing.Pool(NTHREADS) as pool:
        results = [
            pool.apply_async(
                collect_and_save_data_from_random_map,
                (i, NSTEPS)
            ) for i in range(NMAPS) ]
        [result.wait() for result in results]


def test():
    test_data_collection()


if __name__ == "__main__":
    main()
