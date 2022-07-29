import os
import math
import multiprocessing

import numpy as np
from PIL import Image
from torch import save as pt_save
from pydairlib.cassie.cassie_gym.stepnet_data_generator import \
    StepnetDataGenerator, test_data_collection, test_flat_collection

# Terrain data collection
NMAPS = 20000
NSTEPS = 10
NTHREADS = 5

# Flat ground data collection
NSTEPS_FG = 10000
BATCH_SIZE_FG = 5
NTHREADS_FG = 5


HOME = os.getenv("HOME")
DATASET_DIR = HOME + '/workspace/stepnet_learning_data/dataset/'
FLAT_GROUND_DATASET_DIR = HOME + '/workspace/stepnet_learning_data/flat_ground/'
DEPTH_DIR = DATASET_DIR + 'depth/'
ROBO_DIR = DATASET_DIR + 'robot/'
DEPTH_SCALE = 255 / 10.0  # depth camera clips at 10 m


def ndigits(number):
    return int(math.log10(number)) + 1


def collect_data_from_random_map(size, seed):
    env = StepnetDataGenerator.make_randomized_env(visualize=False)
    data = []
    for i in range(size):
        data.append(env.get_stepnet_data_point(seed=seed+i))
    env.free_sim()
    return data


def collect_and_save_data_from_random_map(i, size):
    data = collect_data_from_random_map(size, i*NSTEPS)
    print(i)
    ni = ndigits(NMAPS)
    nj = ndigits(NSTEPS)
    for j, stp in enumerate(data):
        depth = np.nan_to_num(stp['depth'], posinf=0).squeeze()
        depth = (DEPTH_SCALE * depth).astype('uint8')
        im = Image.fromarray(depth)
        robot = {key: stp[key] for key in ['state', 'target', 'error']}
        im.save(os.path.join(DEPTH_DIR, f'{i:0{ni}}_{j:0{nj}}.png'))
        pt_save(robot, os.path.join(ROBO_DIR, f'{i:0{ni}}_{j:0{nj}}.pt'))


def collect_flat_ground_data(size, seed):
    env = StepnetDataGenerator.make_flat_env()
    data = []
    for i in range(size):
        data.append(env.get_stepnet_data_point(seed=seed+i))
        print(seed+i)
    env.free_sim()
    for i, entry in enumerate(data):
        pt_save(data, os.path.join(FLAT_GROUND_DATASET_DIR, f'{seed + i}.pt'))


def flat_gound_data_main():
    if not os.path.isdir(FLAT_GROUND_DATASET_DIR):
        os.makedirs(FLAT_GROUND_DATASET_DIR)

    for j in range(int(NSTEPS_FG / NTHREADS_FG / BATCH_SIZE_FG)):
        with multiprocessing.Pool(NTHREADS_FG) as pool:
            results = [
                pool.apply_async(
                    collect_flat_ground_data,
                    (BATCH_SIZE_FG, NTHREADS_FG * j + BATCH_SIZE_FG * i)
                ) for i in range(NTHREADS_FG)]
            [result.wait(timeout=BATCH_SIZE_FG*5) for result in results]


def main():
    if not os.path.isdir(DEPTH_DIR):
        os.makedirs(DEPTH_DIR)
    if not os.path.isdir(ROBO_DIR):
        os.makedirs(ROBO_DIR)

    for j in range(int(NMAPS / NTHREADS)):
        with multiprocessing.Pool(NTHREADS) as pool:
            results = [
                pool.apply_async(
                    collect_and_save_data_from_random_map,
                    (NTHREADS * j + i, NSTEPS)
                ) for i in range(NTHREADS) ]
            [result.wait(timeout=NSTEPS*5) for result in results]


def test():
    test_data_collection()


if __name__ == "__main__":
    test()
    # main()
    # flat_gound_data_main()
