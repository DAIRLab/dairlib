import os
import math
import multiprocessing

import numpy as np
from PIL import Image
from torch import save as pt_save
from pydairlib.cassie.cassie_gym.stepnet_data_generator import \
    StepnetDataGenerator, test_data_collection

NMAPS = 20000
NSTEPS = 10
NTHREADS = 6

HOME = os.getenv("HOME")
DATASET_DIR = HOME + '/workspace/stepnet_learning_data/dataset/'
DEPTH_DIR = DATASET_DIR + 'depth/'
ROBO_DIR = DATASET_DIR + 'robot/'


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
        depth = (255 * depth / max(.001, np.max(depth))).astype('uint8')
        im = Image.fromarray(depth)
        robot = {key: stp[key] for key in ['state', 'target', 'error']}
        im.save(os.path.join(DEPTH_DIR, f'{i:0{ni}}_{j:0{nj}}.png'))
        pt_save(robot, os.path.join(ROBO_DIR, f'{i:0{ni}}_{j:0{nj}}.pt'))


def main():
    if not os.path.isdir(DEPTH_DIR):
        os.makedirs(DEPTH_DIR)
    if not os.path.isdir(ROBO_DIR):
        os.makedirs(ROBO_DIR)

    # collect_and_save_data_from_random_map(0, 10)
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
    main()
