import os
import multiprocessing

from torch import save
from pydairlib.cassie.cassie_gym.stepnet_data_generator import StepnetDataGenerator

NMAPS = 20000
NSTEPS = 10
NTHREADS = 10

DATASET_DIR = '.learning_data/dataset/'


def collect_data_from_random_map(size):
    env = StepnetDataGenerator.make_randomized_env()
    data = []
    for i in range(size):
        data.append(env.get_stepnet_data_point())
    env.free_sim()
    return data


def collect_and_save_data_from_random_map(i, size):
    print(i)
    data = collect_data_from_random_map(size)
    save(data, os.path.join(DATASET_DIR, f'{i}.pt'))


def main():
    with multiprocessing.Pool(NTHREADS) as pool:
        results = [
            pool.apply_async(
                collect_and_save_data_from_random_map,
                (i, NSTEPS)
            ) for i in range(NMAPS) ]
        [result.wait() for result in results]


if __name__ == "__main__":
    main()
