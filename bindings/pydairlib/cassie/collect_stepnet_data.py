import os
import multiprocessing
from PIL import Image
from torch import save as pt_save
from pydairlib.cassie.cassie_gym.stepnet_data_generator import \
    StepnetDataGenerator, test_data_collection

NMAPS = 2000
NSTEPS = 100
NTHREADS = 1

DATASET_DIR = '.learning_data/dataset/'
DEPTH_DIR = DATASET_DIR + 'depth/'
ROBO_DIR = DATASET_DIR + 'robot/'


def collect_data_from_random_map(size):
    env = StepnetDataGenerator.make_randomized_env(visualize=True)
    data = []
    for i in range(size):
        print(i)
        data.append(env.get_stepnet_data_point())
    env.free_sim()
    return data


def collect_and_save_data_from_random_map(i, size):
    data = collect_data_from_random_map(size)
    for i, step in enumerate(data):
        depth = Image.fromarray(step['depth'])
        robot = {key: step[key] for key in ['state', 'target', 'error']}

        depth.save(os.path.join(DEPTH_DIR, f'{i}'))
        pt_save(robot, os.path.join(DATASET_DIR, f'{i}.pt'))


def main():
    if not os.path.isdir(DEPTH_DIR):
        os.mkdir(DEPTH_DIR)
    if not os.path.isdir(ROBO_DIR):
        os.mkdir(ROBO_DIR)
    with multiprocessing.Pool(NTHREADS) as pool:
        results = [
            pool.apply_async(
                collect_and_save_data_from_random_map,
                (i, NSTEPS)
            ) for i in range(NMAPS) ]
        [result.wait() for result in results]


def test():
    main()


if __name__ == "__main__":
    test_data_collection()
