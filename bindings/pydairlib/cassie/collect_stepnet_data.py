import os
import sys
import math
import pickle
import argparse
import numpy as np
from PIL import Image
import multiprocessing
from pydantic import BaseModel
from dataclasses import dataclass
from torch import save as pt_save
from pydairlib.cassie.cassie_gym.config_types import \
    DataCollectionParams, DepthCameraInfo, save_config
from pydairlib.cassie.cassie_gym.stepnet_data_generator import \
    StepnetDataGenerator, test_data_collection, test_flat_collection


HOME = os.getenv("HOME")
DATASET_DIR = HOME + '/workspace/stepnet_learning_data/step_3d_no_terrain/'
FLAT_GROUND_DATASET_DIR = HOME + '/workspace/stepnet_learning_data/flat_ground/'
DEPTH_DIR = DATASET_DIR + 'depth/'
ROBO_DIR = DATASET_DIR + 'robot/'


def ndigits(number):
    return int(math.log10(number)) + 1


def collect_data_from_random_map(size, seed, params):
    np.random.seed(seed)
    env = StepnetDataGenerator.make_randomized_env(
        params.generator_params,
        params.randomization_bounds,
        visualize=False
    )
    data = []
    for i in range(size):
        data.append(env.get_stepnet_data_point(seed=seed+i))
    env.free_sim()
    return data


def collect_and_save_data_from_random_map(i, size, params: DataCollectionParams):
    data = collect_data_from_random_map(size, i*params.nsteps, params)
    print(f'map {i}, t = {data[0]["time"]}')
    ni = ndigits(params.nmaps)
    nj = ndigits(params.nsteps)
    for j, step in enumerate(data):
        # Proces the depth image and save as PNG
        depth = np.nan_to_num(step['depth'], posinf=0).squeeze()
        depth = (params.depth_scale * depth).astype('uint16')
        im = Image.fromarray(depth)
        robot = {key: step[key] for key in ['state', 'target', 'time', 'error']}
        im.save(os.path.join(params.depth_path, f'{i:0{ni}}_{j:0{nj}}.png'))
        pt_save(robot, os.path.join(params.robot_path, f'{i:0{ni}}_{j:0{nj}}.pt'))


def collect_flat_ground_data(size, seed, params):
    env = StepnetDataGenerator.make_flat_env(params.generator_params)
    data = []
    for i in range(size):
        data.append(env.get_flat_ground_stepnet_datapoint(seed=seed+i))
        print(seed+i)
    env.free_sim()
    for i, entry in enumerate(data):
        pt_save(entry, os.path.join(params.robot_path, f'{seed + i}.pt'))


def flat_main(collection_params):
    if not os.path.isdir(collection_params.robot_path):
        os.makedirs(collection_params.robot_path)

    nsteps = collection_params.nsteps
    nthreads = collection_params.nthreads
    batch_size = collection_params.nmaps
    for j in range(int(nsteps/ (nthreads * batch_size))):
        with multiprocessing.Pool(nthreads) as pool:
            results = [
                pool.apply_async(
                    collect_flat_ground_data,
                    (batch_size, nthreads * batch_size * j +
                     batch_size * i,
                     collection_params)
                ) for i in range(nthreads)]
            [result.wait(timeout=batch_size*5) for result in results]


def terrain_main(collection_params):
    for path in [collection_params.robot_path,
                 collection_params.depth_path]:
        if not os.path.isdir(path):
            os.makedirs(path)

    for j in range(int(collection_params.nmaps / collection_params.nthreads)):
        with multiprocessing.Pool(collection_params.nthreads) as pool:
            results = [
                pool.apply_async(
                    collect_and_save_data_from_random_map,
                    (collection_params.nthreads * j + i,
                     collection_params.nsteps,
                     collection_params)
                ) for i in range(collection_params.nthreads) ]
            [result.wait(timeout=collection_params.nsteps) for result in results]


def main(args):
    # Set dataset collection params
    if args.terrain == 'flat':
        args.dataset_parent_folder += 'flat'

    collection_params = DataCollectionParams(
        nmaps=args.nmaps,
        nsteps=args.nsteps,
        nthreads=args.nthreads,
        robot_path=os.path.join(args.dataset_parent_folder, 'robot'),
        depth_path=os.path.join(args.dataset_parent_folder, 'depth'),
        has_terrain=(args.terrain == 'varied')
    )
    if collection_params.has_terrain:
        collection_params.target_to_map_tf = \
            DepthCameraInfo().get_pelvis_to_image_tf()

    if not os.path.isdir(args.dataset_parent_folder):
        os.makedirs(args.dataset_parent_folder)

    # Save collection params
    save_config(
        collection_params,
        os.path.join(args.dataset_parent_folder, 'dataset_config.yaml')
    )

    # run the data collection
    if collection_params.has_terrain:
        terrain_main(collection_params)
    else:
        flat_main(collection_params)


def test():
    test_data_collection()


def save_config_test(args):
    collection_params = DataCollectionParams(
        nmaps=args.nmaps,
        nsteps=args.nsteps,
        nthreads=args.nthreads,
        robot_path=os.path.join(args.dataset_parent_folder, 'robot'),
        depth_path=os.path.join(args.dataset_parent_folder, 'depth'),
        has_terrain=(args.terrain == 'varied')
    )
    if collection_params.has_terrain:
        collection_params.target_to_map_tf = \
            DepthCameraInfo().get_pelvis_to_image_tf()

    save_config(collection_params, '/home/brian/workspace/test_config.yaml')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--terrain', type=str, default='varied', help='\'varied\' or \'flat\'')
    parser.add_argument('--nmaps', type=int, default=50000)
    parser.add_argument('--nsteps', type=int, default=10)
    parser.add_argument('--nthreads', type=int, default=5)
    parser.add_argument('--dataset_parent_folder', type=str, default=DATASET_DIR)

    try:
        args = parser.parse_args()
    except:
        parser.print_help()
        sys.exit(0)
    main(args)
    # test()
