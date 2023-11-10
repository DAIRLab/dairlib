import os
import pdb
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader
import torchvision
from tqdm import tqdm

if not os.getcwd().split('/')[-1] == 'dairlib':
    raise RuntimeError('To align with bazel-bin, run learning scripts from '
                       'dairlib (e.g. python bindings/pydairlib/'
                       'perceptive_locomotion/perception_learning/inference/'
                       '<FILE>.py')

perception_learning_base_folder = \
    "bindings/pydairlib/perceptive_locomotion/perception_learning"


def get_device():
    """
        :return: Returns the highest performance device available in the current
        pytorch configuration
    """

    if torch.cuda.is_available():
        return torch.device('cuda')

    if torch.backends.mps.is_available():
        return torch.device('mps')

    return torch.device('cpu')


def data_reader():
    """
        :return: Returns the data from tmp folder
    """
    # --- Data values ---
    # 'stance': 0 or 1,
    # 'phase': a scalar float,
    # 'initial_swing_foot_pos': numpy.ndarray (3,)
    # 'q':                      numpy.ndarray (23,)
    # 'v':                      numpy.ndarray (24,)
    # 'desired_velocity':       numpy.ndarray (2,)
    # 'hmap':                   numpy.ndarray (M, N),  M and N can vary
    # 'U':                      numpy.ndarray (2, M, N)
    # 'footstep_command':       numpy.ndarray (3,)
    # 'x_k':                    numpy.ndarray (4,)
    # 'V_kp1': float,
    # 'x_kp1':                  numpy.ndarray (4,)
    # 'V_k': float,
    # 'residual': float
    
    data = np.load(
        os.path.join(
            perception_learning_base_folder,
            'tmp/data.npz'
        ), allow_pickle=True
    )
    return data


# try writing it as a torch.utils.Dataset function
class CassieDataset(Dataset):
    """
        Pytorch internal Dataset utility, used for efficient batch Dataloader
    """
    def __init__(self, data_list):
        self.data_list = data_list

    def __len__(self):
        return len(self.data_list)

    def __getitem__(self, idx):
        data_point = self.data_list[idx]
        input_data = tile_and_concatenate_inputs(
            data_point['hmap'], data_point['x_k'], data_point['U']
        )
        target_data = {
            'residual': torch.tensor(
                data_point['residual'], dtype=torch.float32
            ),
            'i': data_point['i'],
            'j': data_point['j'],
        }
        return input_data, target_data


def tile_and_concatenate_inputs(heightmap, state, input_space):
    """
        function of tiling state and input to hmap grid, written for a single
        data point
        :return: process input (hamp, tiled state and input) and target (
        residual)
        for a single data point
    """
    # Getting the heightmap of size (20,20) for the data point.
    hmap = heightmap.reshape(1, heightmap.shape[0], heightmap.shape[1])

    # Tile the initial state to the hmap dimemsions
    initial_state_tiled = np.broadcast_to(
        state[:, np.newaxis, np.newaxis],
        (state.shape[0], hmap.shape[1], hmap.shape[2])
    )

    # Converting the data to tensors
    hmap_tensor = torch.tensor(hmap, dtype=torch.float32)
    initial_state_tensor = torch.tensor(initial_state_tiled, dtype=torch.float32)
    input_space_tensor = torch.tensor(input_space, dtype=torch.float32)

    # concatenate the tiled data for a single data point
    combined_input = torch.cat(
        [hmap_tensor, initial_state_tensor, input_space_tensor],
        dim=0
    )

    return combined_input


def main() -> None:
    """
    Tests on torch related utilities
    1. device test: The output should look something like: tensor([1.], device='cuda:0')
        If no device is shown, you are using the CPU
    2. Dataloader test: test the batch processing and loading using pytorch Dataset, DataLoader
        Check the size
    """

    # device test
    device = get_device()
    x = torch.ones(1, device=device)
    print(x)

    # pytorch Dataloader test
    # load raw data
    data = data_reader()
    data_list = data['arr_0']

    # create custom dataset using raw cassie data
    # This stores the samples and their corresponding labels (combined_input, target)
    cassie_dataset = CassieDataset(data_list)

    # dataloader parameters
    batch_size = 32  # Set your desired batch size
    shuffle = True  # Set to True if you want to shuffle the data
    num_workers = 4  # Number of parallel data loading processes

    data_loader = DataLoader(
        cassie_dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=num_workers
    )

    for batch in tqdm(data_loader):
        input_data, target_data = batch
        print("Batch size: ", len(batch))
        print("Data shape:", input_data.shape)
        print("Labels shape:", target_data['residual'].shape)
        pdb.set_trace()

if __name__ == '__main__':
    main()
