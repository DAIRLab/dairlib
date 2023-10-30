import os
import numpy as np
import torch
import torchvision
from tqdm import tqdm

# from pydairlib.perceptive_locomotion.perception_learning. \
#     cassie_footstep_controller_environment import (perception_learning_base_folder)



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

def data_loader():
    """
        :return: Returns the data from tmp folder
    """
    # TODO: The data collection will return a list of dictionaries. Each dictionary should contain the following:
    # 'stance': 1,
    # 'phase': 0.1805009999999996,
    # 'initial_swing_foot_pos': numpy.ndarray,  # An array with shape (3,)
    # 'q': numpy.ndarray,  # An array with shape (24,)
    # 'v': numpy.ndarray,  # An array with shape (24,)
    # 'desired_velocity': numpy.ndarray,  # An array with shape (2,)
    # 'hmap': numpy.ndarray,  # An array with shape (3, M, N) where M and N can vary
    # 'footstep_command': numpy.ndarray,  # An array with shape (3,)
    # 'x_k': numpy.ndarray,  # An array with shape (4,)
    # 'V_kp1': float,
    # 'x_kp1': numpy.ndarray,  # An array with shape (4,)
    # 'V_k': float,
    # 'residual': float
    
    # load the data file by the name of 'data.npz'
    # TODO: Change the path to the data file to be based on the perception_learning_base_folder
    # data_path = os.path.join(perception_learning_base_folder, 'tmp', allow_pickle=True)
    # data = np.load(data_path, 'data.npz')
    
    # TODO: Remove this when path is updated as above
    data = np.load('/home/sharanya/workspace/cis7000/dairlib/bindings/pydairlib/perceptive_locomotion/perception_learning/tmp/data.npz', allow_pickle=True)
    return data

def data_process(data):
    """
        :param data: The data from data_loader()
        :return: Returns the tiled data where each data point is tiled to the size of the heightmap
    """
    
    # data is a list of dictionaries by the name arr_0.
    # Getting list
    data_list = data['arr_0']
    
    # Create a tensor of length = number of datapoints in data_list
    concatenated_data = []

    # Iterating over each datapoint in list while displaying a progress bar
    for data_point in tqdm(data_list):
        hmap = data_point['hmap']                                                # Getting the heightmap of size (20,20) for the data point.
        # Get hmap shape and reshape to (1,hmap.shape[0],hmap.shape[1])
        hmap = hmap.reshape(1, hmap.shape[0],hmap.shape[1])
        # print("shape of hmap is: ", hmap.shape)

        U = data_point['U']                                                      # Getting all possible control inputs for the data point.
        # print("shape of U is: ", U.shape)

        initial_state = data_point['x_k']                                        # Getting the initial state of the data point.
        # print("shape of initial_state is: ", initial_state.shape)

        # Repeat the initial state for all points in hmap
        initial_state = np.broadcast_to(initial_state[:, np.newaxis, np.newaxis], (4, 20, 20))
        # print("shape of initial_state after tiling is: ", initial_state.shape)   # Expected shape is (4, 20, 20)

        # Converting the data to tensors
        hmap_tensor = torch.tensor(hmap, dtype=torch.float32)
        initial_state_tensor = torch.tensor(initial_state, dtype=torch.float32)
        U_tensor = torch.tensor(U, dtype=torch.float32)

        # concatenate the tiled data for a single data point
        combined_input = torch.cat([hmap_tensor, initial_state_tensor, U_tensor], dim=0)   # Confirm axis. Final shape should be 7,20,20
        
        # Append the combined input to the list
        concatenated_data.append(combined_input)

        # Stack the list of concatenated tensors into a single tensor
        input_data_tensor = torch.stack(concatenated_data, dim=0)

    return input_data_tensor


def main() -> None:
    """
    Tests what device is returned by get_device(). The output should look
    something like:
                       tensor([1.], device='cuda:0')

    If no device is shown, you are using the CPU
    """
    device = get_device()
    x = torch.ones(1, device=device)
    print(x)
    
    data = data_loader()
    input_data_tensor = data_process(data)
    # print("input shape ", input_data_tensor.shape)   # Expected shape is (number of data points, 7, 20, 20)


if __name__ == '__main__':
    main()
