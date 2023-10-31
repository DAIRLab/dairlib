import os
import numpy as np
import torch
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


def data_process(data):
    """
        :param data: The data from data_loader()
        :return: Returns the processed data in the form of a tensor of shape (number of data points, 7, 20, 20) and the target tensor of shape (number of data points, 1, 20, 20)
    """
    
    # data is a list of dictionaries by the name arr_0.
    # Getting list
    data_list = data['arr_0']
    
    # Create tensors of length = number of datapoints in data_list for input data and target data
    concatenated_data = []
    ground_truth_residual = []

    # Iterating over each datapoint in list while displaying a progress bar

    import pdb; pdb.set_trace()

    for data_point in tqdm(data_list):
        hmap = data_point['hmap']                                                               # Getting the heightmap of size (20,20) for the data point.
        # Get hmap shape and reshape to (1,hmap.shape[0],hmap.shape[1])
        hmap = hmap.reshape(1, hmap.shape[0],hmap.shape[1])
        # print("shape of hmap is: ", hmap.shape)

        U = data_point['U']                                                                     # Getting all possible control inputs for the data point.
        # print("shape of U is: ", U.shape)

        initial_state = data_point['x_k']                                                       # Getting the initial state of the data point.
        # print("shape of initial_state is: ", initial_state.shape)

        # Repeat the initial state for all points in hmap to get an array of shape (4,20,20)
        initial_state = np.broadcast_to(initial_state[:, np.newaxis, np.newaxis], (initial_state.shape[0], hmap.shape[1], hmap.shape[2]))
        # print("shape of initial_state after tiling is: ", initial_state.shape)                

        # Converting the data to tensors
        hmap_tensor = torch.tensor(hmap, dtype=torch.float32)
        initial_state_tensor = torch.tensor(initial_state, dtype=torch.float32)
        U_tensor = torch.tensor(U, dtype=torch.float32)

        # concatenate the tiled data for a single data point
        combined_input = torch.cat([hmap_tensor, initial_state_tensor, U_tensor], dim=0)        # Confirm axis. Final shape should be 7,20,20
        
        # Append the combined input to the list
        concatenated_data.append(combined_input)

        # Stack the list of concatenated tensors into a single tensor
        input_data_tensor = torch.stack(concatenated_data, dim=0)

        # ground truth residual data
        residual = data_point['residual']
        target = np.zeros((1, hmap.shape[1], hmap.shape[2]), dtype=float)
        target[:, :, :] = residual
        target_tensor = torch.tensor(target, dtype=torch.float32)
        # Append the target tensor to the list
        ground_truth_residual.append(target_tensor)
        # Stack the list of target tensors into a single tensor
        ground_truth_residual_tensor = torch.stack(ground_truth_residual, dim=0)

    return input_data_tensor, ground_truth_residual_tensor


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
    
    data = data_reader()
    input_data_tensor, ground_truth_residual = data_process(data)
    print("input shape ", input_data_tensor.shape, "target shape ", ground_truth_residual.shape)   # Expected shape is (number of data points, 7, 20, 20)


if __name__ == '__main__':
    main()
