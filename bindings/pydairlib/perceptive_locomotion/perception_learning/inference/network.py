import torch
from torch import nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
from torch_utils import get_device
from torch.optim import SGD
import torchinfo
from tqdm import tqdm

import torch_utils

NUM_CHANNELS_STATE_VECTOR = 4              # Since each x_k is 4 long.
NUM_CHANNELS_INPUT_VECTOR = 2              # Since each footstep_command is 2 long.

class ResidualLQRPredictionNet(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_dim_1 = 16, hidden_dim_2 = 32):
        super(ResidualLQRPredictionNet, self).__init__()
        # input_dim here is the number of channels in the full input image. This is 7.
        # The first channel is the height map. The next 4 channels are the x_k. The last 2 channels are the footstep_command.
        self.input_dim = input_dim
        # output_dim here is the number of channels in the final output image. Should be 1.
        self.output_dim = output_dim
        # input dim for first layer
        self.layer_1_input_dim = 1
        # hidden_dim_1 and hidden_dim_2 are the number of channels in the hidden layers. May need to be tuned/vary per layer.
        # These represent the features that the network will learn.
        self.hidden_dim_1 = hidden_dim_1
        self.hidden_dim_2 = hidden_dim_2

        # The following two layers are to extract features from the hmap.
        # This is the first convolutional layer. This will take the input image (hmap) and output a hidden_dim_1 channel image.
        # The kernel size is 3x3, and the padding is 1.
        # TODO: Try changing kernel size and padding accordingly if needed to improve performance.
        self.conv1 = nn.Conv2d(in_channels = self.layer_1_input_dim, out_channels = self.hidden_dim_1, kernel_size = 3, padding = 1)
        # This is the second convolutional layer. This will take the hidden_dim channel image from the first convolutional layer
        # and output a hidden_dim_2 channel image.
        self.conv2 = nn.Conv2d(in_channels = self.hidden_dim_1, out_channels = self.hidden_dim_2, kernel_size = 3, padding = 1)
        
        # Convolution layer to process 4 channel state vector input. 
        # As of now, this takes in the state vector as a 4 channel image and outputs a hidden_dim_1 channel image.
        self.conv3 = nn.Conv2d(in_channels = NUM_CHANNELS_STATE_VECTOR, out_channels = self.hidden_dim_1, kernel_size = 3, padding = 1)

        # Convolution layer to process the 2 channel input of footstep locations.
        # As of now, this takes in the footstep locations as a 2 channel image and outputs a hidden_dim_1 channel image.
        self.conv4 = nn.Conv2d(in_channels = NUM_CHANNELS_INPUT_VECTOR, out_channels = self.hidden_dim_1, kernel_size = 3, padding = 1)

        # The concatenated image from the previous layers will be passed through another convolutional layer to output a 1x20x20 output.
        self.conv5 = nn.Conv2d(in_channels = self.hidden_dim_2 + self.hidden_dim_1 + self.hidden_dim_1, out_channels = self.output_dim, kernel_size = 3, padding = 1)
    
    # Define the forward function that takes in the input data and returns the output data.
    def forward(self, input_data):
        # The input data is of shape (batch_size, num_channels, height, width).

        # Get the hmap from the input data. (channel 1)
        # The hmap is the first channel of the input data (channel 1)
        # print("This is my input data size = ", input_data.shape)                       # expected --> (32, 7, 20, 20)
        hmap = input_data[:, 0:1 , :, :]
        # print("shape of hmap is: ", hmap.shape)

        # Get the x_k from the input data. (channel 2-5)
        x_k = input_data[:, 1:5, :, :]
        # print("shape of x_k is: ", x_k.shape)

        # Get the footstep_command from the input data. (channel 6-7)
        footstep_command = input_data[:, 5:7, :, :]
        # print("shape of footstep_command is: ", footstep_command.shape)

        # Apply conv1 to the hmap and apply relu to it.
        conv1_hmap = F.relu(self.conv1(hmap))
        # print("shape of conv1_hmap is: ", conv1_hmap.shape)

        # Apply conv2 to the output of conv1 and apply relu to it.
        conv2_hmap = F.relu(self.conv2(conv1_hmap))
        # print("shape of conv2_hmap is: ", conv2_hmap.shape)

        # Apply conv3 to the x_k and apply relu to it.
        conv3_x_k = F.relu(self.conv3(x_k))
        # print("shape of conv3_x_k is: ", conv3_x_k.shape)

        # Apply conv4 to the footstep_command and apply relu to it.
        conv4_footstep_command = F.relu(self.conv4(footstep_command))
        # print("shape of conv4_footstep_command is: ", conv4_footstep_command.shape)

        # Concatenate the output of conv2, conv3, and conv4.
        # The output of conv2 is of shape (batch_size, hidden_dim_2, height, width).
        # The output of conv3 is of shape (batch_size, hidden_dim_1, height, width).
        # The output of conv4 is of shape (batch_size, hidden_dim_1, height, width).
        # The concatenated output is of shape (batch_size, hidden_dim_2 + hidden_dim_1 + hidden_dim_1, height, width).
        concatenated_output = torch.cat([conv2_hmap, conv3_x_k, conv4_footstep_command], dim = 1)
        # print("shape of concatenated_output is: ", concatenated_output.shape)

        # Apply conv5 to the concatenated output and apply relu to it.
        conv5_output = F.relu(self.conv5(concatenated_output))
        # print("shape of conv5_output is: ", conv5_output.shape)

        # Return the output of conv5
        return conv5_output


def loss_function(prediction, target):
    '''
    :param prediction: output of the model
    :param target: ground truth residual
    :return: MSE loss
    ''' 
    return F.mse_loss(prediction, target)


def main():
    model = ResidualLQRPredictionNet(7, 1)
    device = get_device()
    model = model.to(device)
    learning_rate = 0.01

    cassie_dataset = torch_utils.CassieDataset()

    # dataloader parameters
    batch_size = 32  # Set your desired batch size
    shuffle = True  # Set to True if you want to shuffle the data
    num_workers = 4  # Number of parallel data loading processes

    # create dataloader
    # This wraps an iterable around the Dataset to enable easy access to the samples.
    data_loader = DataLoader(cassie_dataset, batch_size=batch_size, shuffle=shuffle, num_workers=num_workers)
    
    # print('length of dataloader = ',len(data_loader))
    # Define an optimizer
    optimizer = SGD(model.parameters(), lr=learning_rate)
    
    # Train the model
    for batch in tqdm(data_loader):
        # get input and target data and move them to the same device as model
        input_data = batch[0].to(device)
        i = batch[1]['i']
        j = batch[1]['j']
        residual = batch[1]['residual'].to(device)

        # pass input data into model
        predictions = model.forward(input_data)
        predictions = predictions[torch.arange(predictions.shape[0]), 0, i, j]

        # compute loss
        loss = loss_function(predictions, residual)

        # backpropagate loss and update weights
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    summary_str = torchinfo.summary(model, input_size=(batch_size, 7, 20, 20))
    print(summary_str)

if __name__ == '__main__':
    main()