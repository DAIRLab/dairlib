import torch
import torch_utils
from torch.utils.data import Dataset, DataLoader
import torch.nn.functional as F

from unet import UNet
from tqdm import tqdm
import wandb


def train(epochs: int) -> None:
    wandb.init(project="alip-lqr-residual")

    # TODO (@Brian-Acosta) make these externally controllable/optimize-able
    wandb.config['batch_size'] = 32
    wandb.config['shuffle'] = True
    wandb.config['num_workers'] = 4
    wandb.config['lr'] = 0.001

    cassie_dataset = torch_utils.CassieDataset()
    data_loader = DataLoader(
        cassie_dataset,
        batch_size=wandb.config['batch_size'],
        shuffle=wandb.config['shuffle'],
        num_workers=wandb.config['num_workers']
    )
    device = torch_utils.get_device()
    model = UNet(enc_chs=(7, 64, 128, 256, 512), out_sz=(20,20))
    model.to(device)

    optim = torch.optim.Adam(model.parameters(), lr=wandb.config['lr'])
    for i in tqdm(range(epochs)):
        train_loss = 0
        for input_data, output_data in data_loader:
            input_data =input_data.to(device)
            i = output_data['i']
            j = output_data['j']
            residual = output_data['residual'].to(device)

            # pass input data into model
            predictions = model.forward(input_data)
            predictions = predictions[torch.arange(predictions.shape[0]), 0, i, j]

            # compute loss
            loss = F.mse_loss(predictions, residual)
            train_loss += loss

            # backpropagate loss and update weights
            optim.zero_grad()
            loss.backward()
            optim.step()

        wandb.log({"epoch": i, "train_loss": train_loss / len(data_loader)})

    wandb.finish()


if __name__ == '__main__':
    train(100)
