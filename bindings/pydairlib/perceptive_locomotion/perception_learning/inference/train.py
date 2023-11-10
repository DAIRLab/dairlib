import torch
import torch_utils
from torch.utils.data import Dataset, DataLoader
import torch.nn.functional as F

from unet import UNet
from tqdm import tqdm
import wandb
from dataclasses import dataclass


@dataclass
class Hyperparams:
    batch_size: int = 32
    epochs: int = 100
    shuffle: bool = True
    num_workers: int = 4
    learning_rate: float = 0.001
    project: str = 'alip-lqr-residual'

    def log_to_wandb(self):
        wandb.config['batch_size'] = self.batch_size
        wandb.config['shuffle'] = self.shuffle
        wandb.config['num_workers'] = self.num_workers
        wandb.config['lr'] = self.learning_rate


def train(params: Hyperparams, use_wandb: bool = False) -> None:

    if use_wandb:
        wandb.init(project="alip-lqr-residual")
        params.log_to_wandb()

    cassie_dataset = torch_utils.CassieDataset()
    data_loader = DataLoader(
        cassie_dataset,
        batch_size=params.batch_size,
        shuffle=params.shuffle,
        num_workers=params.num_workers
    )
    device = torch_utils.get_device()
    model = UNet()

    model.to(device)

    optim = torch.optim.Adam(model.parameters(), lr=params.learning_rate)
    for i in tqdm(range(params.epochs)):
        train_loss = 0
        for input_data, output_data in data_loader:
            input_data = input_data.to(device)
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

        if use_wandb:
            wandb.log({"epoch": i, "train_loss": train_loss / len(data_loader)})

    if use_wandb:
        wandb.finish()


if __name__ == '__main__':
    train(Hyperparams(), use_wandb=True)
