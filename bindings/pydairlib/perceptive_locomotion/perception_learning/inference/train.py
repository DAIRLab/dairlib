import torch
import torch_utils
from torch.utils.data import Dataset, DataLoader
import torch.nn.functional as F

from tqdm import tqdm
import wandb
from dataclasses import dataclass

from network import ResidualLQRPredictionNet
from unet import UNet

@dataclass
class Hyperparams:
    batch_size: int = 32
    epochs: int = 200
    shuffle: bool = True
    num_workers: int = 1
    learning_rate: float = 5e-4
    project: str = 'alip-lqr-residual'
    optimizer: str = 'Adam'

    def log_to_wandb(self):
        wandb.config['batch_size'] = self.batch_size
        wandb.config['shuffle'] = self.shuffle
        wandb.config['num_workers'] = self.num_workers
        wandb.config['lr'] = self.learning_rate
        wandb.config['omptimizer'] = self.optimizer


def train(params: Hyperparams, use_wandb: bool = False) -> None:

    if use_wandb:
        wandb.init(project="alip-lqr-residual", entity="alip-lqr-residuals")
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

    optim = torch.optim.Adam( model.parameters(), lr=params.learning_rate)\
        if params.optimizer == 'Adam' else \
        torch.optim.SGD(model.parameters(), lr=params.learning_rate)

    for epoch in tqdm(range(params.epochs)):
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

        # TODO (@Brian-Acosta) Save best performing checkpoints on validation
        #  data
        if use_wandb:
            wandb.log({
                "epoch": epoch,
                "train_loss": train_loss / len(data_loader) / params.batch_size
            })

    if use_wandb:
        wandb.finish()


if __name__ == '__main__':
    train(Hyperparams(), use_wandb=True)
