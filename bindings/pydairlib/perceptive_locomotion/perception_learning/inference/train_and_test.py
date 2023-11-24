import torch
import torch_utils
from torch.utils.data import Dataset, DataLoader, random_split
import torch.nn.functional as F
from tqdm import tqdm
import wandb
import os
from dataclasses import dataclass
from network import ResidualLQRPredictionNet
from unet import UNet

perception_learning_base_folder = \
    "bindings/pydairlib/perceptive_locomotion/perception_learning"

checkpoint_path = os.path.join(
    perception_learning_base_folder, 'tmp/best_model_checkpoint.pth')


@dataclass
class Hyperparams:
    batch_size: int = 32
    epochs: int = 200
    shuffle: bool = True
    num_workers: int = 1
    learning_rate: float = 1e-4
    project: str = 'alip-lqr-residual'
    optimizer: str = 'Adam'

    def log_to_wandb(self):
        wandb.config['batch_size'] = self.batch_size
        wandb.config['shuffle'] = self.shuffle
        wandb.config['num_workers'] = self.num_workers
        wandb.config['lr'] = self.learning_rate
        wandb.config['optimizer'] = self.optimizer


def run_epoch(model, data_loader, device, optimizer=None, is_training=True):
    epoch_loss = 0

    model.train() if is_training else model.eval()

    with torch.set_grad_enabled(is_training):
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
            epoch_loss += loss.item()

            # backpropagate loss and update weights
            if is_training:
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

    return epoch_loss / len(data_loader.dataset)


def train_validate_test_split(dataset, split_ratio=(0.7, 0.15, 0.15), seed=3407):
    total_size = len(dataset)
    train_size = int(split_ratio[0] * total_size)
    val_size = int(split_ratio[1] * total_size)
    test_size = total_size - train_size - val_size

    train_dataset, val_dataset, test_dataset = random_split(
        dataset, [train_size, val_size, test_size],
        generator=torch.Generator().manual_seed(seed)
    )

    return train_dataset, val_dataset, test_dataset


def train_and_test(params: Hyperparams, use_wandb: bool = False) -> None:
    if use_wandb:
        wandb.init(project="alip-lqr-residual", entity="alip-lqr-residuals")
        params.log_to_wandb()

    cassie_dataset = torch_utils.CassieDataset()
    train_dataset, val_dataset, test_dataset = train_validate_test_split(cassie_dataset)

    train_loader = DataLoader(
        train_dataset,
        batch_size=params.batch_size,
        shuffle=params.shuffle,
        num_workers=params.num_workers
    )

    val_loader = DataLoader(
        val_dataset,
        batch_size=params.batch_size,
        shuffle=False,
        num_workers=params.num_workers
    )

    test_loader = DataLoader(
        test_dataset,
        batch_size=params.batch_size,
        shuffle=False,
        num_workers=params.num_workers
    )

    device = torch_utils.get_device()
    model = UNet()
    model.to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=params.learning_rate) \
        if params.optimizer == 'Adam' else \
        torch.optim.SGD(model.parameters(), lr=params.learning_rate)

    best_val_loss = float('inf')  # Set to positive infinity initially

    for epoch in tqdm(range(params.epochs)):
        # Training phase
        train_loss = run_epoch(model, train_loader, device, optimizer, is_training=True)

        # Validation phase
        val_loss = run_epoch(model, val_loader, device, optimizer=None, is_training=False)

        # Logging for training and validation losses if using WandB
        if use_wandb:
            wandb.log({
                "epoch": epoch,
                "train_loss": train_loss,
                "val_loss": val_loss
            })

        # Save the model if the validation loss improves
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), checkpoint_path)

    # Testing phase
    # load the best model
    model.load_state_dict(torch.load(checkpoint_path))
    # test the data
    test_loss = run_epoch(model, test_loader, device, is_training=False)

    # Logging for test loss if using WandB
    if use_wandb:
        wandb.log({
            "test_loss": test_loss
        })

    if use_wandb:
        wandb.finish()


if __name__ == '__main__':
    train_and_test(Hyperparams(), use_wandb=False)
