import torch
import torch_utils
from torch.utils.data import Dataset, DataLoader, random_split
import torch.nn.functional as F
from tqdm import tqdm
import wandb
import os
import argparse
from dataclasses import dataclass
from network import ResidualLQRPredictionNet
from unet import UNet

perception_learning_base_folder = \
    "bindings/pydairlib/perceptive_locomotion/perception_learning"

default_checkpoint_path = os.path.join(
    perception_learning_base_folder, 'tmp/best_model_checkpoint.pth')


@dataclass
class Hyperparams:
    batch_size: int = 32
    epochs: int = 400
    shuffle: bool = True
    learning_rate: float = 1e-5
    patience: int = 20
    project: str = 'alip-lqr-residual'
    loss: str = 'huber'
    optimizer: str = 'Adam'
    data_path: str = None

    def log_to_wandb(self):
        wandb.config['batch_size'] = self.batch_size
        wandb.config['shuffle'] = self.shuffle
        wandb.config['lr'] = self.learning_rate
        wandb.config['optimizer'] = self.optimizer
        wandb.config['loss'] = self.loss
        wandb.config['patience'] = self.patience


def run_epoch(model, data_loader, device, loss_function, optimizer=None, is_training=True):
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
            if loss_function == 'mse':
                loss = F.mse_loss(predictions, residual)
            elif loss_function == 'huber':
                # TODO (@Brian-Acosta) set delta automatically from data
                loss = F.huber_loss(predictions, residual, delta=1)
            elif loss_function == 'l1':
                loss = F.l1_loss(predictions, residual)
            else:
                raise RuntimeError(f"Unsupported loss function {loss_function}")
            epoch_loss += loss.item()

            # backpropagate loss and update weights
            if is_training:
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
    return epoch_loss / len(data_loader.dataset)


def train_validate_test_split(dataset, split_ratio=(0.7, 0.15, 0.15)):
    total_size = len(dataset)
    train_size = int(split_ratio[0] * total_size)
    val_size = int(split_ratio[1] * total_size)
    test_size = total_size - train_size - val_size

    train_dataset, val_dataset, test_dataset = random_split(
        dataset, [train_size, val_size, test_size],
        generator=torch.Generator()
    )

    return train_dataset, val_dataset, test_dataset


def train_and_test(params: Hyperparams, use_wandb: bool = False) -> None:
    checkpoint_path = default_checkpoint_path
    if use_wandb:
        wandb.init(project="alip-lqr-residual", entity="alip-lqr-residuals")
        params.log_to_wandb()
        checkpoint_path = os.path.join(
            perception_learning_base_folder,
            f'tmp/{wandb.run.name}.pth'
        )

    cassie_dataset = torch_utils.CassieDataset()
    train_dataset, val_dataset, test_dataset = train_validate_test_split(cassie_dataset)

    train_loader = DataLoader(
        train_dataset,
        batch_size=params.batch_size,
        shuffle=params.shuffle,
    )

    val_loader = DataLoader(
        val_dataset,
        batch_size=params.batch_size,
        shuffle=False,
    )

    test_loader = DataLoader(
        test_dataset,
        batch_size=params.batch_size,
        shuffle=False,
    )

    device = torch_utils.get_device()
    model = UNet(7, 1)
    for param in model.parameters():
        param.data.uniform_(-5e-3, 5e-3)

    # set the last layer to zeros
    # for param in model.outc.parameters():
    #     param.data.zero_()

    model.to(device)

    optimizer = torch.optim.Adam(
        model.parameters(), lr=params.learning_rate, weight_decay=1e-5) \
        if params.optimizer == 'Adam' else \
        torch.optim.SGD(model.parameters(), lr=params.learning_rate)

    best_val_loss = float('inf')  # Set to positive infinity initially

    epochs_since_best_validation_loss = 0
    for epoch in tqdm(range(params.epochs)):
        # Training phase
        train_loss = run_epoch(model, train_loader, device, params.loss, optimizer, is_training=True)

        # Validation phase
        val_loss = run_epoch(model, val_loader, device, params.loss, optimizer=None, is_training=False)

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
            epochs_since_best_validation_loss = 0
        else:
            epochs_since_best_validation_loss += 1

        if epochs_since_best_validation_loss > params.patience:
            break

    # Testing phase
    # load the best model
    model.load_state_dict(torch.load(checkpoint_path))
    # test the data
    test_loss = run_epoch(model, test_loader, device, params.loss, is_training=False)

    # Logging for test loss if using WandB
    if use_wandb:
        wandb.log({
            "test_loss": test_loss
        })

    if use_wandb:
        wandb.finish()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--data_path',
        type=str,
        default=None,
    )
    parser.add_argument(
        '--use_wandb',
        type=bool,
        default=True,
    )
    args = parser.parse_args()

    train_and_test(
        Hyperparams(data_path=args.data_path),
        use_wandb=True
    )
