# This script can be run as python bindings/pydairlib/perceptive_locomotion/perception_learning/inference/learner.py
# You can also provide a config file as python bindings/pydairlib/perceptive_locomotion/perception_learning/inference/learner.py --config <config_file>.yaml

################ WANDB HYPERPARAMETER SWEEP ################
# You can also run a sweep as wandb sweep --project alip-lqr-residual bindings/pydairlib/perceptive_locomotion/perception_learning/inference/sweep_config.yaml
# Then run the sweep as wandb agent <sweep_id> available after running the previous command

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

default_checkpoint_path = os.path.join(
    perception_learning_base_folder, 'tmp/best_model_checkpoint.pth')

class Learner:
    """ This class is used to train and compare multiple models based on the parameters provided. It also logs the results of each to wandb. """
    def __init__(self, args=None):
        
        ################
        ## Parameters ##
        ################

        self.args = args

        if self.args is not None:
            print("Using args provided")
            self.use_wandb = args.use_wandb
            self.run_name = args.run_name
            self.model_type = args.model_type
            self.optimizer = args.optimizer

            ################
            ## Dataset ##
            ################
            self.dataset = torch_utils.CassieDataset()
            self.shuffle = args.shuffle
            self.num_workers = args.num_workers
            self.batch_size = args.batch_size
            # self.load_trainval = args.load_trainval
            self.split_ratio = args.split_ratio


            ################
            ## Training ##
            ################
            self.lr = args.lr
            self.patience = args.patience
            self.num_epochs = args.num_epochs
            self.loss = args.loss
            self.save_model_freq = args.save_model_freq
            self.val_freq = args.val_freq
        else:
            print("No args provided, using default values")
            self.use_wandb = True
            self.run_name = 'sharanya-test'
            self.model_type = 'UNet'
            self.optimizer = 'adam'
            self.dataset = torch_utils.CassieDataset()
            self.shuffle = True
            self.num_workers = 1
            # self.load_trainval = False
            self.split_ratio = (0.7, 0.15, 0.15)
            self.lr = 1e-4
            self.patience = 10
            self.batch_size = 32
            self.num_epochs = 100
            self.loss = 'mse'
            self.save_model_freq = 25
            self.val_freq = 10

    def log_to_wandb(self):
        """This function logs the current set of parameters to wandb."""
        wandb.config['batch_size'] = self.batch_size
        wandb.config['shuffle'] = self.shuffle
        wandb.config['num_workers'] = self.num_workers
        wandb.config['lr'] = self.lr
        wandb.config['patience'] = self.patience
        wandb.config['optimizer'] = self.optimizer
        wandb.config['loss'] = self.loss

    def train_validate_test_split(self, dataset, seed=3407):
        """This function splits the dataset into train, validation and test sets."""
        total_size = len(self.dataset)
        train_size = int(self.split_ratio[0] * total_size)
        val_size = int(self.split_ratio[1] * total_size)
        test_size = total_size - train_size - val_size

        train_dataset, val_dataset, test_dataset = random_split(
            self.dataset, [train_size, val_size, test_size],
            generator=torch.Generator().manual_seed(seed)
        )

        return train_dataset, val_dataset, test_dataset

    def run_epoch(self, model, data_loader, device, optimizer=None, is_training=False):
        """This function runs one epoch of training or testing."""
        
        model.train() if is_training else model.eval()

        epoch_loss = 0

        with torch.set_grad_enabled(is_training):
            for input_data, output_data in data_loader:
                input_data = input_data.to(device)
                i = output_data['i']
                j = output_data['j']
                residual = output_data['residual'].to(device)

                # pass input data into model
                predictions = model.forward(input_data)
                predictions = predictions[torch.arange(predictions.shape[0]), 0, i, j]
                
                # compute the loss
                if self.loss == 'mse':
                    loss = F.mse_loss(predictions, residual)
                elif self.loss == 'huber':
                    loss = F.huber_loss(predictions, residual, delta=1)
                elif self.loss == 'l1':
                    loss = F.l1_loss(predictions, residual)
                else:
                    raise RuntimeError(f"Unsupported loss function {self.loss}")
                
                epoch_loss += loss.item()
                
                # backpropagate loss and update weights if training 
                if is_training:
                    optimizer.zero_grad()
                    loss.backward()
                    optimizer.step()

        return epoch_loss / len(data_loader.dataset)

    def train_and_test(self, use_wandb: bool = False) -> None:
        """This function trains and tests the model based on the parameters provided. It also logs the results of each to wandb."""
        
        checkpoint_path = default_checkpoint_path
        if use_wandb:
            wandb.init(project="alip-lqr-residual", entity="alip-lqr-residuals", name=self.run_name)
            self.log_to_wandb()
            checkpoint_path = os.path.join(
            perception_learning_base_folder,
            f'tmp/{wandb.run.name}.pth'
        )

        cassie_dataset = self.dataset
        train_dataset, val_dataset, test_dataset = self.train_validate_test_split(cassie_dataset)

        # Create dataloaders
        train_loader = DataLoader(
            train_dataset,
            batch_size=self.batch_size,
            shuffle=self.shuffle,
            num_workers=self.num_workers
        )

        val_loader = DataLoader(
            val_dataset,
            batch_size=self.batch_size,
            shuffle=False,
            num_workers=self.num_workers
        )

        test_loader = DataLoader(
            test_dataset,
            batch_size=self.batch_size,
            shuffle=False,
            num_workers=self.num_workers
        )

        device = torch_utils.get_device()
        if self.model_type == 'UNet':
            model = UNet()
        elif self.model_type == 'ResidualLQRPredictionNet':
            model = ResidualLQRPredictionNet(7, 1)
        else:
            print("Failing loss function ", self.loss)
            raise ValueError('Invalid model type')
        model.to(device)

        # Choose the optimizer
        optimizer = torch.optim.Adam(model.parameters(), lr=self.lr) \
            if self.optimizer == 'Adam' else \
            torch.optim.SGD(model.parameters(), lr=self.lr)
        
        # Train the model
        device = torch_utils.get_device()

        best_val_loss = float('inf')  # Set to positive infinity initially

        for epoch in tqdm(range(self.num_epochs)):
            # Train the model
            train_loss = self.run_epoch(model, train_loader, device, optimizer, is_training=True)

            # Evaluate on validation set
            val_loss = self.run_epoch(model, val_loader, device, optimizer=None, is_training=False)

            # Logging for training and validation losses of current epoch if using WandB
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

            if epochs_since_best_validation_loss > self.patience:
                break

        # Testing phase
        # load the best model
        model.load_state_dict(torch.load(checkpoint_path))
        # test the data
        test_loss = self.run_epoch(model, test_loader, device, is_training=False)

        # Logging for test loss if using WandB
        if use_wandb:
            wandb.log({
                "test_loss": test_loss
            })

        if use_wandb:
            wandb.finish()



def argparsing():

    import configargparse
    parser = configargparse.ArgumentParser()

    parser.add_argument('--config', is_config_file=True, help='config file relative path. config file must be of type .yaml')


    # TODO: @sharanyashastry might want to add more args
    # experiment-level and learner params
    parser.add_argument('--use_wandb', type=bool, default=True, help='whether to use wandb for logging')
    parser.add_argument('--model_type', type=str, default='UNet', help='match model name to string in unet.py')
    parser.add_argument('--optimizer', type=str, default='adam', help='select optimizer type from [adam, ..]')  #What other optimizers are there?
    parser.add_argument('--shuffle', action='store_true', default=True, help='whether to shuffle the dataset')
    parser.add_argument('--num_workers', type=int, default=1, help='number of workers to use for dataloader')
    parser.add_argument('--split_ratio', nargs='+', type=float, default=[0.7, 0.15, 0.15], help='fraction of dataset to use for training, validation, and testing.')

    # parser.add_argument('--device', type=str, default='cuda', help='generic cuda device; specific GPU should be specified in CUDA_VISIBLE_DEVICES')
    # parser.add_argument('--load_trainval', action='store_true', help='whether to load the train/val split from the given checkpoint path')
    # parser.add_argument('--checkpoint_path', type=str, default=f'', help='absolute path to model checkpoint')
    parser.add_argument('--batch_size', type=int, default=32, help='batch size')
    parser.add_argument('--lr', type=float, default=1e-4, help='learning rate')
    parser.add_argument('--patience', type=int, default=10, help='number of epochs to wait for validation loss to improve before stopping training')
    parser.add_argument('--num_epochs', type=int, default=100, help='number of epochs to train for')
    parser.add_argument('--loss', type=str, default='mse', help='loss function to use')
    # parser.add_argument('--lr_warmup_epochs', type=int, default=5, help='number of epochs to warmup learning rate for')
    # parser.add_argument('--lr_decay', action='store_true', help='whether to use lr_decay, hardcoded to exponentially decay to 0.01 * lr by end of training')
    parser.add_argument('--save_model_freq', type=int, default=25, help='frequency with which to save model checkpoints')
    parser.add_argument('--val_freq', type=int, default=10, help='frequency with which to evaluate on validation set')    
    # parser.add_argument('--eval_plots_freq', type=int, default=0, help='frequency with which to generate evaluation plots')
    parser.add_argument('--run_name', type=str, default='sharanya-test', help='name of wandb run')
    # evaluationPlots args
    # parser.add_argument('--models', nargs='+', type=str, default=None, help='an unformatted list of model names to evaluate')

    args = parser.parse_args()
    print(f'[CONFIGARGPARSE] Parsing args from config file {args.config}')

    return args

if __name__ == '__main__':

    args = argparsing()
    print(args)

    learner = Learner(args)
    learner.train_and_test(use_wandb=True)