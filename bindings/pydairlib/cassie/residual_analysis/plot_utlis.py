import wandb

class PlotViewlizer():

    def __init__(self, project_name):
        wandb.init(project=project_name)
    def add_info(self, log):
        wandb.log(log)