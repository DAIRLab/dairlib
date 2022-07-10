from torch import save

from pydairlib.cassie.cassie_gym.stepnet_data_generator import StepnetDataGenerator

NMAPS = 2
NSTEPS = 2


def collect_data_from_random_map(size):
    env = StepnetDataGenerator.make_randomized_env()
    data = []
    for i in range(size):
        data.append(env.get_stepnet_data_point())
    env.free_sim()
    return data


def main():
    for i in range(NMAPS):
        data = collect_data_from_random_map(NSTEPS)
        save(data, f'.learning_data/dataset/{i}.pt')


if __name__ == "__main__":
    main()
