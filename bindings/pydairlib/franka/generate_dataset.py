from franka_env import *
from yaml import *
from datetime import datetime
import h5py

# gains are controller params
# thetas/intrinsics are environment params
def generate_dataset(n_datapoints,
                     batch_size,
                     gains_to_randomize,
                     thetas_to_randomize,
                     high_level_folder):
  assert n_datapoints == int(n_datapoints / batch_size) * batch_size

  num_batches = int(n_datapoints / (batch_size))
  # Each batch is associated with a single set of system intrinsic parameters
  intrinsics = np.zeros((num_batches, len(thetas_to_randomize)))
  # for each parameter vector, generate $batch_size random gains to evaluate
  gains_to_test = np.zeros((num_batches, batch_size, len(gains_to_randomize)))

  subfolder = "occam_data" + datetime.now().strftime("%b_%d_%Y_%H%M") + "/"
  if not os.path.exists(os.path.join(high_level_folder, subfolder)):
    os.makedirs(os.path.join(high_level_folder, subfolder), exist_ok=True)

  with h5py.File(os.path.join(high_level_folder, subfolder, "dataset.hdf5"), 'w') as f:
    # intrinsic vectors don't actually get used anywhere, but good to save for bookkeeping
    f.create_dataset("intrinsics", shape=intrinsics.shape)
    f.create_dataset("gains", shape=gains_to_test.shape)
    # this system only has 1 performance metric, but in principle this could be any number
    f.create_dataset("metrics", shape=(num_batches, batch_size, 1))
    # f.create_dataset("states", shape=(num_batches, batch_size, 1))
    for batch in range(num_batches):
      # Generate a random intrinsic vector for this batch
      # batch_intrinsic_obj = params_t.generate_random(thetas_to_randomize)
      f["intrinsics"][batch, ...] = thetas_to_randomize
      # robot = Branin(batch_intrinsic_obj, gains_to_randomize)
      for point in range(batch_size):
        # generate a random gain vector
        # gain = BraninInputs.generate_random(gains_to_randomize).get_list()[gains_to_randomize]
        gain = 2 * np.random.random(gains_to_randomize.shape[0]) - 1  # center around 0. [-1, 1]
        # run the simulation and obtain the performance metrics
        # metric = robot.evaluate_x(gain)
        metric, states = run_sim(thetas_to_randomize, gain)
        f["metrics"][batch, point] = metric
        f["gains"][batch, point] = gain
        # f["states"][batch, point] = states
      print(f"finished batch {batch}")


if __name__ == '__main__':
  hersh_dataset_params_file = 'bindings/pydairlib/franka/parameters/dataset_params.yaml'
  params = yaml_load(filename=hersh_dataset_params_file)
  generate_dataset(50 * 64, 64, np.ones(5), np.zeros(5), params['high_level_folder'])
