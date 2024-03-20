import numpy as np
import os
import datetime
def gather_theta_batched_branin_data(n_datapoints,
                                     batch_size,
                                     gains_to_randomize,
                                     thetas_to_randomize,
                                     high_level_folder,
                                     params_t=BraninFnParamsTrain):

  assert n_datapoints == int(n_datapoints / batch_size) * batch_size

  num_batches = int(n_datapoints / (batch_size))
  # Each batch is associated with a single set of system intrinsic parameters
  intrinsics = np.zeros((num_batches, len(thetas_to_randomize)))
  # for each parameter vector, generate $batch_size random gains to evaluate
  gains_to_test = np.zeros((num_batches, batch_size, len(gains_to_randomize)))

  subfolder = "branin_data_" + datetime.now().strftime("%b_%d_%Y_%H%M") + "/"
  if not os.path.exists(os.path.join(high_level_folder, subfolder)):
    os.makedirs(os.path.join(high_level_folder, subfolder), exist_ok=True)

  with h5py.File(os.path.join(high_level_folder, subfolder, "dataset.hdf5"), 'w') as f:
    # intrinsic vectors don't actually get used anywhere, but good to save for bookkeeping
    f.create_dataset("intrinsics", shape=intrinsics.shape)
    f.create_dataset("gains", shape=gains_to_test.shape)
    # this system only has 1 performance metric, but in principle this could be any number
    f.create_dataset("metrics", shape=(num_batches, batch_size, 1))
    for batch in range(num_batches):
      # Generate a random intrinsic vector for this batch
      batch_intrinsic_obj = params_t.generate_random(thetas_to_randomize)
      f["intrinsics"][batch,...] = batch_intrinsic_obj.get_list()
      robot = Branin(batch_intrinsic_obj, gains_to_randomize)
      for point in range(batch_size):
        # generate a random gain vector
        gain = BraninInputs.generate_random(gains_to_randomize).get_list()[gains_to_randomize]
        # run the simulation and obtain the performance metrics
        metric = robot.evaluate_x(gain)
        f["metrics"][batch,point] = metric
        f["gains"][batch,point] = gain
      print(f"finished batch {batch}")