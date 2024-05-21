## Learning perceptive locomotion for Cassie

This is an environment to support some initial work doing perceptive learning on 
Cassie. A lot of the infrastructure builds off of what we built for Humanoids 2023, 
"Bipedal Walking on Constrained Footholds with MPC Footstep Control"  
For questions about infrastructure and setup, reach out to
[@Brian-Acosta](github.com/Brian-Acosta), [@min-ku](github.com/min-ku). 

### Ground rules:
1. To keep this workspace tidy, any temporary files like logs, data, etc. should be saved to the tmp subdirectory, which is already part of the global dairlib .gitignore on this branch.

### Pipeline
1. Collect residual data by running alip_lqr_data_collection.py
```
bazel-bin/bindings/pydairlib/perceptive_locomotion/perception_learning/alip_lqr_data_collection
```
2. Change the collected data name to data.npz and train the U-Net in /inference/ using either train.py or train_and_test.py
```
bazel-bin/bindings/pydairlib/perceptive_locomotion/perception_learning/inference/train_and_test
```
3. Collect Expert Trajectory datasets [actions, observations] by running alip_nn_data_collection.py. Note that collecting bad trajectories and training through supervised learning makes the policy sub-obtimal.
```
bazel-bin/bindings/pydairlib/perceptive_locomotion/perception_learning/alip_nn_data_collection
```
4. Run imitation.py (PPO/RPO) until convergence
```
bazel-bin/bindings/pydairlib/perceptive_locomotion/perception_learning/imitation
```
5. Train using PPO/RPO through train.py.
```
bazel-bin/bindings/pydairlib/perceptive_locomotion/perception_learning/train
```

### Run policy
Sample the policy through run_sample.py
```
bazel-bin/bindings/pydairlib/perceptive_locomotion/perception_learning/run_sample --model_path {path}.zip
```
