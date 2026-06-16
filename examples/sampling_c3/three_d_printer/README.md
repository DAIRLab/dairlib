# Push Anything

This is an implementation of our paper currently available on Arxiv; it is accepted to IEEE International Robotics and Automation (ICRA) 2026.

[Project webpage](https://dairlab.github.io/push-anything/) · [ArXiv](https://arxiv.org/abs/2510.19974)

## Abstract

Non-prehensile manipulation of diverse objects remains a core challenge in robotics, driven by unknown physical properties and the complexity of contact-rich interactions. Recent advances in contact-implicit model predictive control (CI-MPC), with contact reasoning embedded directly in the trajectory optimization, have shown promise in tackling the task efficiently and robustly. However, demonstrations have been limited to narrowly curated examples. In this work, we showcase the broader capabilities of CI-MPC through precise planar pushing tasks over a wide range of object geometries, including multi-object domains. These scenarios demand reasoning over numerous inter-object and object-environment contacts to strategically manipulate and de-clutter the environment, which was intractable for prior CI-MPC methods. To achieve this, we introduce Consensus Complementarity Control Plus (C3+), an enhanced CI-MPC algorithm integrated into a complete pipeline spanning object scanning, mesh reconstruction, and hardware execution. Compared to its predecessor C3, C3+ achieves substantially faster solve times, enabling real-time performance even in multi-object pushing tasks. On hardware, our system achieves overall 98% success rate across 33 objects, reaching pose goals within tight tolerances. The average time-to-goal is approximately 0.5, 1.6, 3.2, and 5.3 minutes for 1-, 2-, 3-, and 4-object tasks, respectively. Project page: [https://dairlab.github.io/push-anything/](https://dairlab.github.io/push-anything/).

## BibTeX

```
@inproceedings{Bui2026,
 title = {Push Anything: Single- and Multi-Object Pushing From First Sight with Contact-Implicit MPC},
 author = {Bui, Hien and Gao, Yufeiyang and Yang, Haoran and Cui, Eric and Mody, Siddhant and Acosta, Brian and Felix, Thomas Stephen and Bianchini, Bibit and Posa, Michael},
 year = {2026},
 website = {https://dairlab.github.io/push-anything/},
 arxiv = {2510.19974},
 booktitle = {To appear in the IEEE International Conference on Robotics and Automation (ICRA) 2026}
}
```

## Simulation setup

Development environment and simulation workflow for the `anything` demo (`--demo_name=anything`). Parameters live in `parameters/`; procman scripts are `franka_sim_anything.pmd` (sim) and `franka_hardware_anything.pmd` (hardware).

## Clone dairlib

```bash
git clone https://github.com/DAIRLab/dairlib.git
cd dairlib
git checkout push_anything_dev
```

All commands below are run from the repository root (`dairlib/`).

## Run the container

**First time** (creates container `push-anything-container`):

```bash
xhost +local:docker

docker compose -f examples/sampling_c3/docker/docker-compose.yml run --name push-anything-container push-anything-container-launch-service
```

With a custom checkout path:

```bash
DAIRLIB_HOST_PATH=/path/to/dairlib \
  docker compose -f examples/sampling_c3/docker/docker-compose.yml run --name push-anything-container push-anything-container-launch-service
```

**Later** (reattach to the same container; also use this after a host reboot):

```bash
xhost +local:docker
docker start -ai push-anything-container
```

## Build dairlib (must be inside the container)

```bash
cd /home/pushanything/dairlib
bazel build ...
```

A full `bazel build ...` takes on the order of **30 minutes** on an Intel i9-14900K; expect longer on slower CPUs.

## Run simulation

From the repository root inside the container (`/home/pushanything/dairlib`):

### 1. Choose objects

Scene objects are set by `base_names` in [`parameters/sampling_c3_controller_params.yaml`](parameters/sampling_c3_controller_params.yaml). Each entry is a folder name under `examples/sampling_c3/urdf/` (for example, `G_shape_video` maps to `examples/sampling_c3/urdf/G_shape_video/`).

Keep one active `base_names` line and comment out the rest. The list length sets how many objects appear in the scene (1–4 are supported):

```yaml
# base_names: [Y_shape_video, A_shape_video, N_shape_video, G_shape_video]
base_names: [G_shape_video, R_shape_texture, A_shape_video, S_shape_texture]
```

When you start an experiment, `multi_rewrite` reads `base_names` and updates the related configs automatically (object models, initial poses, goal poses, and C3+ tuning for the object count).

### 2. Launch procman

```bash
bot-procman-sheriff -l examples/sampling_c3/anything/franka_sim_anything.pmd
```

### 3. Start the visualizer

In the procman window, start **`visualizer`** under the `operator` group, then open [http://localhost:7000](http://localhost:7000) in a browser for Meshcat.

### 4. Start the experiment

Run **`script:start_experiment_no_logs`** from the procman menu (top bar). This script:

1. Runs `multi_rewrite` to apply your `base_names` selection.
2. Restarts the simulation stack:
   - `franka_sim` — Drake simulator (publishes state over LCM).
   - `franka_osc` — task-space controller that tracks MPC trajectories.
   - `sampling_c3` — C3+ contact-implicit MPC planner.

Use **`script:start_experiment_with_logs`** instead if you also want LCM logging.

### 5. Stop the experiment

Run **`script:end_experiment`** to stop the simulator and controllers.
