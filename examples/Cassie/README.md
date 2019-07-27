## PD controller tests
Launch bot-procman-sherrif using the cassie_test.pmd configuration file.
Currently, you must bein the root `dairlib` directory for this to work properly.
```
bot-procman-sherrif examples/Cassie/cassie_test.pmd
```
Note: if you've installed libbot2, using apt, `bot-procman-sherrif` is probably located in `/opt/libbot2/0.0.1.20180130/bin/` or a similar directory.
Running `bot-procman-sherrif -l` will allow you to skip the "Spawn local deputy" step.

### Local Gazebo simulation (with dispatcher)
The following steps will launch a PD controller and simulation:
1. Options->Spawn local deputy (allows launching of processes)
2. Right-click and start `drake-director` and `state-visualizer`. It's a good idea to allow Director to open before launching the visualizer.
3. Right-click and start `pd-controller` and `dispatcher`
4. Start the Gazebo simulation

### Mujoco simulation
You will first need to install Mujoco and the Cassie simulator. Instructions for installation are here: https://github.com/DAIRLab/cassie-mujoco-sim

**It is recommended that you use the mujoco200 branch**

The default Agility simulation has been modified in a few ways. Most critically, it also outputs LCM messages alongside the UDP message.
The simulation can be run via the `cassie_test.pmd` procman script.

See `cassiesim.c` for a full list of options.

### Gazebo simulation
You will first need to install Gazebo and the Cassie simulator. Instructions for installation are here: https://github.com/DAIRLab/cassie-gazebo-sim

Note: you will need to `cmake ../plugins` if you put the `build` directory inside the root `cassie-gazebo-sim`

1. `source /usr/share/gazebo/setup.sh` 
2. `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<WORKSPACE>/cassie-gazebo-sim/`
3. `export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:<WORKSPACE>/cassie-gazebo-sim/build`
4. `export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:<WORKSPACE>/cassie-gazebo-sim/build`
5. From the `cassie-gazebo-sim/cassie` directory, run `gazebo cassie.world`

### Local Drake simulation (no dispatcher)
The following steps will launch a PD controller and simulation:
1. Options->Spawn local deputy (allows launching of processes)
2. Right-click and start `drake-director` and `state-visualizer`. It's a good idea to allow Director to open before launching the visualizer.
3. Right-click and start `pd-controller` and `simulator`