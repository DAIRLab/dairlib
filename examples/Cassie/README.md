## Starting bot-procman-sheriff
Launch bot-procman-sheriff with either `examples/Cassie/cassie_simulation.pmd` or `examples/Cassie/cassie_hardware.pmd` 
depending on whether you are testing in simulation or on hardware.

Note: You must be in the root `dairlib` directory for this to work properly.
```
bot-procman-sheriff -l examples/Cassie/cassie_simulation.pmd
```

## Quick tips on bot-procman
Details about bot-procman: https://github.com/libbot2/libbot2/tree/master/bot2-procman

Keyboard shortcuts:
 - Ctrl + s to start a process
 - Ctrl + t to stop a process
 - Ctrl + r to restart a process
 - Can select multiple processes together to start or stop them together
 - Enter to edit a process
 
 Scripts
 - procman has the ability to write a script to start/stop multiple processes in order. THey are located under the scripts tab.     

## Drake visualizer/directory GUI
`drake-director` is primary GUI, and also the source for the PD controller panel and pose/trajectory visualization.
`state-visualizer` in its many variants will draw Cassie's pose on the `drake-director` GUI according to the state published by either `CASSIE_STATE_SIMULATION
` or `CASSIE_STATE_DISPATCHER`.

## Simulators
We currently use the Drake simulator and MuJoCo simulator. Agility robotics also provides a Gazebo simulator for Cassie but there are no benefits of using
the Gazebo simulator over the Drake or Mujoco simulator.
 
**Drake simulator**
- The Drake simulator can be started using either the `simulator-fixed` or `simulator-floating` processes.
- See the gflags in `multibody_sim.cc` for possible options

**MuJoCo simulator**
- In order to run the mujoco simulator, you first have to install it from `https://github.com/DAIRLab/cassie-mujoco-sim/tree/output_contact`
- The `mujoco200` branch also works but the feet contact forces will not be published
- The simulator can be started by running the compiled `cassie-mujoco-sim/test/cassiesim` process. The command is under the `other-simulators` group in
 `cassie_simulation.pmd`.
- Flags:
    - -v: run with the mujoco GUI
    - -s: publish the current state over LCM
    - -r: run in realtime or at the current realtime rate set in `cassie_sim.c`.
    - See `cassiesim.c` for a full list of options.
- Note: You must run `dispatcher-robot-in` in addition to the controllers to communicate with the mujoco simulator    

## Controllers
We currently have several controller implemented for cassie. A PD controller for the joints, as well as several operational space based controllers for
tracking some reference motions.

PD controller
- `pd-controller`: Best used with the fixed-base simulator

OSC controllers
- `osc-standing-controller`: Keeps the center of mass at the center of the four contact points at the desired height.  
- `osc-walking-controller`: Tracks a walking trajectory generated with a LIP-based center of mass trajectory and a Raibert based footstep planning controller
for the swing foot
- `osc-jumping-controller`: WIP. Tracks a jumping trajectory from trajectory optimization.
    

## Dispatchers
Dispatchers are the processes that we use to translate the LCM messages used by the controllers and visualization to the UDP messages used by Cassie.

`dispatcher-robot-out`
- This processes translates the UDP message **out**putted from Cassie (robot-out) into `lcmt_cassie_out` and `lcmt_robot_output`. 
- `lcmt_cassie_out` contains the raw sensor data from cassie 
- `lcmt_robot_output` contains cassie's state after state estimation and packages it in a neat `q, v, u` + IMU state vector.
- While the dispatcher is intended to be used on hardware, the state estimator can be tested with the simulators as well.

`dispatcher-robot-in`
- This process translates the desired **in**puts calculated from the active controller to UDP commands sent to cassie (robot-in).  
- This is also how we send input commands to the mujoco simulator.

## lcm-spy
lcm-spy is a GUI that lists all the LCM messages being published. Useful for debugging purposes.



