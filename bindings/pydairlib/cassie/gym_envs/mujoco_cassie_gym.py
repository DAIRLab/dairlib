import numpy as np

from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import *
from pydrake.systems.analysis import Simulator

from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import *
from pydairlib.multibody import *
from pydairlib.systems.primitives import *
from dairlib import lcmt_cassie_out
from pydairlib.cassie.gym_envs.cassie_env_state import CassieEnvState, quat_to_rotation, \
    reexpress_state_local_to_global_omega, reexpress_state_global_to_local_omega
from pydairlib.cassie.mujoco.drake_to_mujoco_converter import DrakeToMujocoConverter
# from drake_to_mujoco_converter import DrakeToMujocoConverter

from pydairlib.cassie.mujoco.cassiemujoco import *
from pydairlib.cassie.mujoco.mujoco_lcm_utils import *
from pydairlib.cassie.mujoco.drake_to_mujoco_converter import DrakeToMujocoConverter

from pydairlib.systems import (RobotOutputSender, RobotOutputReceiver)

class MuJoCoCassieGym():
    def __init__(self, reward_func, visualize=False):
        self.sim_dt = 8e-5
        self.gym_dt = 1e-3
        self.visualize = visualize
        self.reward_func = reward_func
        # hardware logs are 50ms long and start approximately 5ms before impact
        # the simulator will check to make sure ground reaction forces are first detected within 3-7ms
        self.start_time = 0.00
        self.current_time = 0.00
        self.end_time = 7.5

        self.default_model_directory = '/home/yangwill/workspace/cassie-mujoco-sim/model/'
        self.default_model_file = '/home/yangwill/workspace/cassie-mujoco-sim/model/cassie.xml'
        self.cassie_in = cassie_user_in_t()
        self.cassie_out = cassie_out_t()
        self.cassie_out_lcm = lcmt_cassie_out()
        self.action_dim = 10
        self.state_dim = 45
        self.x_init = np.array(
            [1, 0, 0, 0, 0, 0, 0.85, -0.0358636, 0, 0.67432, -1.588, -0.0458742, 1.90918,
             -0.0381073, -1.82312, 0.0358636, 0, 0.67432, -1.588, -0.0457885, 1.90919, -0.0382424, -1.82321,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.prev_cassie_state = None
        self.controller = None
        self.terminated = False
        self.initialized = False
        self.actuator_index_map = {'hip_roll_left_motor': 0,
                                   'hip_yaw_left_motor': 1,
                                   'hip_pitch_left_motor': 2,
                                   'knee_left_motor': 3,
                                   'toe_left_motor': 4,
                                   'hip_roll_right_motor': 5,
                                   'hip_yaw_right_motor': 6,
                                   'hip_pitch_right_motor': 7,
                                   'knee_right_motor': 8,
                                   'toe_right_motor': 9}

    def make(self, controller, model_xml='cassie.xml'):
        self.builder = DiagramBuilder()
        self.dt = 8e-5
        # self.plant = MultibodyPlant(self.dt)
        self.controller = controller
        self.simulator = CassieSim(self.default_model_directory + model_xml)
        if self.visualize:
            self.cassie_vis = CassieVis(self.simulator)
        self.builder.AddSystem(self.controller)
        # self.robot_output_sender = RobotOutputSender(self.controller.get_plant(), True)
        self.robot_output_sender = RobotOutputSender(self.controller.get_plant(), False)
        self.builder.AddSystem(self.robot_output_sender)

        self.builder.Connect(self.robot_output_sender.get_output_port(), self.controller.get_state_input_port())
        self.drake_to_mujoco_converter = DrakeToMujocoConverter(self.sim_dt)

        self.diagram = self.builder.Build()
        self.sim = Simulator(self.diagram)
        self.robot_output_sender_context = self.diagram.GetMutableSubsystemContext(self.robot_output_sender,
                                                                          self.sim.get_mutable_context())
        self.controller_context = self.diagram.GetMutableSubsystemContext(self.controller,
                                                                          self.sim.get_mutable_context())
        self.controller_output_port = self.controller.get_torque_output_port()
        self.radio_input_port = self.controller.get_radio_input_port()
        self.sim.get_mutable_context().SetTime(self.start_time)
        self.reset()
        self.initialized = True

    def reset(self):
        # self.traj = CassieTraj()
        q_mujoco, v_mujoco = self.drake_to_mujoco_converter.convert_to_mujoco(
            reexpress_state_global_to_local_omega(self.x_init))
        mujoco_state = self.simulator.get_state()
        mujoco_state.set_qpos(q_mujoco)
        mujoco_state.set_qvel(v_mujoco)
        mujoco_state.set_time(self.start_time)
        self.simulator.set_state(mujoco_state)
        self.sim.get_mutable_context().SetTime(self.start_time)
        u = np.zeros(10)
        self.sim.Initialize()
        self.current_time = self.start_time
        self.prev_cassie_state = CassieEnvState(self.current_time, self.x_init, u, np.zeros(18))
        self.cassie_state = CassieEnvState(self.current_time, self.x_init, u, np.zeros(18))
        cassie_in, u_mujoco = self.pack_input(self.cassie_in, u)
        self.cassie_out = self.simulator.step(cassie_in)
        self.cassie_out_lcm = self.pack_cassie_out(self.cassie_out)
        self.terminated = False
        return

    def advance_to(self, time):
        while self.current_time < time and not self.terminated:
            self.step()
        return

    def check_termination(self):
        return self.cassie_state.get_fb_positions()[2] < 0.4

    def step(self, action=np.zeros(18)):
        if not self.initialized:
            print("Call make() before calling step() or advance()")

        next_timestep = self.sim.get_context().get_time() + self.gym_dt
        # action[2] = 0.75
        self.robot_output_sender.get_input_port_state().FixValue(self.robot_output_sender_context, self.cassie_state.x)
        self.radio_input_port.FixValue(self.controller_context, action)

        u = self.controller_output_port.Eval(self.controller_context)[:-1]  # remove the timestamp
        cassie_in, u_mujoco = self.pack_input(self.cassie_in, u)
        self.sim.AdvanceTo(next_timestep)
        while self.simulator.time() < next_timestep:
            self.simulator.step(cassie_in)
        if self.visualize:
            self.cassie_vis.draw(self.simulator)

        self.current_time = next_timestep
        t = self.simulator.time()
        q = self.simulator.qpos()
        v = self.simulator.qvel()
        q, v = self.drake_to_mujoco_converter.convert_to_drake(q, v)
        self.current_time = t
        x = np.hstack((q, v))
        x = reexpress_state_local_to_global_omega(x)
        self.cassie_state = CassieEnvState(self.current_time, x, u, action)
        reward = self.reward_func.compute_reward(self.cassie_state, self.prev_cassie_state)
        self.terminated = self.check_termination()
        self.prev_cassie_state = self.cassie_state
        return self.cassie_state, reward

    def pack_input(self, cassie_in, u_drake):
        act_map = self.drake_to_mujoco_converter.act_map
        # Set control parameters
        u_mujoco = np.zeros(10)
        for u_name in act_map:
            cassie_in.torque[self.actuator_index_map[u_name]] = u_drake[act_map[u_name]]
            u_mujoco[self.actuator_index_map[u_name]] = u_drake[act_map[u_name]]
        return cassie_in, u_mujoco

    def pack_cassie_out(self, cassie_out):
        cassie_out_lcm = lcmt_cassie_out()
        cassie_out_lcm.utime = self.current_time * 1e6
        cassie_out_lcm.pelvis.targetPc.etherCatStatus = cassie_out.pelvis.targetPc.etherCatStatus
        cassie_out_lcm.pelvis.targetPc.etherCatNotifications = cassie_out.pelvis.targetPc.etherCatNotifications
        cassie_out_lcm.pelvis.targetPc.taskExecutionTime = cassie_out.pelvis.targetPc.taskExecutionTime
        cassie_out_lcm.pelvis.targetPc.overloadCounter = cassie_out.pelvis.targetPc.overloadCounter
        cassie_out_lcm.pelvis.targetPc.cpuTemperature = cassie_out.pelvis.targetPc.cpuTemperature
        cassie_out_lcm.pelvis.battery.dataGood = cassie_out.pelvis.battery.dataGood
        cassie_out_lcm.pelvis.battery.stateOfCharge = cassie_out.pelvis.battery.stateOfCharge
        cassie_out_lcm.pelvis.battery.current = cassie_out.pelvis.battery.current
        cassie_out_lcm.pelvis.battery.voltage = cassie_out.pelvis.battery.voltage
        cassie_out_lcm.pelvis.battery.temperature = cassie_out.pelvis.battery.temperature
        cassie_out_lcm.pelvis.radio.radioReceiverSignalGood = cassie_out.pelvis.radio.radioReceiverSignalGood
        cassie_out_lcm.pelvis.radio.receiverMedullaSignalGood = cassie_out.pelvis.radio.receiverMedullaSignalGood
        cassie_out_lcm.pelvis.radio.channel = cassie_out.pelvis.radio.channel
        cassie_out_lcm.pelvis.radio.receiverMedullaSignalGood = cassie_out.pelvis.radio.receiverMedullaSignalGood
        cassie_out_lcm.pelvis.battery.temperature = cassie_out.pelvis.battery.temperature
        cassie_out_lcm.pelvis.vectorNav.dataGood = cassie_out.pelvis.vectorNav.dataGood
        cassie_out_lcm.pelvis.vectorNav.vpeStatus = cassie_out.pelvis.vectorNav.vpeStatus
        cassie_out_lcm.pelvis.vectorNav.pressure = cassie_out.pelvis.vectorNav.pressure
        cassie_out_lcm.pelvis.vectorNav.temperature = cassie_out.pelvis.vectorNav.temperature
        cassie_out_lcm.pelvis.vectorNav.magneticField = cassie_out.pelvis.vectorNav.magneticField
        cassie_out_lcm.pelvis.vectorNav.angularVelocity = cassie_out.pelvis.vectorNav.angularVelocity
        cassie_out_lcm.pelvis.vectorNav.linearAcceleration = cassie_out.pelvis.vectorNav.linearAcceleration
        cassie_out_lcm.pelvis.vectorNav.orientation = cassie_out.pelvis.vectorNav.orientation
        cassie_out_lcm.pelvis.medullaCounter = cassie_out.pelvis.medullaCounter
        cassie_out_lcm.pelvis.medullaCpuLoad = cassie_out.pelvis.medullaCpuLoad
        cassie_out_lcm.pelvis.bleederState = cassie_out.pelvis.bleederState
        cassie_out_lcm.pelvis.leftReedSwitchState = cassie_out.pelvis.leftReedSwitchState
        cassie_out_lcm.pelvis.rightReedSwitchState = cassie_out.pelvis.rightReedSwitchState
        cassie_out_lcm.pelvis.vtmTemperature = cassie_out.pelvis.vtmTemperature

        cassie_out_lcm.isCalibrated = cassie_out.isCalibrated
        cassie_out_lcm.pelvis.vectorNav.temperature = cassie_out.pelvis.vectorNav.temperature

        self.copy_leg(cassie_out_lcm.leftLeg, cassie_out.leftLeg)
        self.copy_leg(cassie_out_lcm.rightLeg, cassie_out.rightLeg)

        # import pdb; pdb.set_trace()

        # copy_vector_int16(cassie_out->messages,
        #                               message->messages, 4);

    def copy_leg(self, cassie_leg_out_lcm, cassie_leg_out):
        self.copy_elmo(cassie_leg_out_lcm.hipRollDrive, cassie_leg_out.hipRollDrive)
        self.copy_elmo(cassie_leg_out_lcm.hipYawDrive, cassie_leg_out.hipYawDrive)
        self.copy_elmo(cassie_leg_out_lcm.hipPitchDrive, cassie_leg_out.hipPitchDrive)
        self.copy_elmo(cassie_leg_out_lcm.kneeDrive, cassie_leg_out.kneeDrive)
        self.copy_elmo(cassie_leg_out_lcm.footDrive, cassie_leg_out.footDrive)
        cassie_leg_out_lcm.shinJoint.position = cassie_leg_out.shinJoint.position
        cassie_leg_out_lcm.shinJoint.velocity = cassie_leg_out.shinJoint.velocity
        cassie_leg_out_lcm.tarsusJoint.position = cassie_leg_out.tarsusJoint.position
        cassie_leg_out_lcm.tarsusJoint.velocity = cassie_leg_out.tarsusJoint.velocity
        cassie_leg_out_lcm.footJoint.position = cassie_leg_out.footJoint.position
        cassie_leg_out_lcm.footJoint.velocity = cassie_leg_out.footJoint.velocity
        cassie_leg_out_lcm.medullaCounter = cassie_leg_out.medullaCounter
        cassie_leg_out_lcm.medullaCpuLoad = cassie_leg_out.medullaCpuLoad
        cassie_leg_out_lcm.reedSwitchState = cassie_leg_out.reedSwitchState

    def copy_elmo(self, elmo_out_lcm, elmo_out):
        elmo_out_lcm.statusWord = elmo_out.statusWord
        elmo_out_lcm.position = elmo_out.position
        elmo_out_lcm.velocity = elmo_out.velocity
        elmo_out_lcm.torque = elmo_out.torque
        elmo_out_lcm.driveTemperature = elmo_out.driveTemperature
        elmo_out_lcm.dcLinkVoltage = elmo_out.dcLinkVoltage
        elmo_out_lcm.torqueLimit = elmo_out.torqueLimit
        elmo_out_lcm.gearRatio = elmo_out.gearRatio

    def free_sim(self):
        del self.cassie_env
